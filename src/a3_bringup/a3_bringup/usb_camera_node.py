#!/usr/bin/env python3
"""USB 摄像头（V4L2）→ sensor_msgs/Image 发布节点。

背景：本机（RK3588 + rockchip-multimedia PPA）`apt` 的 ros-humble-usb-cam /
v4l2-camera / gscam 均硬依赖 libopencv-dev → libav*-dev，而该 PPA 的 pin 文件
把 libav*-dev 钉在优先级 1（与 rkmpp 版 ffmpeg 冲突），无法安装。
因此直接用系统已装的 python3-opencv(cv2) 采集并发布，运行依赖仅 rclpy + cv2。

发布话题（配合命名空间使用）：
  <namespace>/image_raw    sensor_msgs/Image  (bgr8)
  <namespace>/camera_info  sensor_msgs/CameraInfo
"""

import array
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# 像素格式名 → OpenCV FOURCC
FOURCC = {
    "mjpg": "MJPG",
    "yuyv": "YUYV",
}


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__("usb_cam")

        self.declare_parameter("video_device", "/dev/video0")
        self.declare_parameter("frame_id", "camera_optical_frame")
        # 默认 640x480@25：消息 921KB（720p 是 2.7MB），重载板上消费端更容易吃满；
        # 要 30fps 需切 1280x720（设备唯一 30fps 模式，见 LL-168「性能」）。
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("framerate", 25.0)
        self.declare_parameter("pixel_format", "mjpg")
        # 在画面左上角烧"接收帧率"（本机订阅端实测的 DDS 投递速率）。
        # F194 起帧率统一由 vision_display_mux 在 /a3/vision/display 右上角显示，
        # 这里默认关，避免文字压进 /camera/image_raw 再被人脸标注叠一层。
        self.declare_parameter("overlay_fps", False)

        device = self.get_parameter("video_device").value
        self.frame_id = self.get_parameter("frame_id").value
        width = self.get_parameter("image_width").value
        height = self.get_parameter("image_height").value
        framerate = self.get_parameter("framerate").value
        pixel_format = str(self.get_parameter("pixel_format").value).lower()
        self.overlay_fps = bool(self.get_parameter("overlay_fps").value)
        self._rx = 0
        self._rx_fps = 0.0
        self._rx_t0 = time.monotonic()

        # 图像走 sensor-data 惯例（BEST_EFFORT + 只留最新）：消费者（RViz / 后续处理节点）
        # 偶发变慢时不累积积压、也不会拖住发布端；RViz Image display 的 Topic 需同样选
        # Best Effort。调试用 `ros2 topic hz --qos-reliability best_effort`。
        sensor_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub_image = self.create_publisher(Image, "image_raw", sensor_qos)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", sensor_qos)

        if self.overlay_fps:
            # raw=True：只计数收到的原始字节，不反序列化成 Image → 开销远小于正常订阅
            self.create_subscription(
                Image, "image_raw", self._on_rx, sensor_qos, raw=True
            )
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id

        self._stop = threading.Event()
        self._thread = None
        self.cap = None

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            # 相机是可选外设：打不开就空转，不能让整栈起不来（真机栈默认带本节点）
            self.get_logger().error(
                f"无法打开摄像头设备 {device}（不存在 / 被占用？）：本节点空转，"
                "其余节点不受影响；可改 camera_device 参数或 use_camera:=false 关掉"
            )
            return

        if pixel_format in FOURCC:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC[pixel_format]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        cap.set(cv2.CAP_PROP_FPS, float(framerate))
        self.cap = cap

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS) or framerate

        self.camera_info.width = actual_w
        self.camera_info.height = actual_h

        self._thread = threading.Thread(
            target=self._capture_loop, name="usb_cam_capture", daemon=True
        )
        self._thread.start()

        self.get_logger().info(
            f"摄像头就绪：{device} {actual_w}x{actual_h} @ {actual_fps:.1f}fps "
            f"({pixel_format})"
        )

    def _on_rx(self, _msg):
        self._rx += 1

    def _annotate_fps(self, frame):
        """在画面左上角画**接收帧率**（本机订阅端实测的投递速率，每 0.5s 刷新）。

        注意：这是 DDS 投递到订阅端的速率，不是 RViz 的渲染/显示帧率——后者看
        RViz 窗口右下角状态栏（软件渲染重场景下可能只有个位数）。
        """
        now = time.monotonic()
        dt = now - self._rx_t0
        if dt >= 0.5:
            self._rx_fps = self._rx / dt
            self._rx = 0
            self._rx_t0 = now

        text = f"recv {self._rx_fps:.1f} fps"
        scale = max(0.6, frame.shape[1] / 1600.0)
        thickness = max(1, int(round(scale * 2)))
        org = (int(12 * scale * 1.5), int(38 * scale * 1.5))
        # 先画黑色描边再画绿色，浅色/深色画面下都看得清
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0),
                    thickness + 3, cv2.LINE_AA)
        cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 0),
                    thickness, cv2.LINE_AA)

    def _capture_loop(self):
        # 采集与发布放在独立线程；节点本身无订阅/定时器，主线程不参与搬运
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warning(
                    "读取帧失败（设备是否被占用 / 拔出？）", throttle_duration_sec=5.0
                )
                time.sleep(0.05)
                continue

            if self.overlay_fps:
                self._annotate_fps(frame)

            stamp = self.get_clock().now().to_msg()
            height, width = frame.shape[:2]

            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height = height
            msg.width = width
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = width * 3
            # 必须用 array.array('B')：直接赋 bytes 会走 rclpy 逐元素转换，
            # 640x480 一帧要 ~150ms（掉到 ~6fps），array.array 只要 ~0.1ms。
            msg.data = array.array("B", frame.tobytes())
            self.pub_image.publish(msg)

            self.camera_info.header.stamp = stamp
            self.pub_info.publish(self.camera_info)

    def destroy_node(self):
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
