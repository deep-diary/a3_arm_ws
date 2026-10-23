# LL-181：人脸检测接入三坑（venv + PYTHONPATH 顺序 / 近距 6 点 PnP 镜像解 / YuNet 版本耦合）

## 现象

接 F183 人脸跟踪时，三个问题都会让人以为"算法不行"，实际都是环境或建模问题：

1. **`import mediapipe` 失败**：
   `ImportError: cannot import name 'one_of' from 'pyparsing' (/usr/lib/python3/dist-packages/pyparsing.py)`
   ——即使 venv 里已装 `pyparsing 3.3.3`，报错里出现的仍是系统那份 `2.4.7`。
2. **pose 忽正忽负**：手写 6 点模型 + `cv2.solvePnP` 在同一段正脸视频上给出
   yaw ≈ +47° / −47° 两套解（两帧之间符号翻转），重投影误差 8–12 px，控制根本没法用。
3. **换用 OpenCV YuNet**：`face_detection_yunet_2023mar.onnx` 在 cv2 4.5.4 上直接抛
   `cv2.error: (-204) Layer with requested id=-1 not found in function 'getLayerData'`。

## 根因

1. **`a3_shell_env.sh` 把 `/usr/lib/python3/dist-packages` 放在 `PYTHONPATH` 最前**，
   venv 的 `site-packages` 排在它后面 → 系统 `pyparsing 2.4.7` 压过 venv 的 3.3.3。
   mediapipe 的导入链是 `mediapipe → python.solutions.drawing_styles → drawing_utils → matplotlib`，
   matplotlib 需要 `pyparsing>=3` 的 `one_of`。**不能用 `env -u PYTHONPATH` 绕过**
   （那样连 `/opt/ros/humble` 一起丢，rclpy 直接没了）。
2. 6 个点（双眼外角/鼻尖/嘴角/下巴）在人脸上接近共面，近距离（<0.5 m）+ 少点 + 内参不准时
   PnP 有**镜像歧义**，SQPNP/ITERATIVE 会给两组等价解；再叠上手工 3D 模型与真实人脸比例不符
   （下巴/嘴宽/鼻凸）就放大成 ±47°。
3. YuNet 2023mar 是**动态输入 shape**，OpenCV 4.5.4 的 DNN 不支持 → 必须用 fixed-shape 的
   2022mar 及更早；本机 cv2 由 apt 固定 4.5.4（rkmpp pin 挡了升级路径）。

## 正确做法

1. **venv 前置而不是清空 PYTHONPATH**：
   ```bash
   PYTHONPATH="$HOME/.a3/venv/face_track/lib/python3.10/site-packages:$PYTHONPATH" \
       ~/.a3/venv/face_track/bin/python -m a3_perception.face_detect_node
   ```
   `launch/face_track.launch.py` 已用 `ExecuteProcess(additional_env={PYTHONPATH:[venv_site,":",$PYTHONPATH]})`
   固化这个顺序；只跑离线脚本时才用 `env -u PYTHONPATH`。
2. **姿态改用 mediapipe 自带 `output_facial_transformation_matrixes`**：正脸帧 yaw 稳定在 ±6° 内，
   与画面朝向一致（实测右转 +57°、左转 −54°）。roll 用双眼连线在图像里求，避免欧拉角符号陷阱。
   深度仍需一次单点标定（规范模型单位 ≠ mm）。
3. YuNet 若要试：**用 2022mar 模型**（`opencv_zoo` 的 `c97242ce` 提交），并注意它在
   cv2 4.5.4 上 640×480 要 85 ms、320×240 才 17.7 ms（成本 ∝ 输入面积）；
   mediapipe 同样 640×480 只要 32 ms（输入固定 192/256），本机选媒体管道无需犹豫。

## 相关路径

- `src/a3_perception/launch/face_track.launch.py`（venv 前置 + ExecuteProcess）
- `src/a3_perception/a3_perception/face_detect_node.py`（`pose_from_matrix`、One-Euro、coasting）
- `scripts/setup_face_track_env.sh`（建 venv / 装 mediapipe / 下模型）
- `scripts/face_landmark_overlay.py`（离线视频验证与深度标定读数）
- `docs/edge/requirements/F183-face-tracking-servo.md`（实测数据与设计取舍）
