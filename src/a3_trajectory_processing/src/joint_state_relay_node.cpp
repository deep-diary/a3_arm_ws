// 负载优化：/joint_states 降频中转（200 Hz → 50 Hz）。
//
// 背景：Humble 的 joint_state_broadcaster 不支持 publish_rate，/joint_states 跟随
// controller_manager update_rate（200 Hz）。控制链（JTC/servo）需要 200 Hz，但
// 诊断/显示/FK/重力前馈等消费者只需要 20–50 Hz；每个 Python 订阅者都要为 200 Hz
// 付执行器唤醒开销（实测单节点 25–35% 单核）。
//
// 本节点用一次 C++ 订阅把最新一帧按 rate_hz 转发到 /joint_states_lite，低频消费者
// 改订 lite 话题即可。输入停止时不再转发（seq 不变），保证下游新鲜度语义不变。
//
// 参数：
//   input_topic       默认 /joint_states
//   output_topic      默认 /joint_states_lite
//   rate_hz           默认 50.0
//   min_rate_hz       原始输入频率下限（诊断用，默认 180.0）
//   max_rate_hz       原始输入频率上限（诊断用，默认 220.0）
//   diag_period_s     诊断发布周期（默认 5.0；0=关闭）
//   publish_diagnostics 是否发布 /diagnostics（默认 true）

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class JointStateRelay : public rclcpp::Node
{
public:
  JointStateRelay()
  : rclcpp::Node("joint_state_relay")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/joint_states");
    output_topic_ = declare_parameter<std::string>("output_topic", "/joint_states_lite");
    const double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    min_rate_hz_ = declare_parameter<double>("min_rate_hz", 180.0);
    max_rate_hz_ = declare_parameter<double>("max_rate_hz", 220.0);
    const double diag_period = declare_parameter<double>("diag_period_s", 5.0);
    const bool diag_enabled = declare_parameter<bool>("publish_diagnostics", true);

    auto in_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic_, in_qos,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        last_ = msg;
        ++seq_;
        rx_stamps_.push_back(now_seconds());
        prune();
      });
    pub_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, out_qos);

    const double period = 1.0 / std::max(1.0, rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      [this]() {
        if (last_ && seq_ != published_seq_) {
          pub_->publish(*last_);
          published_seq_ = seq_;
        }
      });

    if (diag_enabled && diag_period > 0.0) {
      diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
      diag_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(diag_period)),
        [this]() { publish_diagnostics(); });
    }

    RCLCPP_INFO(
      get_logger(),
      "joint_state_relay ready: %s -> %s @ %.1f Hz (raw band %.1f-%.1f Hz)",
      input_topic_.c_str(), output_topic_.c_str(), rate_hz, min_rate_hz_, max_rate_hz_);
  }

private:
  static double now_seconds()
  {
    return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  }

  void prune()
  {
    const double now = now_seconds();
    while (!rx_stamps_.empty() && rx_stamps_.front() < now - 5.0) {
      rx_stamps_.pop_front();
    }
  }

  void publish_diagnostics()
  {
    const double now = now_seconds();
    prune();
    // 最近 5 s 窗口内的平均频率（首帧不足窗口时按实际跨度）
    double rate = 0.0;
    if (rx_stamps_.size() >= 2) {
      const double span = std::max(1e-3, now - rx_stamps_.front());
      rate = static_cast<double>(rx_stamps_.size() - 1) / span;
    }
    const bool ok = rate >= min_rate_hz_ && rate <= max_rate_hz_;

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "a3_joint_state_relay: raw " + input_topic_;
    st.hardware_id = "a3-arm";
    st.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = ok ? "OK" : "Frequency out of band";
    auto add = [&st](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        st.values.push_back(kv);
      };
    add("Actual frequency (Hz)", std::to_string(rate));
    add("Minimum acceptable frequency (Hz)", std::to_string(min_rate_hz_));
    add("Maximum acceptable frequency (Hz)", std::to_string(max_rate_hz_));
    add("Output topic", output_topic_);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_rate_hz_{180.0};
  double max_rate_hz_{220.0};
  uint64_t seq_{0};
  uint64_t published_seq_{0};
  sensor_msgs::msg::JointState::SharedPtr last_;
  std::deque<double> rx_stamps_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateRelay>());
  rclcpp::shutdown();
  return 0;
}
