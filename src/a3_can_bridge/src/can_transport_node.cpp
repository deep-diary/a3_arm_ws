#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "a3_can_bridge/frame_codec.hpp"
#include "a3_can_bridge/protocol_codec.hpp"
#include "a3_can_bridge/socketcan_driver.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;
using std_msgs::msg::UInt8MultiArray;

namespace a3_can_bridge
{

class CanTransportNode : public rclcpp::Node
{
public:
  CanTransportNode()
  : Node("can_transport_node")
  {
    can0_name_ = this->declare_parameter<std::string>("can0_name", "can0");
    can1_name_ = this->declare_parameter<std::string>("can1_name", "can1");
    rx_poll_ms_ = this->declare_parameter<int>("rx_poll_ms", 2);
    max_tx_per_cycle_ = this->declare_parameter<int>("max_tx_per_cycle", 8);
    tx_queue_max_ = this->declare_parameter<int>("tx_queue_max", 2048);
    max_retry_per_frame_ = this->declare_parameter<int>("max_retry_per_frame", 20);
    publish_hex_lines_ = this->declare_parameter<bool>("publish_hex_lines", false);
    hex_tx_line_topic_ =
      this->declare_parameter<std::string>("hex_tx_line_topic", "/can_frames_tx_line");
    hex_rx_line_topic_ =
      this->declare_parameter<std::string>("hex_rx_line_topic", "/can_frames_rx_line");

    const int can_tx_qos_depth = this->declare_parameter<int>("can_tx_frames_qos_depth", 4000);
    rclcpp::QoS qos_can_tx(static_cast<size_t>(std::max(1, can_tx_qos_depth)));
    qos_can_tx.reliable();
    tx_sub_ = this->create_subscription<UInt8MultiArray>(
      "/can_tx_frames", qos_can_tx,
      std::bind(&CanTransportNode::OnTxFrame, this, std::placeholders::_1));
    rx_pub_ = this->create_publisher<UInt8MultiArray>("/can_rx_frames", rclcpp::SensorDataQoS());
    hex_tx_line_pub_ =
      this->create_publisher<String>(hex_tx_line_topic_, rclcpp::SensorDataQoS());
    hex_rx_line_pub_ =
      this->create_publisher<String>(hex_rx_line_topic_, rclcpp::SensorDataQoS());

    if (!driver_.Open(can0_name_, can1_name_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open SocketCAN interfaces [%s, %s]. Keep retrying...",
        can0_name_.c_str(), can1_name_.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "SocketCAN opened: can0=%s can1=%s", can0_name_.c_str(), can1_name_.c_str());
    }

    if (publish_hex_lines_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Hex line preview: TX=%s RX=%s (std_msgs/String, high rate when active)",
        hex_tx_line_topic_.c_str(), hex_rx_line_topic_.c_str());
    }

    poll_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(rx_poll_ms_),
      std::bind(&CanTransportNode::PollRx, this));
    stats_timer_ = this->create_wall_timer(
      5s,
      std::bind(&CanTransportNode::PrintStats, this));
  }

private:
  struct PendingFrame
  {
    CanFrameMessage frame;
    int retries{0};
  };

  static bool IsRetryableSendErrno(int err)
  {
    return err == EAGAIN || err == ENOBUFS || err == EINTR;
  }

  void OnTxFrame(const UInt8MultiArray::SharedPtr msg)
  {
    const auto frame = FrameCodec::Unpack(*msg);
    if (!frame.has_value()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignore malformed /can_tx_frames message (expect 15 bytes)");
      return;
    }
    EnqueueFrame(frame.value());
  }

  void PollRx()
  {
    if (!driver_.IsOpen() && !driver_.Open(can0_name_, can1_name_)) {
      return;
    }

    FlushTxQueue();
    PollOneBus(CanBus::CAN0);
    PollOneBus(CanBus::CAN1);
  }

  void EnqueueFrame(const CanFrameMessage & frame)
  {
    ++tx_enqueued_total_;
    if (frame.bus == CanBus::CAN0) {
      ++tx_enqueued_can0_total_;
    } else {
      ++tx_enqueued_can1_total_;
    }
    if (static_cast<int>(tx_queue_can0_.size() + tx_queue_can1_.size()) >= tx_queue_max_) {
      ++tx_drop_overflow_total_;
      DropOldestForOverflow();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TX queue overflow, dropping oldest frame (queue_max=%d)", tx_queue_max_);
    }
    if (frame.bus == CanBus::CAN0) {
      tx_queue_can0_.push_back(PendingFrame{frame, 0});
    } else {
      tx_queue_can1_.push_back(PendingFrame{frame, 0});
    }
  }

  void DropOldestForOverflow()
  {
    if (tx_queue_can0_.empty() && tx_queue_can1_.empty()) {
      return;
    }
    if (tx_queue_can0_.empty()) {
      tx_queue_can1_.pop_front();
      return;
    }
    if (tx_queue_can1_.empty()) {
      tx_queue_can0_.pop_front();
      return;
    }
    // Prefer dropping from the longer queue to keep both buses balanced.
    if (tx_queue_can0_.size() >= tx_queue_can1_.size()) {
      tx_queue_can0_.pop_front();
    } else {
      tx_queue_can1_.pop_front();
    }
  }

  int FlushOneBusQueue(std::deque<PendingFrame> & queue, int budget)
  {
    int sent = 0;
    while (!queue.empty() && sent < budget) {
      auto & pending = queue.front();
      const bool is_can0 = (pending.frame.bus == CanBus::CAN0);
      const bool sent_ok = driver_.Send(pending.frame);
      const int err = driver_.LastErrno();
      const long io_size = driver_.LastIoSize();
      if (sent_ok) {
        ++tx_send_ok_total_;
        if (is_can0) {
          ++tx_ok_can0_total_;
        } else {
          ++tx_ok_can1_total_;
        }
        if (pending.retries > 0) {
          ++tx_retry_success_total_;
          if (is_can0) {
            ++tx_retry_success_can0_total_;
          } else {
            ++tx_retry_success_can1_total_;
          }
        }
        PublishHexLine(true, pending.frame);
        RCLCPP_DEBUG(
          this->get_logger(), "TX %s", ProtocolCodec::FrameSummary(pending.frame).c_str());
        queue.pop_front();
        ++sent;
        continue;
      }

      ++tx_send_fail_total_;
      if (is_can0) {
        ++tx_send_fail_can0_total_;
      } else {
        ++tx_send_fail_can1_total_;
      }
      if (IsRetryableSendErrno(err)) {
        ++tx_retryable_fail_total_;
        if (is_can0) {
          ++tx_retryable_fail_can0_total_;
        } else {
          ++tx_retryable_fail_can1_total_;
        }
        ++pending.retries;
        if (pending.retries == 1) {
          ++tx_retry_frame_total_;
          if (is_can0) {
            ++tx_retry_frame_can0_total_;
          } else {
            ++tx_retry_frame_can1_total_;
          }
        }
        if (pending.retries > max_retry_per_frame_) {
          ++tx_drop_retry_exceeded_total_;
          if (is_can0) {
            ++tx_drop_retry_exceeded_can0_total_;
          } else {
            ++tx_drop_retry_exceeded_can1_total_;
          }
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "SocketCAN send failed retry-exceeded: retries=%d errno=%d(%s) io_size=%ld frame=%s",
            pending.retries,
            err,
            std::strerror(err),
            io_size,
            ProtocolCodec::FrameSummary(pending.frame).c_str());
          queue.pop_front();
          continue;
        }
        // Retryable error: stop this bus in this cycle, but allow the other bus to proceed.
        break;
      }

      ++tx_drop_non_retryable_total_;
      if (is_can0) {
        ++tx_drop_non_retryable_can0_total_;
      } else {
        ++tx_drop_non_retryable_can1_total_;
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "SocketCAN send failed non-retryable: errno=%d(%s) io_size=%ld frame=%s",
        err,
        std::strerror(err),
        io_size,
        ProtocolCodec::FrameSummary(pending.frame).c_str());
      queue.pop_front();
    }
    return sent;
  }

  void FlushTxQueue()
  {
    if (max_tx_per_cycle_ <= 0) {
      return;
    }
    const int half_budget = std::max(1, max_tx_per_cycle_ / 2);
    const int budget0 = half_budget;
    const int budget1 = max_tx_per_cycle_ - half_budget;

    int sent = 0;
    sent += FlushOneBusQueue(tx_queue_can0_, budget0);
    sent += FlushOneBusQueue(tx_queue_can1_, budget1);

    int remaining = max_tx_per_cycle_ - sent;
    while (remaining > 0) {
      int extra = 0;
      extra += FlushOneBusQueue(tx_queue_can0_, 1);
      if (extra == 0) {
        extra += FlushOneBusQueue(tx_queue_can1_, 1);
      }
      if (extra == 0) {
        break;
      }
      sent += extra;
      remaining = max_tx_per_cycle_ - sent;
    }
  }

  void PollOneBus(CanBus bus)
  {
    for (int i = 0; i < 64; ++i) {
      const auto frame = driver_.Receive(bus);
      if (!frame.has_value()) {
        break;
      }
      auto msg = FrameCodec::Pack(frame.value());
      rx_pub_->publish(msg);
      if (bus == CanBus::CAN0) {
        ++rx_can0_total_;
      } else {
        ++rx_can1_total_;
      }
      PublishHexLine(false, frame.value());
      RCLCPP_DEBUG(this->get_logger(), "RX %s", ProtocolCodec::FrameSummary(frame.value()).c_str());
    }
  }

  void PublishHexLine(bool is_tx, const CanFrameMessage & frame)
  {
    if (!publish_hex_lines_) {
      return;
    }
    String line;
    line.data = ProtocolCodec::FrameLineHex(is_tx, frame);
    if (is_tx) {
      hex_tx_line_pub_->publish(line);
    } else {
      hex_rx_line_pub_->publish(line);
    }
  }

  void PrintStats()
  {
    const uint64_t d_enq = tx_enqueued_total_ - last_tx_enqueued_total_;
    const uint64_t d_enq0 = tx_enqueued_can0_total_ - last_tx_enqueued_can0_total_;
    const uint64_t d_enq1 = tx_enqueued_can1_total_ - last_tx_enqueued_can1_total_;
    const uint64_t d_ok = tx_send_ok_total_ - last_tx_send_ok_total_;
    const uint64_t d_fail = tx_send_fail_total_ - last_tx_send_fail_total_;
    const uint64_t d_retry_fail = tx_retryable_fail_total_ - last_tx_retryable_fail_total_;
    const uint64_t d_retry_ok = tx_retry_success_total_ - last_tx_retry_success_total_;
    const uint64_t d_retry_frames = tx_retry_frame_total_ - last_tx_retry_frame_total_;
    const uint64_t d_drop_ov = tx_drop_overflow_total_ - last_tx_drop_overflow_total_;
    const uint64_t d_drop_nr = tx_drop_non_retryable_total_ - last_tx_drop_non_retryable_total_;
    const uint64_t d_drop_re = tx_drop_retry_exceeded_total_ - last_tx_drop_retry_exceeded_total_;
    const uint64_t d_fail0 = tx_send_fail_can0_total_ - last_tx_send_fail_can0_total_;
    const uint64_t d_fail1 = tx_send_fail_can1_total_ - last_tx_send_fail_can1_total_;
    const uint64_t d_retry_fail0 =
      tx_retryable_fail_can0_total_ - last_tx_retryable_fail_can0_total_;
    const uint64_t d_retry_fail1 =
      tx_retryable_fail_can1_total_ - last_tx_retryable_fail_can1_total_;
    const uint64_t d_retry_ok0 =
      tx_retry_success_can0_total_ - last_tx_retry_success_can0_total_;
    const uint64_t d_retry_ok1 =
      tx_retry_success_can1_total_ - last_tx_retry_success_can1_total_;
    const uint64_t d_retry_frame0 =
      tx_retry_frame_can0_total_ - last_tx_retry_frame_can0_total_;
    const uint64_t d_retry_frame1 =
      tx_retry_frame_can1_total_ - last_tx_retry_frame_can1_total_;
    const uint64_t d_drop_nr0 =
      tx_drop_non_retryable_can0_total_ - last_tx_drop_non_retryable_can0_total_;
    const uint64_t d_drop_nr1 =
      tx_drop_non_retryable_can1_total_ - last_tx_drop_non_retryable_can1_total_;
    const uint64_t d_drop_re0 =
      tx_drop_retry_exceeded_can0_total_ - last_tx_drop_retry_exceeded_can0_total_;
    const uint64_t d_drop_re1 =
      tx_drop_retry_exceeded_can1_total_ - last_tx_drop_retry_exceeded_can1_total_;

    last_tx_enqueued_total_ = tx_enqueued_total_;
    last_tx_enqueued_can0_total_ = tx_enqueued_can0_total_;
    last_tx_enqueued_can1_total_ = tx_enqueued_can1_total_;
    last_tx_send_ok_total_ = tx_send_ok_total_;
    last_tx_send_fail_total_ = tx_send_fail_total_;
    last_tx_retryable_fail_total_ = tx_retryable_fail_total_;
    last_tx_retry_success_total_ = tx_retry_success_total_;
    last_tx_retry_frame_total_ = tx_retry_frame_total_;
    last_tx_drop_overflow_total_ = tx_drop_overflow_total_;
    last_tx_drop_non_retryable_total_ = tx_drop_non_retryable_total_;
    last_tx_drop_retry_exceeded_total_ = tx_drop_retry_exceeded_total_;
    last_tx_send_fail_can0_total_ = tx_send_fail_can0_total_;
    last_tx_send_fail_can1_total_ = tx_send_fail_can1_total_;
    last_tx_retryable_fail_can0_total_ = tx_retryable_fail_can0_total_;
    last_tx_retryable_fail_can1_total_ = tx_retryable_fail_can1_total_;
    last_tx_retry_success_can0_total_ = tx_retry_success_can0_total_;
    last_tx_retry_success_can1_total_ = tx_retry_success_can1_total_;
    last_tx_retry_frame_can0_total_ = tx_retry_frame_can0_total_;
    last_tx_retry_frame_can1_total_ = tx_retry_frame_can1_total_;
    last_tx_drop_non_retryable_can0_total_ = tx_drop_non_retryable_can0_total_;
    last_tx_drop_non_retryable_can1_total_ = tx_drop_non_retryable_can1_total_;
    last_tx_drop_retry_exceeded_can0_total_ = tx_drop_retry_exceeded_can0_total_;
    last_tx_drop_retry_exceeded_can1_total_ = tx_drop_retry_exceeded_can1_total_;

    const uint64_t d_tx0 = tx_ok_can0_total_ - last_tx_ok_can0_total_;
    const uint64_t d_tx1 = tx_ok_can1_total_ - last_tx_ok_can1_total_;
    const uint64_t d_rx0 = rx_can0_total_ - last_rx_can0_total_;
    const uint64_t d_rx1 = rx_can1_total_ - last_rx_can1_total_;
    last_tx_ok_can0_total_ = tx_ok_can0_total_;
    last_tx_ok_can1_total_ = tx_ok_can1_total_;
    last_rx_can0_total_ = rx_can0_total_;
    last_rx_can1_total_ = rx_can1_total_;

    const double fail_rate = (d_ok + d_fail) > 0 ?
      (100.0 * static_cast<double>(d_fail) / static_cast<double>(d_ok + d_fail)) : 0.0;
    const double retry_frame_ok_rate = d_retry_frames > 0 ?
      (100.0 * static_cast<double>(d_retry_ok) / static_cast<double>(d_retry_frames)) : 0.0;
    std::ostringstream fail_rate_ss;
    fail_rate_ss << std::fixed << std::setprecision(2) << fail_rate << "%";
    std::ostringstream retry_frame_ok_rate_ss;
    retry_frame_ok_rate_ss << std::fixed << std::setprecision(2) << retry_frame_ok_rate << "%";

    RCLCPP_INFO(
      this->get_logger(),
      "TX stats(5s): enq=%lu ok=%lu fail=%lu retry_fail=%lu retry_ok=%lu "
      "drop_overflow=%lu drop_non_retry=%lu drop_retry_exceeded=%lu queue=%zu "
      "fail_rate=%s retry_frame_ok_rate=%s",
      d_enq, d_ok, d_fail, d_retry_fail, d_retry_ok,
      d_drop_ov, d_drop_nr, d_drop_re, tx_queue_can0_.size() + tx_queue_can1_.size(),
      fail_rate_ss.str().c_str(), retry_frame_ok_rate_ss.str().c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "TX enqueue bus split(5s): enq_can0=%lu enq_can1=%lu (ratio=%.3f)",
      d_enq0, d_enq1,
      d_enq1 > 0 ? (static_cast<double>(d_enq0) / static_cast<double>(d_enq1)) : 0.0);

    const int64_t mirror_0 =
      static_cast<int64_t>(d_tx0) - static_cast<int64_t>(d_rx1);
    const int64_t mirror_1 =
      static_cast<int64_t>(d_tx1) - static_cast<int64_t>(d_rx0);
    RCLCPP_INFO(
      this->get_logger(),
      "CAN bus counts(5s): tx_can0=%lu tx_can1=%lu rx_can0=%lu rx_can1=%lu | "
      "two-wire ping-pong check (expect ~0): (tx_can0-rx_can1)=%lld "
      "(tx_can1-rx_can0)=%lld",
      d_tx0, d_tx1, d_rx0, d_rx1,
      static_cast<long long>(mirror_0), static_cast<long long>(mirror_1));
    RCLCPP_INFO(
      this->get_logger(),
      "TX bus health(5s): "
      "can0{fail=%lu retry_fail=%lu retry_ok=%lu retry_frames=%lu drop_non_retry=%lu drop_retry_exceeded=%lu} "
      "can1{fail=%lu retry_fail=%lu retry_ok=%lu retry_frames=%lu drop_non_retry=%lu drop_retry_exceeded=%lu}",
      d_fail0, d_retry_fail0, d_retry_ok0, d_retry_frame0, d_drop_nr0, d_drop_re0,
      d_fail1, d_retry_fail1, d_retry_ok1, d_retry_frame1, d_drop_nr1, d_drop_re1);
  }

  std::string can0_name_;
  std::string can1_name_;
  int rx_poll_ms_{2};
  int max_tx_per_cycle_{8};
  int tx_queue_max_{2048};
  int max_retry_per_frame_{20};
  bool publish_hex_lines_{false};
  std::string hex_tx_line_topic_;
  std::string hex_rx_line_topic_;

  SocketCanDriver driver_;
  std::deque<PendingFrame> tx_queue_can0_;
  std::deque<PendingFrame> tx_queue_can1_;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr tx_sub_;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr rx_pub_;
  rclcpp::Publisher<String>::SharedPtr hex_tx_line_pub_;
  rclcpp::Publisher<String>::SharedPtr hex_rx_line_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  uint64_t tx_enqueued_total_{0};
  uint64_t tx_enqueued_can0_total_{0};
  uint64_t tx_enqueued_can1_total_{0};
  uint64_t tx_send_ok_total_{0};
  uint64_t tx_send_fail_total_{0};
  uint64_t tx_send_fail_can0_total_{0};
  uint64_t tx_send_fail_can1_total_{0};
  uint64_t tx_retryable_fail_total_{0};
  uint64_t tx_retryable_fail_can0_total_{0};
  uint64_t tx_retryable_fail_can1_total_{0};
  uint64_t tx_retry_success_total_{0};
  uint64_t tx_retry_success_can0_total_{0};
  uint64_t tx_retry_success_can1_total_{0};
  uint64_t tx_retry_frame_total_{0};
  uint64_t tx_retry_frame_can0_total_{0};
  uint64_t tx_retry_frame_can1_total_{0};
  uint64_t tx_drop_overflow_total_{0};
  uint64_t tx_drop_non_retryable_total_{0};
  uint64_t tx_drop_non_retryable_can0_total_{0};
  uint64_t tx_drop_non_retryable_can1_total_{0};
  uint64_t tx_drop_retry_exceeded_total_{0};
  uint64_t tx_drop_retry_exceeded_can0_total_{0};
  uint64_t tx_drop_retry_exceeded_can1_total_{0};

  uint64_t last_tx_enqueued_total_{0};
  uint64_t last_tx_enqueued_can0_total_{0};
  uint64_t last_tx_enqueued_can1_total_{0};
  uint64_t last_tx_send_ok_total_{0};
  uint64_t last_tx_send_fail_total_{0};
  uint64_t last_tx_send_fail_can0_total_{0};
  uint64_t last_tx_send_fail_can1_total_{0};
  uint64_t last_tx_retryable_fail_total_{0};
  uint64_t last_tx_retryable_fail_can0_total_{0};
  uint64_t last_tx_retryable_fail_can1_total_{0};
  uint64_t last_tx_retry_success_total_{0};
  uint64_t last_tx_retry_success_can0_total_{0};
  uint64_t last_tx_retry_success_can1_total_{0};
  uint64_t last_tx_retry_frame_total_{0};
  uint64_t last_tx_retry_frame_can0_total_{0};
  uint64_t last_tx_retry_frame_can1_total_{0};
  uint64_t last_tx_drop_overflow_total_{0};
  uint64_t last_tx_drop_non_retryable_total_{0};
  uint64_t last_tx_drop_non_retryable_can0_total_{0};
  uint64_t last_tx_drop_non_retryable_can1_total_{0};
  uint64_t last_tx_drop_retry_exceeded_total_{0};
  uint64_t last_tx_drop_retry_exceeded_can0_total_{0};
  uint64_t last_tx_drop_retry_exceeded_can1_total_{0};

  uint64_t tx_ok_can0_total_{0};
  uint64_t tx_ok_can1_total_{0};
  uint64_t rx_can0_total_{0};
  uint64_t rx_can1_total_{0};
  uint64_t last_tx_ok_can0_total_{0};
  uint64_t last_tx_ok_can1_total_{0};
  uint64_t last_rx_can0_total_{0};
  uint64_t last_rx_can1_total_{0};
};

}  // namespace a3_can_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<a3_can_bridge::CanTransportNode>());
  rclcpp::shutdown();
  return 0;
}
