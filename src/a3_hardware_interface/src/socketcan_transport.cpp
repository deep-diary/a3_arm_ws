#include "a3_hardware_interface/socketcan_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>
#include <unistd.h>

namespace a3_hardware_interface
{

namespace
{
constexpr int kTxRetries = 2;
constexpr auto kTxRetryDelay = std::chrono::microseconds(50);
}  // namespace

SocketcanTransport::~SocketcanTransport()
{
  Close();
}

bool SocketcanTransport::Open(const std::string & interface, std::string * error)
{
  Close();

  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    if (error) {
      *error = "socket(PF_CAN) failed: " + std::string(std::strerror(errno));
    }
    return false;
  }

  // Feedback-only hardware filter: cmd type at bits 24-28 of the 29-bit EFF id.
  // Pass type 2 (0x02) and type 24 active report (0x18), drop TX loopback of
  // our own control/enable frames in-kernel.
  constexpr uint32_t kCmdMask = CAN_EFF_FLAG | (0x1Fu << 24);
  can_filter filters[2];
  filters[0].can_id = CAN_EFF_FLAG | (0x02u << 24);
  filters[0].can_mask = kCmdMask;
  filters[1].can_id = CAN_EFF_FLAG | (0x18u << 24);
  filters[1].can_mask = kCmdMask;
  if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters)) < 0) {
    if (error) {
      *error = "setsockopt(CAN_RAW_FILTER) failed: " + std::string(std::strerror(errno));
    }
    Close();
    return false;
  }

  // Receive timeout 100 ms (RX loop stays responsive during shutdown).
  timeval rx_tv{};
  rx_tv.tv_sec = 0;
  rx_tv.tv_usec = 100000;
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &rx_tv, sizeof(rx_tv));

  // Send timeout 10 ms + enlarged send buffer.
  timeval tx_tv{};
  tx_tv.tv_sec = 0;
  tx_tv.tv_usec = 10000;
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tx_tv, sizeof(tx_tv));
  const int sndbuf = 8192;
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    if (error) {
      *error = "ioctl(SIOCGIFINDEX " + interface +
               ") failed: " + std::string(std::strerror(errno));
    }
    Close();
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    if (error) {
      *error = "bind(" + interface + ") failed: " + std::string(std::strerror(errno));
    }
    Close();
    return false;
  }

  return true;
}

void SocketcanTransport::Close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketcanTransport::Receive(CanFrameMessage * frame)
{
  can_frame cf{};
  const ssize_t n = ::read(fd_, &cf, sizeof(cf));
  if (n != sizeof(cf)) {
    return false;  // timeout or error; caller retries
  }
  if ((cf.can_id & CAN_EFF_FLAG) == 0) {
    return false;
  }
  frame->is_extended = true;
  frame->can_id = cf.can_id & CAN_EFF_MASK;
  frame->dlc = cf.can_dlc;
  std::memcpy(frame->data.data(), cf.data, 8);
  return true;
}

bool SocketcanTransport::Send(const CanFrameMessage & frame, std::string * error)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);

  can_frame cf{};
  cf.can_id = (frame.can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  cf.can_dlc = frame.dlc;
  std::memcpy(cf.data, frame.data.data(), 8);

  for (int attempt = 0; attempt <= kTxRetries; ++attempt) {
    const ssize_t n = ::write(fd_, &cf, sizeof(cf));
    if (n == sizeof(cf)) {
      return true;
    }
    if (errno != ENOBUFS && errno != EAGAIN) {
      break;
    }
    if (attempt < kTxRetries) {
      std::this_thread::sleep_for(kTxRetryDelay);
    }
  }

  if (error) {
    *error = "CAN write failed: " + std::string(std::strerror(errno));
  }
  return false;
}

}  // namespace a3_hardware_interface
