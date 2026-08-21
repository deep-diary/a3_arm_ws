#include "a3_can_bridge/socketcan_driver.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace a3_can_bridge
{

SocketCanDriver::~SocketCanDriver()
{
  Close();
}

bool SocketCanDriver::Open(const std::string & can0_name, const std::string & can1_name)
{
  Close();
  sockets_[0] = OpenOne(can0_name);
  sockets_[1] = OpenOne(can1_name);
  return sockets_[0] >= 0 && sockets_[1] >= 0;
}

void SocketCanDriver::Close()
{
  for (auto & fd : sockets_) {
    if (fd >= 0) {
      close(fd);
      fd = -1;
    }
  }
}

bool SocketCanDriver::Send(const CanFrameMessage & frame)
{
  last_errno_ = 0;
  last_io_size_ = 0;
  const size_t idx = frame.bus == CanBus::CAN0 ? 0u : 1u;
  const int fd = sockets_[idx];
  if (fd < 0) {
    last_errno_ = ENODEV;
    return false;
  }

  can_frame raw{};
  raw.can_id = frame.can_id;
  if (frame.is_extended) {
    raw.can_id |= CAN_EFF_FLAG;
  }
  raw.can_dlc = frame.dlc;
  std::memcpy(raw.data, frame.data.data(), frame.data.size());

  const auto n = write(fd, &raw, sizeof(raw));
  last_io_size_ = static_cast<long>(n);
  if (n < 0) {
    last_errno_ = errno;
    return false;
  }
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    last_errno_ = EMSGSIZE;
    return false;
  }
  return true;
}

std::optional<CanFrameMessage> SocketCanDriver::Receive(CanBus bus)
{
  const size_t idx = bus == CanBus::CAN0 ? 0u : 1u;
  const int fd = sockets_[idx];
  if (fd < 0) {
    return std::nullopt;
  }

  can_frame raw{};
  const auto n = read(fd, &raw, sizeof(raw));
  if (n < 0) {
    return std::nullopt;
  }
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    return std::nullopt;
  }

  CanFrameMessage frame;
  frame.bus = bus;
  frame.is_extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame.can_id = raw.can_id & CAN_EFF_MASK;
  frame.dlc = raw.can_dlc;
  std::memcpy(frame.data.data(), raw.data, frame.data.size());
  return frame;
}

bool SocketCanDriver::IsOpen() const
{
  return sockets_[0] >= 0 && sockets_[1] >= 0;
}

int SocketCanDriver::LastErrno() const
{
  return last_errno_;
}

long SocketCanDriver::LastIoSize() const
{
  return last_io_size_;
}

int SocketCanDriver::OpenOne(const std::string & ifname)
{
  const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    return -1;
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    close(fd);
    return -1;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }

  if (!SetNonBlocking(fd)) {
    close(fd);
    return -1;
  }

  return fd;
}

bool SocketCanDriver::SetNonBlocking(int fd)
{
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) {
    return false;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

}  // namespace a3_can_bridge
