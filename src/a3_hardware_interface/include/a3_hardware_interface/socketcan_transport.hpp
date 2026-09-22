#ifndef A3_HARDWARE_INTERFACE__SOCKETCAN_TRANSPORT_HPP_
#define A3_HARDWARE_INTERFACE__SOCKETCAN_TRANSPORT_HPP_

#include <mutex>
#include <string>

#include "a3_hardware_interface/motor_model.hpp"

namespace a3_hardware_interface
{

// Single-bus SocketCAN transport (SOCK_RAW/CAN_RAW), modeled on the official
// robstride_can_driver: blocking receive with timeout, send mutex +
// ENOBUFS/EAGAIN retries, feedback-only hardware filter.
class SocketcanTransport
{
public:
  SocketcanTransport() = default;
  ~SocketcanTransport();

  SocketcanTransport(const SocketcanTransport &) = delete;
  SocketcanTransport & operator=(const SocketcanTransport &) = delete;

  bool Open(const std::string & interface, std::string * error);
  void Close();
  bool IsOpen() const {return fd_ >= 0;}

  // Returns false on timeout/error; *frame valid only when returning true.
  bool Receive(CanFrameMessage * frame);
  bool Send(const CanFrameMessage & frame, std::string * error);

private:
  int fd_{-1};
  std::mutex tx_mutex_;
};

}  // namespace a3_hardware_interface

#endif  // A3_HARDWARE_INTERFACE__SOCKETCAN_TRANSPORT_HPP_
