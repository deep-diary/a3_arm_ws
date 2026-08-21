#pragma once

#include <array>
#include <optional>
#include <string>

#include "a3_can_bridge/motor_model.hpp"

namespace a3_can_bridge
{

class SocketCanDriver
{
public:
  SocketCanDriver() = default;
  ~SocketCanDriver();

  bool Open(const std::string & can0_name, const std::string & can1_name);
  void Close();

  bool Send(const CanFrameMessage & frame);
  std::optional<CanFrameMessage> Receive(CanBus bus);

  bool IsOpen() const;
  int LastErrno() const;
  long LastIoSize() const;

private:
  int OpenOne(const std::string & ifname);
  static bool SetNonBlocking(int fd);

  std::array<int, 2> sockets_{{-1, -1}};
  int last_errno_{0};
  long last_io_size_{0};
};

}  // namespace a3_can_bridge
