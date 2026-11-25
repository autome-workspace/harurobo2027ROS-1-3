#ifndef HARUROBO_BASE__SOCKETCAN_RECEIVER_HPP_
#define HARUROBO_BASE__SOCKETCAN_RECEIVER_HPP_

#include <linux/can.h>
#include <linux/can/error.h>

#include <cstddef>
#include <string>

namespace harurobo_base
{
enum class FrameKind { classic, fd, remote_request, error, invalid };

inline FrameKind classify_frame(const canfd_frame & frame, std::size_t size)
{
  if (size == CAN_MTU && frame.len <= CAN_MAX_DLEN) {
    if (frame.can_id & CAN_ERR_FLAG) {
      return frame.len == CAN_ERR_DLC && !(frame.can_id & (CAN_RTR_FLAG | CAN_EFF_FLAG)) ?
        FrameKind::error : FrameKind::invalid;
    }
    return frame.can_id & CAN_RTR_FLAG ? FrameKind::remote_request : FrameKind::classic;
  }
  if (size == CANFD_MTU && frame.len <= CANFD_MAX_DLEN &&
    !(frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)))
  {
    return FrameKind::fd;
  }
  return FrameKind::invalid;
}

class SocketCanReceiver
{
public:
  struct ReadResult
  {
    enum class State { frame, empty, interrupted, failed };
    State state;
    canfd_frame frame{};
    std::size_t size{};
    int error{};
  };

  SocketCanReceiver() = default;
  ~SocketCanReceiver();
  SocketCanReceiver(const SocketCanReceiver &) = delete;
  SocketCanReceiver & operator=(const SocketCanReceiver &) = delete;

  void open(const std::string & interface, bool enable_fd);
  void close() noexcept;
  bool is_open() const noexcept {return socket_ >= 0;}
  ReadResult receive() const noexcept;

private:
  int socket_{-1};
};
}  // namespace harurobo_base

#endif  // HARUROBO_BASE__SOCKETCAN_RECEIVER_HPP_
