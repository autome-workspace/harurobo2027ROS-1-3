#include "harurobo_base/socketcan_receiver.hpp"

#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>

namespace harurobo_base
{
namespace
{
void check(int rc, const char * operation)
{
  if (rc < 0) {
    throw std::system_error(errno, std::generic_category(), operation);
  }
}
}  // namespace

SocketCanReceiver::~SocketCanReceiver()
{
  close();
}

void SocketCanReceiver::open(const std::string & interface, bool enable_fd)
{
  if (interface.empty() || interface.size() >= IFNAMSIZ ||
    interface.find('\0') != std::string::npos)
  {
    throw std::invalid_argument("invalid CAN interface name");
  }
  close();
  const int fd = ::socket(AF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
  check(fd, "socket(AF_CAN)");
  try {
    struct ifreq request {};
    std::memcpy(request.ifr_name, interface.c_str(), interface.size() + 1);
    check(::ioctl(fd, SIOCGIFINDEX, &request), "SIOCGIFINDEX");
    const int index = request.ifr_ifindex;
    check(::ioctl(fd, SIOCGIFFLAGS, &request), "SIOCGIFFLAGS");
    if (!(request.ifr_flags & IFF_UP)) {
      throw std::system_error(ENETDOWN, std::generic_category(), "CAN interface is not UP");
    }
    check(::ioctl(fd, SIOCGIFMTU, &request), "SIOCGIFMTU");
    if (request.ifr_mtu != CAN_MTU && request.ifr_mtu != CANFD_MTU) {
      throw std::invalid_argument("interface does not have a Classic CAN or CAN FD MTU");
    }
    if (enable_fd && request.ifr_mtu != CANFD_MTU) {
      throw std::invalid_argument("CAN FD requested but interface MTU is not CANFD_MTU");
    }
    if (enable_fd) {
      const int on = 1;
      check(::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)),
        "CAN_RAW_FD_FRAMES");
    }
    const can_err_mask_t error_mask = CAN_ERR_MASK;
    check(::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)),
      "CAN_RAW_ERR_FILTER");
    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = index;
    check(::bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)), "CAN bind");
    socket_ = fd;
  } catch (...) {
    ::close(fd);
    throw;
  }
}

void SocketCanReceiver::close() noexcept
{
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

SocketCanReceiver::ReadResult SocketCanReceiver::receive() const noexcept
{
  using State = ReadResult::State;
  ReadResult result{State::frame};
  const auto size = ::recv(socket_, &result.frame, sizeof(result.frame), MSG_TRUNC);
  if (size >= 0) {
    result.size = static_cast<std::size_t>(size);
    return result;
  }
  const int error = errno;
  if (error == EAGAIN || error == EWOULDBLOCK) {
    return {State::empty};
  }
  if (error == EINTR) {
    return {State::interrupted};
  }
  return {State::failed, {}, 0, error};
}
}  // namespace harurobo_base
