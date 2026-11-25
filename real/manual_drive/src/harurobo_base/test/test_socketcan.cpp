#include <stdexcept>

#include "gtest/gtest.h"
#include "harurobo_base/socketcan_receiver.hpp"

namespace
{
using harurobo_base::classify_frame;
using harurobo_base::FrameKind;
using harurobo_base::SocketCanReceiver;

TEST(SocketCan, ClassifiesClassicStandardExtendedAndRemoteRequest)
{
  canfd_frame frame{};
  frame.can_id = 0x123;
  frame.len = 8;
  EXPECT_EQ(classify_frame(frame, CAN_MTU), FrameKind::classic);
  frame.can_id = CAN_EFF_FLAG | 0x1234567;
  EXPECT_EQ(classify_frame(frame, CAN_MTU), FrameKind::classic);
  frame.can_id |= CAN_RTR_FLAG;
  EXPECT_EQ(classify_frame(frame, CAN_MTU), FrameKind::remote_request);
  EXPECT_EQ(classify_frame(frame, CANFD_MTU), FrameKind::invalid);
}

TEST(SocketCan, DistinguishesBusOffFromData)
{
  canfd_frame frame{};
  frame.can_id = CAN_ERR_FLAG | CAN_ERR_BUSOFF;
  frame.len = CAN_ERR_DLC;
  EXPECT_EQ(classify_frame(frame, CAN_MTU), FrameKind::error);
  EXPECT_EQ(classify_frame(frame, CANFD_MTU), FrameKind::invalid);
  frame.len = 0;
  EXPECT_EQ(classify_frame(frame, CAN_MTU), FrameKind::invalid);
}

TEST(SocketCan, ClassifiesFdAndRejectsInvalidLengths)
{
  canfd_frame frame{};
  frame.can_id = 0x123;
  frame.len = 64;
  EXPECT_EQ(classify_frame(frame, CANFD_MTU), FrameKind::fd);
  EXPECT_EQ(classify_frame(frame, CAN_MTU), FrameKind::invalid);
  EXPECT_EQ(classify_frame(frame, CANFD_MTU + 1), FrameKind::invalid);
  frame.len = 65;
  EXPECT_EQ(classify_frame(frame, CANFD_MTU), FrameKind::invalid);
}

TEST(SocketCan, InvalidInterfaceNamesDoNotOpenSocket)
{
  SocketCanReceiver receiver;
  EXPECT_THROW(receiver.open("", false), std::invalid_argument);
  EXPECT_THROW(receiver.open(std::string(64, 'x'), false), std::invalid_argument);
  EXPECT_FALSE(receiver.is_open());
}
}  // namespace
