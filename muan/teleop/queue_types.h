#ifndef MUAN_TELEOP_QUEUE_TYPES_H_
#define MUAN_TELEOP_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/teleop/joystick.pb.h"

namespace muan {
namespace teleop {

using JoystickStatusProto = muan::proto::StackProto<JoystickStatus, 512>;

using JoystickStatusQueue = muan::queues::MessageQueue<JoystickStatusProto>;

using XBoxRumbleProto = muan::proto::StackProto<XBoxRumble, 512>;

using XBoxRumbleQueue = muan::queues::MessageQueue<XBoxRumbleProto>;
}  // namespace teleop
}  // namespace muan

#endif  // MUAN_TELEOP_QUEUE_TYPES_H_
