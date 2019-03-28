#ifndef MUAN_WPILIB_QUEUE_TYPES_H_
#define MUAN_WPILIB_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/wpilib/state.pb.h"

namespace muan {
namespace wpilib {

using DriverStationProto = muan::proto::StackProto<DriverStationStatus, 256>;
using DriverStationQueue = muan::queues::MessageQueue<DriverStationProto>;

using AccelerometerProto = muan::proto::StackProto<AccelerometerInput, 256>;
using AccelerometerQueue = muan::queues::MessageQueue<AccelerometerProto>;

using TiltProto = muan::proto::StackProto<TiltStatus, 256>;
using TiltQueue = muan::queues::MessageQueue<TiltProto>;

using PdpMessage = muan::proto::StackProto<PdpStatus, 512>;
using PdpQueue = muan::queues::MessageQueue<PdpMessage>;

using GameSpecificStringProto =
    muan::proto::StackProto<GameSpecificString, 512>;
using GameSpecificStringQueue =
    muan::queues::MessageQueue<GameSpecificStringProto>;

}  // namespace wpilib
}  // namespace muan

#endif  // MUAN_WPILIB_QUEUE_TYPES_H_
