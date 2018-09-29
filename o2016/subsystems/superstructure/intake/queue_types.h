#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_QUEUE_TYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/intake/intake.pb.h"

namespace o2016 {
namespace intake {

using IntakeOutputProto = muan::proto::StackProto<IntakeOutput, 1024>;
using IntakeInputProto = muan::proto::StackProto<IntakeInput, 1024>;
using IntakeStatusProto = muan::proto::StackProto<IntakeStatus, 1024>;
using IntakeGoalProto = muan::proto::StackProto<IntakeGoal, 1024>;

using IntakeOutputQueue = muan::queues::MessageQueue<IntakeOutputProto>;
using IntakeInputQueue = muan::queues::MessageQueue<IntakeInputProto>;
using IntakeStatusQueue = muan::queues::MessageQueue<IntakeStatusProto>;
using IntakeGoalQueue = muan::queues::MessageQueue<IntakeGoalProto>;

}  // namespace intake
}  // namespace o2016

#endif  // O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_QUEUE_TYPES_H_
