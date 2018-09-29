#ifndef O2016_SUBSYSTEMS_DRIVETRAIN_QUEUE_TYPES_H_
#define O2016_SUBSYSTEMS_DRIVETRAIN_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/drivetrain/drivetrain.pb.h"

namespace o2016 {
namespace drivetrain {

using StackDrivetrainInput = muan::proto::StackProto<DrivetrainInput, 80>;
using StackDrivetrainGoal = muan::proto::StackProto<DrivetrainGoal, 210>;
using StackDrivetrainStatus = muan::proto::StackProto<DrivetrainStatus, 130>;
using StackDrivetrainOutput = muan::proto::StackProto<DrivetrainOutput, 80>;

using DrivetrainInputQueue = muan::queues::MessageQueue<StackDrivetrainInput>;
using DrivetrainGoalQueue = muan::queues::MessageQueue<StackDrivetrainGoal>;
using DrivetrainStatusQueue = muan::queues::MessageQueue<StackDrivetrainStatus>;
using DrivetrainOutputQueue = muan::queues::MessageQueue<StackDrivetrainOutput>;

}  // namespace drivetrain
}  // namespace o2016

#endif  // O2016_SUBSYSTEMS_DRIVETRAIN_QUEUE_TYPES_H_
