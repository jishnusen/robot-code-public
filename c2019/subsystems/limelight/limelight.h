#ifndef C2019_SUBSYSTEMS_LIMELIGHT_LIMELIGHT_H_
#define C2019_SUBSYSTEMS_LIMELIGHT_LIMELIGHT_H_

#include <cmath>
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

namespace c2019 {
namespace limelight {

class Limelight {
 public:
  Limelight(double limelight_height, double limelight_angle,
            double object_height);
  void GetTable();
  void Update();
  double ObjectDistance(double vertical_angle);

 private:
  LimelightStatusQueue* status_queue_;
  LimelightGoalQueue::QueueReader goal_reader_{
      muan::queues::QueueManager<LimelightGoalProto>::Fetch()->MakeReader()};
  // FRIEND_TEST(LimelightTest, HasNoTarget);
  double limelight_height_;
  double limelight_angle_;
  double object_height_;
  double dist_factor_ = 1;
  double dist_offset_ = 0;
  double distance_;
  double target_vertical_angle_;
  double target_horizontal_angle_;
  double target_area_;
  double target_skew_;
  bool target_present_;
};

class BackLimelight {
 public:
  BackLimelight(double back_limelight_height, double back_limelight_angle,
                double back_object_height, double back_dist_factor,
                double back_dist_offset);
  void GetBackTable();
  void UpdateBack();
  double BackObjectDistance(double back_vertical_angle);

 private:
  LimelightStatusQueue* status_queue_;
  LimelightGoalQueue::QueueReader goal_reader_{
      muan::queues::QueueManager<LimelightGoalProto>::Fetch()->MakeReader()};
  // FRIEND_TEST(LimelightTest, HasNoTarget);
  double back_limelight_height_;
  double back_limelight_angle_;
  double back_object_height_;
  double back_dist_factor_ = 1;
  double back_dist_offset_ = 0;
  double back_distance_;
  double back_target_vertical_angle_;
  double back_target_horizontal_angle_;
  double back_target_area_;
  double back_target_skew_;
  bool back_target_present_;
};

}  // namespace limelight
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_LIMELIGHT_LIMELIGHT_H_
