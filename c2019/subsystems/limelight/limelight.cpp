#include "c2019/subsystems/limelight/limelight.h"
#include "c2019/subsystems/superstructure/queue_types.h"
#include <algorithm>
#include <memory>
#include <vector>

using muan::queues::QueueManager;
using c2019::superstructure::SuperstructureStatusProto;

namespace c2019 {
namespace limelight {

Limelight::Limelight(const double limelight_height,
                     const double limelight_angle, const double object_height)
    : status_queue_{QueueManager<LimelightStatusProto>::Fetch()},
      limelight_height_(limelight_height),
      limelight_angle_(limelight_angle),
      object_height_(object_height) {}

void Limelight::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(10));
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("Limelight");

  running_ = true;

  while (running_) {
    Update();
    BackUpdate();
    phased_loop.SleepUntilNext();
  }
}

void Limelight::Update() {
  SuperstructureStatusProto super_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &super_status);
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight-front");
  std::shared_ptr<nt::NetworkTable> expensive_table = inst.GetTable("limelight-pricey");
  double target_vertical_angle = table->GetEntry("ty").GetDouble(0);
  double skew = table->GetEntry("ts").GetDouble(0);
  double target_horizontal_angle = table->GetEntry("tx").GetDouble(0);
  double target_1_horizontal_angle =
      table->GetEntry("tx0").GetDouble(-1000) * (59.6 / 2.0) * (M_PI / 180.);
  double target_2_horizontal_angle =
      table->GetEntry("tx1").GetDouble(-1000) * (59.6 / 2.0) * (M_PI / 180.);
  double target_3_horizontal_angle =
      table->GetEntry("tx2").GetDouble(-1000) * (59.6 / 2.0) * (M_PI / 180.);

  std::vector<double> zero = {0, 0};
  std::vector<double> y_corner = table->GetEntry("tcorny").GetDoubleArray(zero);
  std::vector<double> x_corner = table->GetEntry("tcornx").GetDoubleArray(zero);

//  slope_ = (x_corner[3] - x_corner[1]) / (y_corner[3] - y_corner[1]);

  table->PutNumber("pipeline", 0);
  target_dist_ =
      std::tan((target_vertical_angle + limelight_angle_) * (M_PI / 180.)) *
      (limelight_height_ - object_height_ * 0.0254);

  target_dist_ = std::tan((target_vertical_angle + 60.0) * (M_PI / 180.)) *
                 ((limelight_height_ - object_height_) * 0.0254);
  double distance =
      2.497 * pow(target_dist_, 2) - 0.0397 * target_dist_ + 0.2124;

  if (super_status->elevator_height() > 1.0) {
    target_horizontal_angle = expensive_table->GetEntry("tx").GetDouble(-1000);
  }
  horiz_angle_ = (target_horizontal_angle * (M_PI / 180.));

  double overall_tx = target_horizontal_angle / (59.6 * 0.5); // normalized tx
  std::array<double, 3> horiz_angles = {table->GetEntry("tx0").GetDouble(-1000), table->GetEntry("tx1").GetDouble(-1000), table->GetEntry("tx2").GetDouble(-1000)};
  std::sort(horiz_angles.begin(), horiz_angles.end());
  for (double angle : horiz_angles) {
    if (angle < overall_tx) {
      target_1_horizontal_angle_ = angle;
    }
  }
  std::sort(horiz_angles.begin(), horiz_angles.end());
  std::reverse(horiz_angles.begin(), horiz_angles.end());

  for (double angle : horiz_angles) {
    if (angle > overall_tx) {
      target_2_horizontal_angle_ = angle;
    }
  }

  target_1_horizontal_angle_ = target_1_horizontal_angle_ * (59.6 * 0.5) * (M_PI / 180.);
  target_2_horizontal_angle_ = target_2_horizontal_angle_ * (59.6 * 0.5) * (M_PI / 180.);
  double difference = target_1_horizontal_angle - target_2_horizontal_angle;
  if(skew > -45){
	heading_ = std::abs(skew/8.);
	to_the_left_ = true;
}
  else{
	heading_ =((skew+90)/8.);
	to_the_left_ = false;
}

  double has_target = table->GetEntry("tv").GetDouble(0);
  LimelightStatusProto status;
  status->set_target_dist(distance / 2.2);
  status->set_skew(skew);
  status->set_target_1_horizontal_angle(target_1_horizontal_angle_);
  status->set_target_2_horizontal_angle(target_2_horizontal_angle_);
  status->set_to_the_left(to_the_left_);
  status->set_heading(heading_);
  status->set_has_target(has_target == 1);
  status->set_difference(difference);
  status->set_horiz_angle(horiz_angle_ * 1.667 * (0.42 / 0.58));
  status->set_overall_tx(target_horizontal_angle / (59.6 * 0.5));
  status->set_unfiltered_horiz_angle_1(target_1_horizontal_angle);
  status->set_unfiltered_horiz_angle_2(target_2_horizontal_angle);
  status->set_unfiltered_horiz_angle_3(target_3_horizontal_angle);

  std::shared_ptr<nt::NetworkTable> back_table = inst.GetTable("limelight-back");
  double back_target_vertical_angle = back_table->GetEntry("ty").GetDouble(0);
  double back_target_horizontal_angle = back_table->GetEntry("tx").GetDouble(0);
 back_target_dist_ = std::tan((back_target_vertical_angle + 60.0) * (M_PI / 180.)) *
                 ((limelight_height_ - object_height_) * 0.0254);
  double back_distance =
      2.497 * pow(back_target_dist_, 2) - 0.0397 * back_target_dist_ + 0.2124;
  back_horiz_angle_ = (back_target_horizontal_angle * (M_PI / 180.));
  status->set_back_horiz_angle(back_horiz_angle_);
  status->set_back_target_dist(back_distance / 2.2);
  status->set_back_has_target(static_cast<bool>(back_table->GetEntry("tv").GetDouble(0)));
  status_queue_->WriteMessage(status);
}

void Limelight::BackUpdate(){
  LimelightStatusProto status; 
}

}  // namespace limelight
}  // namespace c2019
