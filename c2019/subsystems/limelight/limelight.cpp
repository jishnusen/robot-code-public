#include "c2019/subsystems/limelight/limelight.h"
#include <memory>

using muan::queues::QueueManager;

namespace c2019 {
namespace limelight {

Limelight::Limelight(double limelight_height, double limelight_angle,
                     double object_height, double dist_factor,
                     double dist_offset)
    : status_queue_{QueueManager<LimelightStatusProto>::Fetch()},
      limelight_height_(limelight_height),
      limelight_angle_(limelight_angle),
      object_height_(object_height),
      dist_factor_(dist_factor),
      dist_offset_(dist_offset) {}

void Limelight::GetTable() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight-");
  target_horizontal_angle_ = table->GetEntry("tx").GetDouble(0);
  target_vertical_angle_ = table->GetEntry("ty").GetDouble(0);
  target_area_ = table->GetEntry("ta").GetDouble(0);
  target_skew_ = table->GetEntry("ts").GetDouble(0);
  target_present_ = table->GetEntry("tv").GetDouble(0);
}

double Limelight::ObjectDistance(double vertical_angle) {
  distance_ = (limelight_height_ - object_height_) *
              tan((M_PI / 180.) * (limelight_angle_ + vertical_angle));
  return distance_;
}

void Limelight::Update() {
  LimeLightStatusProto status;
  LimelightGoalProto goal;

  if (goal_reader_.ReadLastMessage(&goal)) {
    auto inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight");
    table->PutNumber("ledMode", static_cast<int>(goal->limelight_state()));
  }

  status->set_has_target(target_present_);
  if (target_present_) {
    status->set_dist((ObjectDistance(target_vertical_angle_) * dist_factor_) -
                     dist_offset_);
    status->set_theta(target_horizontal_angle_);
    status->set_relative_x(std::cos(status->theta() * (M_PI / 180.)) *
                           status->dist());
    status->set_relative_y(std::sin(status->theta() * (M_PI / 180.)) *
                           status->dist());
  }
  status_queue_->WriteMessage(status);
}

//////////////////////////////////////////////////////////////////// Front is Up
/// and Back is Down ////////////////////////

BackLimelight::BackLimelight(double back_limelight_height,
                             double back_limelight_angle,
                             double back_object_height, double back_dist_factor,
                             double back_dist_offset)
    : status_queue_{QueueManager<LimelightStatusProto>::Fetch()},
      back_limelight_height_(back_limelight_height),
      back_limelight_angle_(back_limelight_angle),
      back_object_height_(back_object_height),
      back_dist_factor_(back_dist_factor),
      back_dist_offset_(back_dist_offset) {}

void BackLimelight::GetBackTable() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight-back");
  back_target_horizontal_angle_ = table->GetEntry("tx").GetDouble(0);
  back_target_vertical_angle_ = table->GetEntry("ty").GetDouble(0);
  back_target_area_ = table->GetEntry("ta").GetDouble(0);
  back_target_skew_ = table->GetEntry("ts").GetDouble(0);
  back_target_present_ = table->GetEntry("tv").GetDouble(0);
}

double BackLimelight::BackObjectDistance(double back_vertical_angle) {
  back_distance_ =
      (back_lime_height_ - back_object_height_) *
      tan((M_PI / 180.) * (back_lime_angle_ + back_vertical_angle));
  return back_distance_;
}

void BackLimelight::UpdateBack() {
  LimelightStatusProto status;
  LimelightGoalProto goal;

  if (goal_reader_.ReadLastMessage(&goal)) {
    auto inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight-back");
    table->PutNumber("ledMode", static_cast<int>(goal->limelight_state()));
  }

  status->set_back_has_target(back_target_present_);
  if (back_target_present_) {
    status->set_back_dist(
        (BackObjectDistance(back_target_vertical_angle_) * back_dist_factor_) -
        back_dist_offset_);
    status->set_back_theta(back_target_horizontal_angle_);
    status->set_back_relative_x(std::cos(status->back_theta() * (M_PI / 180.)) *
                                status->back_dist());
    status->set_back_relative_y(std::sin(status->back_theta() * (M_PI / 180.)) *
                                status->back_dist());
  }
  status_queue_->WriteMessage(status);
}

}  // namespace limelight
}  // namespace c2019
