#include "c2019/subsystems/limelight/limelight.h"
#include <algorithm>
#include <memory>
#include <vector>

using muan::queues::QueueManager;

namespace c2019 {
namespace limelight {

Limelight::Limelight(const double limelight_height,
                     const double limelight_angle, const double object_height)
    : status_queue_{QueueManager<LimelightStatusProto>::Fetch()},
      limelight_height_(limelight_height),
      limelight_angle_(limelight_angle),
      object_height_(object_height) {}

void Limelight::Update() {
  auto inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight");
  double target_vertical_angle = table->GetEntry("ty").GetDouble(0);
  double target_horizontal_angle = table->GetEntry("tx").GetDouble(0);
  double target_1_horizontal_angle =
      table->GetEntry("tx0").GetDouble(0) * (59.6 / 2.0) * (M_PI / 180.);
  double target_2_horizontal_angle =
      table->GetEntry("tx1").GetDouble(0) * (59.6 / 2.0) * (M_PI / 180.);
  double target_3_horizontal_angle =
      table->GetEntry("tx2").GetDouble(0) * (59.6 / 2.0) * (M_PI / 180.);

  std::vector<double> zero = {0, 0};
  std::vector<double> y_corner = table->GetEntry("tcorny").GetDoubleArray(zero);
  std::vector<double> x_corner = table->GetEntry("tcornx").GetDoubleArray(zero);

  slope_ = (x_corner[3] - x_corner[1]) / (y_corner[3] - y_corner[1]);

  table->PutNumber("pipeline", 0);
  target_dist_ =
      std::tan((target_vertical_angle + limelight_angle_) * (M_PI / 180.)) *
      (limelight_height_ - object_height_ * 0.0254);

  target_dist_ = std::tan((target_vertical_angle + 60.0) * (M_PI / 180.)) *
                 ((limelight_height_ - object_height_) * 0.0254);
  double distance =
      2.497 * pow(target_dist_, 2) - 0.0397 * target_dist_ + 0.2124;

  horiz_angle_ = (target_horizontal_angle * (M_PI / 180.));

  target_1_horizontal_angle_ =
      std::min(target_1_horizontal_angle, target_2_horizontal_angle);
  target_2_horizontal_angle_ =
      std::max(target_1_horizontal_angle, target_2_horizontal_angle);
  double difference = target_1_horizontal_angle - target_2_horizontal_angle;
  double heading_model =
      7.49562907 * pow(target_dist_, 4) - 20.2223 * pow(target_dist_, 3) +
      20.6362229 * pow(target_dist_, 2) - 9.9716668 * target_dist_ + 2.19656;
  double skim_error = heading_model - std::abs(difference);
  double final_heading =
      -445.775 * pow(skim_error, 2) + 32.7477 * skim_error - .0041;
  //  double tx_factor = 1 + 0.4*std::abs(target_horizontal_angle);
  LimelightStatusProto status;
  status->set_target_dist(distance);
  status->set_skew(target_skew_);
  status->set_target_1_horizontal_angle(target_1_horizontal_angle_);
  status->set_target_2_horizontal_angle(target_2_horizontal_angle_);
  status->set_target_3_horizontal_angle(target_3_horizontal_angle);
  status->set_to_the_left(slope_ > 0);
  status->set_heading(std::abs(1.4 * final_heading - .17));
  // status->set_heading_model(heading_model);
  // status->set_difference(difference);
  status->set_horiz_angle(
      std::copysign(std::abs(horiz_angle_) + 0.1, horiz_angle_));
  status_queue_->WriteMessage(status);
}

}  // namespace limelight
}  // namespace c2019
