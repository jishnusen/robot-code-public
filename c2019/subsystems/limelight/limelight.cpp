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
      table->GetEntry("tx0").GetDouble(-1000) * (59.6 / 2.0) * (M_PI / 180.);
  double target_2_horizontal_angle =
      table->GetEntry("tx1").GetDouble(-1000) * (59.6 / 2.0) * (M_PI / 180.);
  double target_3_horizontal_angle =
      table->GetEntry("tx2").GetDouble(-1000) * (59.6 / 2.0) * (M_PI / 180.);

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
  /* double heading_model = */
  /*     7.49562907 * pow(target_dist_, 4) - 20.2223 * pow(target_dist_, 3) + */
  /*     20.6362229 * pow(target_dist_, 2) - 9.9716668 * target_dist_ + 2.19656;
   */
  /*double heading_model =
      0.3884744 * pow(target_dist_, 4) - 1.60138354 * pow(target_dist_, 3) +
      2.8595594 * pow(target_dist_, 2) - 2.5235603 * target_dist_ + 1.09079;*/
  double heading_model = 0.3616 * target_dist_ * target_dist_ - 1.04477 * target_dist_ + 0.8617;
  double skim_error = heading_model - std::abs(difference);
  double has_target = table->GetEntry("tv").GetDouble(0);
  //double error_sub = 0.0273021*pow((distance/2.0), 2) -0.0130306*(distance/2.0) + 0.00812367;
  //  double tx_factor = 1 + 0.4*std::abs(target_horizontal_angle);
  //double final_heading = 16* (skim_error - 0.02);
 // if(distance>0.88){
   
 // final_heading = 28* (skim_error - error_sub);
 
 // }
  LimelightStatusProto status;
  status->set_target_dist(distance / 2.2);
  status->set_skew(target_skew_);
  status->set_target_1_horizontal_angle(target_1_horizontal_angle_);
  status->set_target_2_horizontal_angle(target_2_horizontal_angle_);
  status->set_to_the_left(slope_ > 0);
  status->set_skim_error(skim_error);
  /* status->set_heading(std::abs(1.4 * final_heading - .17)); */
  status->set_heading(std::abs(skim_error * 100));
  status->set_has_target(has_target == 1);
  status->set_heading_model(heading_model);
  status->set_difference(difference);
  status->set_horiz_angle(std::copysign(std::abs(horiz_angle_), horiz_angle_));
  status->set_overall_tx(target_horizontal_angle / (59.6 * 0.5));
  status->set_unfiltered_horiz_angle_1(target_1_horizontal_angle);
  status->set_unfiltered_horiz_angle_2(target_2_horizontal_angle);
  status->set_unfiltered_horiz_angle_3(target_3_horizontal_angle);

  status_queue_->WriteMessage(status);
}

}  // namespace limelight
}  // namespace c2019
