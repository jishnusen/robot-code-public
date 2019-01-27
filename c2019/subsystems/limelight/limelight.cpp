#include "c2019/subsystems/limelight/limelight.h"
#include <memory>

using muan::queues::QueueManager;

namespace c2019 {
namespace limelight {

Limelight::Limelight(const double limelight_height,const double limelight_angle,
                     const double object_height)
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

    std::vector<double> zero = {0, 0};
    std::vector<double> y_corner = table->GetEntry("tcorny").GetDoubleArray(zero);
    std::vector<double> x_corner = table->GetEntry("tcornx").GetDoubleArray(zero);

    slope_  = (x_corner[3] - x_corner[1]) / (y_corner[3] - y_corner[1]);

    table->PutNumber("pipeline", 0);
    target_dist_ =
        std::tan((target_vertical_angle + limelight_angle_) * (M_PI / 180.)) * (limelight_height_ - object_height_  * 0.0254);
    double distance = target_dist_ * 2.70247 - 1.0116;

    horiz_angle_ = (target_horizontal_angle * (M_PI / 180.));

    target_1_horizontal_angle_ =
        std::min(target_1_horizontal_angle, target_2_horizontal_angle);
    target_2_horizontal_angle_ =
        std::max(target_1_horizontal_angle, target_2_horizontal_angle);
    double model_width = -.0305387 * pow(target_dist_, 4) +
                         0.21889895 * pow(target_dist_, 3) -
                         .6260213 * pow(target_dist_, 2) +
                         0.913154444 * target_dist_ - 0.714746 + 0.08;
    double heading = target_1_horizontal_angle_ - target_2_horizontal_angle_;
    double skim_error = model_width - heading;
    double heading_model = 544.0577 * pow(skim_error, 3) -
                           88.244 * pow(skim_error, 2) + 5.7379 * skim_error -
                           0.02217;
    heading_model_ = heading_model * (-0.002 * pow(horiz_angle_, 2) -
                                     0.058 * horiz_angle_ - 0.155);

    target_y_ =
        distance * std::sin(horiz_angle_ + 2 * M_PI * std::abs(heading_model));
    target_x_ =
        distance * std::cos(horiz_angle_ + 2 * M_PI * std::abs(heading_model));
    distance_ = distance;
 
  
  LimelightStatusProto status;
  status->set_target_dist(distance_ * 1.1 * 1.);
  status->set_skew(target_skew_);
  status->set_target_x(target_x_);
  status->set_target_y(target_y_);
  status->set_target_1_horizontal_angle(target_1_horizontal_angle_);
  status->set_target_2_horizontal_angle(target_2_horizontal_angle_);
  status->set_heading(std::abs(2 * M_PI * heading_model_) * 2);
  status->set_to_the_left(slope_ > 0);
  status->set_horiz_angle(std::copysign(std::abs(horiz_angle_) + 0.1, horiz_angle_));
  status_queue_->WriteMessage(status);
}

}  // namespace limelight
}  // namespace c2019
