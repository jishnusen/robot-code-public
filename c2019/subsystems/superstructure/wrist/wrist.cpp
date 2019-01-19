#include "c2019/subsystems/superstructure/wrist/wrist.h"

namespace c2019 {
namespace wrist {

Wrist::Wrist() {}

void Wrist::SetGoal(const WristGoalProto& goal) {
  goal_ = muan::utils::Cap(goal->angle(), kMinAngle, kMaxAngle);
}

double Wrist::CalculateFeedForwards(bool has_cargo, bool has_panel, double angle) {

  double ff = kFF;
  if (has_cargo) { // TODO do this correctly
    ff += kFFCargo;
  }
  if (has_panel) { // TODO do this correctly
    ff += kFFHatch;
  }

  return ff * .5 * cos(angle);
}

void Wrist::Update(const WristInputProto& input,
                   WristOutputProto* output,
                   WristStatusProto* status,
                   bool outputs_enabled) {

  const double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());
      (*status)->set_wrist_angle(calibrated_encoder);

  (*status)->set_is_calibrated(is_calibrated());

  (*output)->set_wrist_setpoint_ff(
      CalculateFeedForwards(input->has_panel(), input->has_cargo(), calibrated_encoder));
  if (outputs_enabled) {
    if (is_calibrated()) {
      (*output)->set_output_type(POSITION);
      (*output)->set_wrist_setpoint(goal_);
      if (calibrated_encoder <= goal_ + 1e-2 &&
          goal_ < 1e-2) {
        (*output)->set_output_type(OPEN_LOOP);
        (*output)->set_wrist_setpoint(0);
      }
    } else {
      (*output)->set_output_type(OPEN_LOOP);
      (*output)->set_wrist_setpoint(0);
    }
  } else {
    (*output)->set_output_type(OPEN_LOOP);
    (*output)->set_wrist_setpoint(0);
  }
}

}  // wrist
}  // c2019
