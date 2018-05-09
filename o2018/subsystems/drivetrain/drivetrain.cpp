#include "o2018/subsystems/drivetrain/drivetrain.h"

namespace o2018 {

namespace drivetrain {

Drivetrain::Drivetrain() {
  /* right_master_ = new TalonSRX(kRightMaster); */
  /* left_master_ = new TalonSRX(kLeftMaster); */
  TalonSRX* _talon = new TalonSRX(0);
  _talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
                                       0, 10);
}

Drivetrain& Drivetrain::GetInstance() {
  static Drivetrain instance;  // Meyers singleton, same across every call
  return instance;
}

void Drivetrain::WriteActuators() {}

}  // namespace drivetrain

}  // namespace o2018
