#include "o2018/subsystems/arm/arm.h"

namespace o2018 {
namespace subsystems {
namespace arm {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Arm::Arm()
    : goal_reader_{QueueManager<ArmGoalProto>::Fetch()->MakeReader()},
      input_reader_{QueueManager<ArmInputProto>::Fetch()->MakeReader()},
      status_queue_{QueueManager<ArmStatusProto>::Fetch()},
      output_queue_{QueueManager<ArmOutputProto>::Fetch()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()} {}

void Arm::SetGoal(double angle, IntakeMode intake_goal) {
  unprofiled_goal_ = muan::utils::Cap(angle, kMinAngle, kMaxAngle);
  intake_goal_ = intake_goal;
}

void Arm::Update() {
  ArmInputProto input;
  ArmOutputProto output;
  ArmStatusProto status;
  ArmGoalProto goal;
  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    driver_station->set_battery_voltage(12.0);
  }

  bool outputs_enabled = driver_station->is_sys_active();

  if (goal_reader_.ReadLastMessage(&goal)) {
    SetGoal(goal->arm_angle(), goal->intake_mode());
  }

  const bool was_calibrated = is_calibrated();
  const double calibrated_encoder =
      hall_calibration_.Update(input->arm_encoder(), input->arm_hall());

  if (!outputs_enabled) {
    profiled_goal_ = calibrated_encoder;
  }

  if (!was_calibrated && is_calibrated()) {
    profiled_goal_ = calibrated_encoder;
  }

  // Encoder falt checking
  if (prev_position_ == calibrated_encoder &&
      std::abs(input->arm_voltage()) >= kEncoderFaultMinVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
      LOG(WARNING, "Encoder fault detected due to offset velocity");
    }
  } else if (prev_position_ != input->arm_encoder()) {
    // Reset the encoder fault checking so it doesn't build up
    num_encoder_fault_ticks_ = 0;
    encoder_fault_detected_ = false;
  }

  prev_position_ = calibrated_encoder;

  bool has_cube = /* pinch_state_ == IDLE_WITH_CUBE && */ input->intake_proxy();

  if (outputs_enabled && !encoder_fault_detected_) {
    if (is_calibrated()) {
      UpdateProfiledGoal(outputs_enabled);

      output->set_arm_output_type(POSITION);
      output->set_arm_setpoint(profiled_goal_ - hall_calibration_.offset());
      output->set_arm_setpoint_ff(CalculateFeedForwards(has_cube, calibrated_encoder));
    } else {
      output->set_arm_output_type(OPEN_LOOP);
      output->set_arm_setpoint(kCalibVoltage);
    }
  } else {
    output->set_arm_output_type(OPEN_LOOP);
    output->set_arm_setpoint(0);
  }

  // Start of intake
  bool intake_solenoid_close = false;
  bool intake_solenoid_open = false;
  double intake_voltage = 0.0;

  if (outputs_enabled) {
    switch (intake_goal_) {
      case IntakeMode::INTAKE:
        intake_voltage = kIntakeVoltage;
        intake_solenoid_close = false;
        intake_solenoid_open = false;
        break;
      case IntakeMode::INTAKE_OPEN:
        intake_voltage = kIntakeVoltage;
        intake_solenoid_close = false;
        intake_solenoid_open = true;
        break;
      case IntakeMode::INTAKE_CLOSE:
      case IntakeMode::SETTLE:
        intake_voltage = kIntakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeMode::OUTTAKE_SLOW:
        intake_voltage = kSlowOuttakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeMode::OUTTAKE_FAST:
        intake_voltage = kFastOuttakeVoltage;
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
      case IntakeMode::DROP:
        intake_voltage = 0;
        intake_solenoid_close = false;
        intake_solenoid_open = true;
        break;
      case IntakeMode::INTAKE_NONE:
        if (has_cube) {
          intake_voltage = kHoldingVoltage;
        } else {
          intake_voltage = 0;
        }
        intake_solenoid_close = true;
        intake_solenoid_open = false;
        break;
    }
  } else {
    intake_voltage = 0;
  }

  output->set_intake_voltage(intake_voltage);
  output->set_intake_open(intake_solenoid_open);
  output->set_intake_close(intake_solenoid_close);

  // Logic to make sure it actually has a cube
  /* switch (pinch_state_) { */
  /*   case MOVING: */
  /*     if (intake_solenoid_close) { */
  /*       has_cube_for_ticks_--; */
  /*     } else { */
  /*       has_cube_for_ticks_ = kNumHasCubeTicks; */
  /*     } */
  /*     if (has_cube_for_ticks_ < 0) { */
  /*       pinch_state_ = IDLE_WITH_CUBE; */
  /*     } */
  /*     break; */
  /*   case IDLE_WITH_CUBE: */
  /*     if (intake_solenoid_open) { */
  /*       pinch_state_ = IDLE_NO_CUBE; */
  /*     } */
  /*     break; */
  /*   case IDLE_NO_CUBE: */
  /*     if (intake_solenoid_close) { */
  /*       pinch_state_ = MOVING; */
  /*     } */
  /*     break; */
  /* } */

  status->set_arm_unprofiled_goal(unprofiled_goal_);
  status->set_arm_profiled_goal(profiled_goal_);

  status->set_arm_angle(calibrated_encoder);
  status->set_arm_velocity(input->arm_velocity());

  status->set_arm_calibrated(is_calibrated());
  status->set_arm_encoder_fault(encoder_fault_detected_);
  status->set_has_cube(has_cube);

  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status);
}

void Arm::UpdateProfiledGoal(bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = unprofiled_goal_;
  }
}

double Arm::CalculateFeedForwards(bool has_cube, double theta) {
  // Magic function goes here
  double FF = ((kArmMass + (has_cube ? kCubeMass : 0)) * 9.80 * kArmRadius) /
              (kArmStallTorque * kArmGearRatio);
  return FF * std::cos(theta);
}

}  // namespace arm
}  // namespace subsystems
}  // namespace o2018
