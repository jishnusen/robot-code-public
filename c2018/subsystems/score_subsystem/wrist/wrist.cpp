#include "c2018/subsystems/score_subsystem/wrist/wrist.h"
#include <cmath>

namespace c2018 {
namespace score_subsystem {
namespace wrist {

using muan::queues::QueueManager;

WristController::WristController()
    : trapezoidal_motion_profile_{::std::chrono::milliseconds(5)},
      status_queue_{QueueManager<ScoreSubsystemStatusProto>::Fetch()},
      output_queue_{QueueManager<ScoreSubsystemOutputProto>::Fetch()},
      hall_calibration_{kHallMagnetPosition} {
  auto wrist_plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::wrist_controller::controller::A(),
      frc1678::wrist_controller::controller::B(),
      frc1678::wrist_controller::controller::C());

  wrist_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::wrist_controller::controller::K(),
      frc1678::wrist_controller::controller::Kff(),
      frc1678::wrist_controller::controller::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -12,
      Eigen::Matrix<double, 1, 1>::Ones() * 12);

  wrist_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      wrist_plant, frc1678::wrist_controller::controller::L());

  trapezoidal_motion_profile_.set_maximum_acceleration(kMaxWristAcceleration);
  trapezoidal_motion_profile_.set_maximum_velocity(kMaxWristVelocity);
}

void WristController::SetGoal(double angle, IntakeMode mode) {
  if (wrist_state_ == SYSTEM_IDLE || wrist_state_ == MOVING) {
    unprofiled_goal_position_ = muan::utils::Cap(angle, 0.0, M_PI);
    wrist_state_ = MOVING;
  }

  intake_mode_ = mode;
}

void WristController::Update(ScoreSubsystemInputProto input,
                             ScoreSubsystemOutputProto* output,
                             ScoreSubsystemStatusProto* status,
                             bool outputs_enabled = true) {
  double calibrated_encoder =
      hall_calibration_.Update(input->wrist_encoder(), input->wrist_hall());
  auto wrist_y =
      (Eigen::Matrix<double, 1, 1>() << calibrated_encoder).finished();

  double wrist_voltage = 0.0;

  if (!outputs_enabled) {
    wrist_voltage = 0.0;
    wrist_state_ = DISABLED;
  } else if (!hall_calibration_.is_calibrated() && !encoder_fault_detected_) {
    wrist_state_ = CALIBRATING;
  } else if (wrist_state_ == DISABLED) {
    wrist_state_ = SYSTEM_IDLE;
  }

  double intake_voltage = 0;

  // Start of intake
  bool wrist_solenoid_1 = false;
  bool wrist_solenoid_2 = false;
  wrist_pinch_ = WRIST_IN;


  if (outputs_enabled) {
    switch (intake_mode_) {
      case INTAKE:
        intake_voltage = 12;
        wrist_pinch_ = WRIST_OUT;
        break;
      case OUTTAKE:
        intake_voltage = -12;
        wrist_pinch_ = WRIST_IN;
        break;
      case IDLE:
        intake_voltage = 0;
        wrist_pinch_ = WRIST_IN;
        break;
      case HOLD:
        intake_voltage = kHoldingVoltage;
        wrist_pinch_ = WRIST_OUT;
        break;
    }
  } else {
    intake_voltage = 0;
    wrist_pinch_ = WRIST_IN;
  }

  switch (wrist_pinch_) {
    case WRIST_IN:
    wrist_solenoid_1 = false;
    wrist_solenoid_2 = false;
    break;
    case WRIST_OUT:
    wrist_solenoid_1 = true;
    wrist_solenoid_2 = true;
    break;
    case WRIST_IDLE:
    // Idle or "neutral" is the starting position
    wrist_solenoid_1 = false;
    wrist_solenoid_2 = false;
    break;
  }
  switch (wrist_state_) {
    case SYSTEM_IDLE:
      wrist_voltage = 0;
      intake_voltage = 0;
      break;
    case ENCODER_FAULT:
      wrist_voltage = 0;
      intake_voltage = 0;
      break;
    case DISABLED:
      wrist_voltage = 0;
      intake_voltage = 0;
      break;
    case INITIALIZING:
      wrist_state_ = CALIBRATING;
      break;
    case CALIBRATING:
      wrist_voltage = kCalibVoltage;
      intake_voltage = 0;
      if (hall_calibration_.is_calibrated()) {
        wrist_state_ = SYSTEM_IDLE;
      }
      break;
    case MOVING:
      // Run the controller
      Eigen::Matrix<double, 3, 1> wrist_r =
          (Eigen::Matrix<double, 3, 1>() << UpdateProfiledGoal(
               unprofiled_goal_position_, outputs_enabled)(0, 0),
           0.0, 0.0)
              .finished();

      wrist_controller_.r() = wrist_r;

      wrist_voltage = wrist_controller_.Update(wrist_observer_.x())(0, 0);

      break;
  }

  // Check for encoder faults
  if (old_pos_ == input->wrist_encoder() && wrist_voltage > kHoldingVoltage) {
    num_encoder_fault_ticks_++;
    if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
      encoder_fault_detected_ = true;
      wrist_state_ = ENCODER_FAULT;
    }
  } else {
    num_encoder_fault_ticks_ = 0;
  }
  old_pos_ = input->wrist_encoder();

  if (!encoder_fault_detected_) {
    wrist_voltage = CapU(wrist_voltage);
  } else {
    wrist_voltage = 0;
  }

  wrist_observer_.Update(
      (Eigen::Matrix<double, 1, 1>() << wrist_voltage).finished(), wrist_y);

  (*output)->set_intake_voltage(intake_voltage);
  (*output)->set_wrist_voltage(wrist_voltage);
  (*output)->set_wrist_solenoid_1(wrist_solenoid_1);
  (*output)->set_wrist_solenoid_2(wrist_solenoid_2);
  (*status)->set_wrist_calibrated(hall_calibration_.is_calibrated());
  (*status)->set_wrist_position(wrist_observer_.x()(0, 0));
  (*status)->set_wrist_pinch(wrist_pinch_);
  (*status)->set_wrist_state(wrist_state_);
  status_queue_->WriteMessage(*status);
  output_queue_->WriteMessage(*output);
}

double WristController::CapU(double wrist_voltage) {
  return muan::utils::Cap(wrist_voltage, -12, 12);
}

Eigen::Matrix<double, 2, 1> WristController::UpdateProfiledGoal(
    double unprofiled_goal_, bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoidal_motion_profile_.Update(unprofiled_goal_, 0.0);
  }

  return profiled_goal_;
}

}  // namespace wrist
}  // namespace score_subsystem
}  // namespace c2018