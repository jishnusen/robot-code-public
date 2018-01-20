#include <cmath>
#include <limits>
#include "c2018/subsystems/score_subsystem/elevator/elevator_controller.h"

namespace c2018 {

namespace score_subsystem {

namespace elevator {

ElevatorController::ElevatorController() {
  auto elevator_plant = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::elevator_controller::controller::first_stage_integral::A(),
      frc1678::elevator_controller::controller::first_stage_integral::B(),
      frc1678::elevator_controller::controller::first_stage_integral::C());

  elevator_controller_ = muan::control::StateSpaceController<1, 3, 1>(
      frc1678::elevator_controller::controller::first_stage_integral::K(),
      frc1678::elevator_controller::controller::first_stage_integral::Kff(),
      frc1678::elevator_controller::controller::first_stage_integral::A(),
      Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
      Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

  elevator_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
      elevator_plant, frc1678::elevator_controller::controller::first_stage_integral::L());

  plant_ = muan::control::StateSpacePlant<1, 3, 1>(
      frc1678::elevator_controller::controller::first_stage_integral::A(),
      frc1678::elevator_controller::controller::first_stage_integral::B(),
      frc1678::elevator_controller::controller::first_stage_integral::C());

  trapezoid_profile_.set_maximum_acceleration(kElevatorAcceleration);
  trapezoid_profile_.set_maximum_velocity(kElevatorVelocity);
}

void ElevatorController::Update(const ScoreSubsystemInputProto& input, ScoreSubsystemOutputProto* output,
                                ScoreSubsystemStatusProto* status, bool outputs_enabled) {
  Eigen::Matrix<double, 3, 1> elevator_r_;

  SetWeights(elevator_observer_.x()(0, 0) >= 1.0, input->has_cube());
  auto elevator_y = (Eigen::Matrix<double, 1, 1>()
                     << hall_calib_.Update(input->elevator_encoder(), input->elevator_hall()));
  elevator_r_ =
      (Eigen::Matrix<double, 3, 1>() << UpdateProfiledGoal(unprofiled_goal_, outputs_enabled)(0, 0), 0.0, 0.0)
          .finished();

  elevator_controller_.r() = elevator_r_;

  auto elevator_u = elevator_controller_.Update(elevator_observer_.x())(0, 0);

  if (!outputs_enabled) {
    elevator_u = 0;
  } else {
    if (!encoder_fault_detected_) {
      (*status)->set_elevator_uncapped_voltage(elevator_u);
      elevator_u = CapU(elevator_u);
    } else {
      (*status)->set_elevator_uncapped_voltage(0);  // TODO (Jishnu) find an actual value for this
      elevator_u = 2;
    }
    if (old_pos_ == input->elevator_encoder() && std::abs(elevator_u) > 2) {
      num_encoder_fault_ticks_++;
      if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
        encoder_fault_detected_ = true;
      }
    } else {
      num_encoder_fault_ticks_ = 0;
    }
  }

  old_pos_ = input->elevator_encoder();

  elevator_observer_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished(), elevator_y.finished());
  plant_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished());

  bool at_top = false;

  if (elevator_observer_.x()(0, 0) == kElevatorMaxHeight) { at_top = true; }

  if (elevator_observer_.x()(0, 0) < 0) { elevator_observer_.x()(0,0) = 0; }

  (*output)->set_elevator_voltage(elevator_u);
  (*status)->set_elevator_actual_height(elevator_observer_.x()(0, 0));
  (*status)->set_elevator_voltage_error(elevator_observer_.x()(2, 0));
  (*status)->set_estimated_velocity(elevator_observer_.x()(0, 0));
  (*status)->set_elevator_calibrated(hall_calib_.is_calibrated());
  (*status)->set_elevator_profiled_goal(profiled_goal_(0, 0));
  (*status)->set_elevator_unprofiled_goal(unprofiled_goal_);
  (*status)->set_elevator_at_top(at_top);
  (*status)->set_elevator_encoder_fault_detected(encoder_fault_detected_);
}

void ElevatorController::SetGoal(c2018::score_subsystem::ScoreSubsystemGoalProto goal) {
  switch (goal->elevator_height()) {
    case HEIGHT_0:
      unprofiled_goal_ = kElevatorStartingHeight;
      break;
    case HEIGHT_1:
      unprofiled_goal_ = kElevatorFirstCubeHeight;
      break;
    case HEIGHT_2:
      unprofiled_goal_ = kElevatorSecondCubeHeight;
      break;
    case HEIGHT_SCORE:
      unprofiled_goal_ = kElevatorMaxHeight;
      break;
  }
}

Eigen::Matrix<double, 2, 1> ElevatorController::UpdateProfiledGoal(double unprofiled_goal_,
                                                                   bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoid_profile_.Update(unprofiled_goal_, 0);
  } else {
    profiled_goal_ = elevator_observer_.x().block<2, 1>(0, 0);
  }
  return profiled_goal_;
}

double ElevatorController::CapU(double elevator_u) {
  double u;
  if (elevator_u > 12) {
    u = 12;
  } else if (elevator_u < -12) {
    u = -12;
  } else {
    u = elevator_u;
  }
  return u;
}
void ElevatorController::SetWeights(bool second_stage, bool has_cube) {
  if (second_stage && has_cube) {
    elevator_controller_.A() = frc1678::elevator_controller::controller::second_stage_cube_integral::A();
    elevator_controller_.K() = frc1678::elevator_controller::controller::second_stage_cube_integral::K();
    elevator_controller_.Kff() = frc1678::elevator_controller::controller::second_stage_cube_integral::Kff();

    elevator_observer_.L() = frc1678::elevator_controller::controller::second_stage_cube_integral::L();

    plant_.A() = frc1678::elevator_controller::controller::second_stage_cube_integral::A();
    plant_.B() = frc1678::elevator_controller::controller::second_stage_cube_integral::B();
    plant_.C() = frc1678::elevator_controller::controller::second_stage_cube_integral::C();
  } else if (second_stage && !has_cube) {
    elevator_controller_.A() = frc1678::elevator_controller::controller::second_stage_integral::A();
    elevator_controller_.K() = frc1678::elevator_controller::controller::second_stage_integral::K();
    elevator_controller_.Kff() = frc1678::elevator_controller::controller::second_stage_integral::Kff();

    elevator_observer_.L() = frc1678::elevator_controller::controller::second_stage_integral::L();

    plant_.A() = frc1678::elevator_controller::controller::second_stage_integral::A();
    plant_.B() = frc1678::elevator_controller::controller::second_stage_integral::B();
    plant_.C() = frc1678::elevator_controller::controller::second_stage_integral::C();
  } else if (!second_stage && has_cube) {
    elevator_controller_.A() = frc1678::elevator_controller::controller::first_stage_cube_integral::A();
    elevator_controller_.K() = frc1678::elevator_controller::controller::first_stage_cube_integral::K();
    elevator_controller_.Kff() = frc1678::elevator_controller::controller::first_stage_cube_integral::Kff();

    elevator_observer_.L() = frc1678::elevator_controller::controller::first_stage_cube_integral::L();

    plant_.A() = frc1678::elevator_controller::controller::first_stage_cube_integral::A();
    plant_.B() = frc1678::elevator_controller::controller::first_stage_cube_integral::B();
    plant_.C() = frc1678::elevator_controller::controller::first_stage_cube_integral::C();
  } else if (!second_stage && !has_cube) {
    elevator_controller_.A() = frc1678::elevator_controller::controller::first_stage_integral::A();
    elevator_controller_.K() = frc1678::elevator_controller::controller::first_stage_integral::K();
    elevator_controller_.Kff() = frc1678::elevator_controller::controller::first_stage_integral::Kff();

    elevator_observer_.L() = frc1678::elevator_controller::controller::first_stage_integral::L();

    plant_.A() = frc1678::elevator_controller::controller::first_stage_integral::A();
    plant_.B() = frc1678::elevator_controller::controller::first_stage_integral::B();
    plant_.C() = frc1678::elevator_controller::controller::first_stage_integral::C();
  }
}

}  // namespace elevator

}  // namespace score_subsystem

}  // namespace c2018
