#include "c2018/subsystems/score_subsystem/elevator/elevator_controller.h"
#include <cmath>
#include <limits>

namespace c2018 {

namespace score_subsystem {

namespace elevator {

ElevatorController::ElevatorController() {
  SetWeights(false, false);
  trapezoid_profile_.set_maximum_acceleration(kElevatorAcceleration);
  trapezoid_profile_.set_maximum_velocity(kElevatorVelocity);
}

void ElevatorController::Update(const ScoreSubsystemInputProto& input, ScoreSubsystemOutputProto* output, ScoreSubsystemStatusProto* status, bool outputs_enabled) {
  Eigen::Matrix<double, 3, 1> elevator_r_;

  SetWeights((*status)->elevator_actual_height() >= 1.0, input->has_cube()); // Dynamic Weighting based on if in second stage, and if it has a cube
  auto elevator_y = (Eigen::Matrix<double, 1, 1>() << hall_calib_.Update(input->elevator_encoder(), input->elevator_hall()));
  elevator_r_ = (Eigen::Matrix<double, 3, 1>() << 0.0,
                UpdateProfiledGoal(unprofiled_goal_, outputs_enabled),
                0.0).finished();

  auto elevator_u = elevator_controller_.Update(elevator_observer_.x())(0,0);

  if (!outputs_enabled) {
    elevator_u = 0;
  } else {
    if (!encoder_fault_detected_) {
      (*status)->set_elevator_uncapped_voltage(elevator_u);
      elevator_u = CapU(elevator_u);
    } else {
      (*status)->set_elevator_uncapped_voltage(3); // TODO (Jishnu) find an actual value for this
      elevator_u = CapU(3);
    }
    if (old_pos_ == input->elevator_encoder()) {
      num_encoder_fault_ticks_++;
      if (num_encoder_fault_ticks_ > kEncoderFaultTicksAllowed) {
        encoder_fault_detected_ = true;
      }
    }
  }

  old_pos_ = input->elevator_encoder();

  elevator_observer_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished(), elevator_y.finished());
  plant_.Update((Eigen::Matrix<double, 1, 1>() << elevator_u).finished());

  (*output)->set_elevator_voltage(elevator_u);
  (*status)->set_elevator_actual_height(elevator_observer_.x()(0,0));
  (*status)->set_estimated_velocity(elevator_observer_.x()(0,0));
}

void ElevatorController::SetGoal(c2018::score_subsystem::ScoreSubsystemGoalProto goal) {
  switch (goal->elevator_height()) {
    case HEIGHT_0:
      unprofiled_goal_ = 0;
    case HEIGHT_1:
      unprofiled_goal_ = 0.3;
    case HEIGHT_2:
      unprofiled_goal_ = 0.6;
    case HEIGHT_SCORE:
      unprofiled_goal_ = 2.07;
  }

  trapezoid_profile_.SetGoal(unprofiled_goal_);
}

Eigen::Matrix<double, 2, 1> ElevatorController::UpdateProfiledGoal(double unprofiled_goal_, bool outputs_enabled) {
  if (outputs_enabled) {
    profiled_goal_ = trapezoid_profile_.Update(unprofiled_goal_,0);
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
  }

  return elevator_u;
}
void ElevatorController::SetWeights(bool second_stage, bool has_cube) {
  if (second_stage && has_cube) {
    auto elevator_plant = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::second_stage_cube_integral::A(),
        frc1678::elevator_controller::controller::second_stage_cube_integral::B(),
        frc1678::elevator_controller::controller::second_stage_cube_integral::C());

    elevator_controller_ = muan::control::StateSpaceController<1, 3, 1>(
        frc1678::elevator_controller::controller::second_stage_cube_integral::K(),
        frc1678::elevator_controller::controller::second_stage_cube_integral::Kff(),
        frc1678::elevator_controller::controller::second_stage_cube_integral::A(),
        Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
        Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

    elevator_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
        elevator_plant, frc1678::elevator_controller::controller::second_stage_cube_integral::L());

    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::second_stage_cube_integral::A(),
        frc1678::elevator_controller::controller::second_stage_cube_integral::B(),
        frc1678::elevator_controller::controller::second_stage_cube_integral::C());
  } else if (second_stage && !has_cube) {
    auto elevator_plant = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::second_stage_integral::A(),
        frc1678::elevator_controller::controller::second_stage_integral::B(),
        frc1678::elevator_controller::controller::second_stage_integral::C());

    elevator_controller_ = muan::control::StateSpaceController<1, 3, 1>(
        frc1678::elevator_controller::controller::second_stage_integral::K(),
        frc1678::elevator_controller::controller::second_stage_integral::Kff(),
        frc1678::elevator_controller::controller::second_stage_integral::A(),
        Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
        Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

    elevator_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
        elevator_plant, frc1678::elevator_controller::controller::second_stage_integral::L());

    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::second_stage_integral::A(),
        frc1678::elevator_controller::controller::second_stage_integral::B(),
        frc1678::elevator_controller::controller::second_stage_integral::C());
  } else if (!second_stage && has_cube) {
    auto elevator_plant = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::first_stage_cube_integral::A(),
        frc1678::elevator_controller::controller::first_stage_cube_integral::B(),
        frc1678::elevator_controller::controller::first_stage_cube_integral::C());

    elevator_controller_ = muan::control::StateSpaceController<1, 3, 1>(
        frc1678::elevator_controller::controller::first_stage_cube_integral::K(),
        frc1678::elevator_controller::controller::first_stage_cube_integral::Kff(),
        frc1678::elevator_controller::controller::first_stage_cube_integral::A(),
        Eigen::Matrix<double, 1, 1>::Ones() * -std::numeric_limits<double>::infinity(),
        Eigen::Matrix<double, 1, 1>::Ones() * std::numeric_limits<double>::infinity());

    elevator_observer_ = muan::control::StateSpaceObserver<1, 3, 1>(
        elevator_plant, frc1678::elevator_controller::controller::first_stage_cube_integral::L());

    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::first_stage_cube_integral::A(),
        frc1678::elevator_controller::controller::first_stage_cube_integral::B(),
        frc1678::elevator_controller::controller::first_stage_cube_integral::C());
  } else if (!second_stage && !has_cube) {
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
  }
}

}  // namespace elevator

}  // namespace score_subsystem

}  // namespace c2018
