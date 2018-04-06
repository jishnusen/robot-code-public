#include "c2018/subsystems/score_subsystem/elevator/elevator.h"
#include "gtest/gtest.h"

namespace c2018 {

namespace score_subsystem {

namespace elevator {

class ElevatorControllerTest : public ::testing::Test {
 public:
  ElevatorControllerTest() {
    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator::controller::first_stage_integral::A(),
        frc1678::elevator::controller::first_stage_integral::B(),
        frc1678::elevator::controller::first_stage_integral::C());
  }

  void Update() {
    if (plant_.x(0) < 0) {
      plant_.x(0) = 0;
    }
    elevator_input_proto_->set_elevator_hall(
        std::abs(plant_.x(0) - kHallEffectHeight) < 2e-2);
    elevator_.Update(elevator_input_proto_, &elevator_output_proto_,
                     &elevator_status_proto_, outputs_enabled_);
    SetWeights(plant_.x()(0, 0) >= 1.0, elevator_input_proto_->has_cube());
    plant_.Update((Eigen::Matrix<double, 1, 1>()
                   << elevator_output_proto_->elevator_voltage())
                      .finished());
  }

  void SetGoal(double goal) { elevator_.SetGoal({goal, 0}); }

  void SetInput(double position, bool hall) {
    elevator_input_proto_->set_elevator_encoder(position);
    elevator_input_proto_->set_elevator_hall(hall);
  }

  void CalibrateDisabled() {
    outputs_enabled_ = false;

    for (int i = 0; i < 2000; i++) {
      double h = i * .0005;
      plant_.x(0) = h;
      elevator_input_proto_->set_elevator_encoder(plant_.y(0));
      Update();
    }
  }

  c2018::score_subsystem::ScoreSubsystemInputProto elevator_input_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto elevator_status_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto elevator_output_proto_;

  bool outputs_enabled_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> plant_;
  ElevatorController elevator_;

 private:
  void SetWeights(bool second_stage, bool has_cube) {
    if (second_stage && has_cube) {
      plant_.A() =
          frc1678::elevator::controller::second_stage_cube_integral::A();
      plant_.B() =
          frc1678::elevator::controller::second_stage_cube_integral::B();
      plant_.C() =
          frc1678::elevator::controller::second_stage_cube_integral::C();
    } else if (second_stage && !has_cube) {
      plant_.A() = frc1678::elevator::controller::second_stage_integral::A();
      plant_.B() = frc1678::elevator::controller::second_stage_integral::B();
      plant_.C() = frc1678::elevator::controller::second_stage_integral::C();
    } else if (!second_stage && has_cube) {
      plant_.A() =
          frc1678::elevator::controller::first_stage_cube_integral::A();
      plant_.B() =
          frc1678::elevator::controller::first_stage_cube_integral::B();
      plant_.C() =
          frc1678::elevator::controller::first_stage_cube_integral::C();
    } else if (!second_stage && !has_cube) {
      plant_.A() = frc1678::elevator::controller::first_stage_integral::A();
      plant_.B() = frc1678::elevator::controller::first_stage_integral::B();
      plant_.C() = frc1678::elevator::controller::first_stage_integral::C();
    }
  }
};

TEST_F(ElevatorControllerTest, NotEnabled) {
  SetGoal(1);

  outputs_enabled_ = false;

  Update();

  EXPECT_EQ(elevator_status_proto_->elevator_actual_height(), 0.);
  EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0., 1e-3);
}

TEST_F(ElevatorControllerTest, Calibration) {
  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  double offset = 1.0;

  SetGoal(kElevatorMaxHeight);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0) + offset);
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_TRUE(elevator_status_proto_->elevator_calibrated());
  EXPECT_TRUE(elevator_status_proto_->elevator_at_top());
  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(),
              kElevatorMaxHeight, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(),
              kElevatorMaxHeight, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(),
              kElevatorMaxHeight, 1e-3);
}

TEST_F(ElevatorControllerTest, AllHeights) {
  CalibrateDisabled();
  EXPECT_TRUE(elevator_status_proto_->elevator_calibrated());

  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(0.6);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0.6, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0.6, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 0.6, 1e-3);

  SetGoal(kElevatorMaxHeight);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_TRUE(elevator_status_proto_->elevator_at_top());
  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(),
              kElevatorMaxHeight, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(),
              kElevatorMaxHeight, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(),
              kElevatorMaxHeight, 1e-3);
  SetGoal(0);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 0, 1e-3);
  SetGoal(0.3);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0.3, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0.3, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 0.3, 1e-3);
}

TEST_F(ElevatorControllerTest, EncoderFault) {
  CalibrateDisabled();

  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(kElevatorMaxHeight);

  for (int i = 0; i < 400; i++) {
    elevator_input_proto_->set_elevator_encoder(0);
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_TRUE(elevator_status_proto_->elevator_encoder_fault_detected());
  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0, 1e-3);
}

TEST_F(ElevatorControllerTest, HeightTooHigh) {
  CalibrateDisabled();

  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(4000);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(),
              kElevatorMaxHeight, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(),
              kElevatorMaxHeight, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(),
              kElevatorMaxHeight, 1e-3);
}

TEST_F(ElevatorControllerTest, GodModePositiveVelocity) {
  CalibrateDisabled();

  outputs_enabled_ = true;

  elevator_.SetGoal({0., 1.5}, true);
  for (int i = 0; i < 1000 && elevator_status_proto_->elevator_actual_height() <
                                  kElevatorMaxHeight - 0.5;
       i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_NEAR(elevator_status_proto_->estimated_velocity(), 1.5, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0., 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal_velocity(), 1.5,
              1e-3);
  EXPECT_TRUE(elevator_status_proto_->elevator_god_mode());
}

TEST_F(ElevatorControllerTest, GodModeNegativeVelocity) {
  CalibrateDisabled();

  outputs_enabled_ = true;

  elevator_.SetGoal({0., -1.5}, true);
  for (int i = 0;
       i < 1000 && elevator_status_proto_->elevator_actual_height() > 0.1;
       i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);
  }

  EXPECT_NEAR(elevator_status_proto_->estimated_velocity(), -1.5, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0., 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal_velocity(), -1.5,
              1e-3);
  EXPECT_TRUE(elevator_status_proto_->elevator_god_mode());
}

TEST_F(ElevatorControllerTest, GodModeCappingTop) {
  CalibrateDisabled();

  outputs_enabled_ = true;

  elevator_.SetGoal({0., 20000.}, true);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);

    EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal_velocity(), 0.,
                kElevatorMaxVelocity);

    if (elevator_status_proto_->elevator_actual_height() >
        kElevatorMaxHeight - 1e-2) {
      EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal_velocity(),
                  0., 1e-7);
    }
  }

  EXPECT_LE(elevator_status_proto_->estimated_velocity(), 1e-7);
}

TEST_F(ElevatorControllerTest, GodModeCappingBottom) {
  CalibrateDisabled();

  outputs_enabled_ = true;

  elevator_.SetGoal({0., -20000.}, true);

  for (int i = 0; i < 1000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0, 12);

    EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal_velocity(), 0.,
                kElevatorMaxVelocity);

    if (elevator_status_proto_->elevator_actual_height() < 1e-2) {
      EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal_velocity(),
                  0., 1e-7);
    }
  }

  EXPECT_GE(elevator_status_proto_->estimated_velocity(), -1e-7);
}

}  // namespace elevator

}  // namespace score_subsystem

}  // namespace c2018
