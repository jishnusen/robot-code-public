#include "c2018/subsystems/score_subsystem/wrist/wrist.h"
#include "gtest/gtest.h"

namespace c2018 {

namespace score_subsystem {

namespace wrist {

class WristTest : public ::testing::Test {
 public:
  WristTest() {
    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::wrist::controller::cube_integral::A(),
        frc1678::wrist::controller::cube_integral::B(),
        frc1678::wrist::controller::cube_integral::C());
  }
  // UPDATE
  void Update() {
    if (plant_.x(0) < 0) {
      plant_.x(0) = 0;
    }
    wrist_input_proto_->set_wrist_hall(plant_.x(0) >= 0.21 &&
                                       plant_.x(0) <= 0.25);
    wrist_.Update(wrist_input_proto_, &wrist_output_proto_,
                  &wrist_status_proto_, outputs_enabled_);

    SetWeights(wrist_input_proto_->has_cube());
    plant_.Update(
        (Eigen::Matrix<double, 1, 1>() << wrist_output_proto_->wrist_voltage())
            .finished());
  }

  void SetGoal(double angle, IntakeGoal intake_mode) {
    wrist_.SetGoal(angle, intake_mode);
  }

  ScoreSubsystemInputProto wrist_input_proto_;
  ScoreSubsystemOutputProto wrist_output_proto_;
  ScoreSubsystemStatusProto wrist_status_proto_;
  ScoreSubsystemGoalProto wrist_goal_proto_;

  double wrist_voltage_;
  bool outputs_enabled_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> plant_;

 private:
  WristController wrist_;
  void SetWeights(bool has_cube) {
    if (has_cube) {
      plant_.A() = frc1678::wrist::controller::cube_integral::A();
      plant_.B() = frc1678::wrist::controller::cube_integral::B();
      plant_.C() = frc1678::wrist::controller::cube_integral::C();
    } else {
      plant_.A() = frc1678::wrist::controller::no_cube_integral::A();
      plant_.B() = frc1678::wrist::controller::no_cube_integral::B();
      plant_.C() = frc1678::wrist::controller::no_cube_integral::C();
    }
  }
};

TEST_F(WristTest, Calib) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(M_PI / 4, IntakeGoal::INTAKE_NONE);

  double offset = -M_PI / 2;

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0) + offset);
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), M_PI / 4, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
}

TEST_F(WristTest, IntakeDefault) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, IntakeGoal::INTAKE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, IntakeModes) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, IntakeGoal::INTAKE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());

  SetGoal(0.0, IntakeGoal::INTAKE_OPEN);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());

  SetGoal(0.0, IntakeGoal::INTAKE_CLOSE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());

  SetGoal(0.0, IntakeGoal::SETTLE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 12.0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());

  SetGoal(0.0, IntakeGoal::OUTTAKE_SLOW);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kSlowOuttakeVoltage, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());

  SetGoal(0.0, IntakeGoal::OUTTAKE_FAST);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kFastOuttakeVoltage, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());

  SetGoal(0.0, IntakeGoal::DROP);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Idle) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, IntakeGoal::INTAKE_NONE);
  Update();

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Stow) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(kWristStowAngle, IntakeGoal::INTAKE_NONE);
  Update();

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), kWristStowAngle, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
  EXPECT_NEAR(wrist_status_proto_->wrist_profiled_goal(), kWristStowAngle,
              1e-3);
  EXPECT_NEAR(wrist_status_proto_->wrist_unprofiled_goal(), kWristStowAngle,
              1e-3);
}

TEST_F(WristTest, Backwards) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(kWristMaxAngle, IntakeGoal::INTAKE_NONE);
  Update();

  for (int i = 0; i < 1000; i++) {
    wrist_input_proto_->set_wrist_encoder(plant_.y(0));
    Update();
    EXPECT_NEAR(wrist_output_proto_->wrist_voltage(), 0, 12);
  }

  EXPECT_NEAR(wrist_status_proto_->wrist_angle(), kWristMaxAngle, 1e-3);
  EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
  EXPECT_NEAR(wrist_status_proto_->wrist_profiled_goal(), kWristMaxAngle, 1e-3);
  EXPECT_NEAR(wrist_status_proto_->wrist_unprofiled_goal(), kWristMaxAngle,
              1e-3);

  EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(wrist_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(wrist_output_proto_->wrist_solenoid_close());
}

TEST_F(WristTest, Disabled) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = false;

  SetGoal(kWristMaxAngle, IntakeGoal::INTAKE);

  Update();

  EXPECT_EQ(wrist_output_proto_->wrist_voltage(), 0.0);
  EXPECT_EQ(wrist_output_proto_->intake_voltage(), 0.0);
}

TEST_F(WristTest, CanCapAngle) {
  outputs_enabled_ = true;
  SetGoal(6e5, IntakeGoal::INTAKE_NONE);

  Update();

  EXPECT_TRUE(wrist_status_proto_->wrist_profiled_goal() <= kWristMaxAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_unprofiled_goal() >= kWristMinAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_profiled_goal() <= kWristMaxAngle);
  EXPECT_TRUE(wrist_status_proto_->wrist_unprofiled_goal() >= kWristMinAngle);
}

TEST_F(WristTest, SaneHasCube) {
  outputs_enabled_ = true;
  wrist_input_proto_->set_has_cube(true);
  SetGoal(0.0, IntakeGoal::INTAKE_CLOSE);
  Update();

  EXPECT_FALSE(wrist_status_proto_->has_cube());
  EXPECT_EQ(wrist_status_proto_->intake_state(), IntakeState::MOVING);

  for (int i = 0; i < 101; i++) {
    Update();
  }

  EXPECT_EQ(wrist_status_proto_->intake_state(), IntakeState::IDLING_WITH_CUBE);
  EXPECT_TRUE(wrist_status_proto_->has_cube());
}

}  // namespace wrist

}  // namespace score_subsystem

}  // namespace c2018
