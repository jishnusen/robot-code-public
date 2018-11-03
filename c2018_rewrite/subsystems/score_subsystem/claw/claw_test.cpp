#include "c2018_rewrite/subsystems/score_subsystem/claw/claw.h"
#include "gtest/gtest.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {
namespace claw {

class ClawTest : public ::testing::Test {
 public:
  void Update() {
    claw_input_proto_->set_intake_proxy(has_cube_);
    claw_input_proto_->set_wrist_voltage(voltage_);
    if (claw_output_proto_->wrist_output_type() == POSITION) {
      claw_input_proto_->set_wrist_encoder(
          claw_output_proto_->wrist_setpoint() + offset_);
      claw_input_proto_->set_wrist_velocity(
          (claw_output_proto_->wrist_setpoint() - prev_position_) / 0.01);
      prev_position_ = claw_input_proto_->wrist_encoder();
    }
    if (calibrate_test_) {
      claw_input_proto_->set_wrist_hall(
          std::abs(calibrate_angle_ - kHallEffectAngle) < 2e-2);
    } else {
      claw_input_proto_->set_wrist_hall(
          std::abs(claw_status_proto_->wrist_angle() - kHallEffectAngle) <
          2e-2);
    }
    claw_.Update(claw_input_proto_, &claw_output_proto_, &claw_status_proto_,
                 outputs_enabled_);
  }

  bool NotBroken() {
    return (claw_status_proto_->wrist_angle() > kMinAngle - 1e-6 &&
            claw_status_proto_->wrist_angle() < kMaxAngle + 1e-6);
  }

  void CheckGoal() {
    EXPECT_NEAR(claw_status_proto_->wrist_angle(),
                claw_status_proto_->wrist_profiled_goal(), 1e-9);
    EXPECT_NEAR(claw_status_proto_->wrist_angle(),
                claw_status_proto_->wrist_unprofiled_goal(), 1e-9);
    switch (intake_goal_) {
      case INTAKE_NONE:
        EXPECT_TRUE(claw_output_proto_->intake_close());
        EXPECT_FALSE(claw_output_proto_->intake_open());
        if (claw_status_proto_->has_cube()) {
          EXPECT_NEAR(claw_output_proto_->intake_voltage(), kHoldingVoltage,
                      1e-9);
        } else {
          EXPECT_NEAR(claw_output_proto_->intake_voltage(), 0., 1e-9);
        }
        break;
      case INTAKE:
        EXPECT_NEAR(claw_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case INTAKE_OPEN:
        EXPECT_TRUE(claw_output_proto_->intake_open());
        EXPECT_FALSE(claw_output_proto_->intake_close());
        EXPECT_NEAR(claw_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case INTAKE_CLOSE:
      case SETTLE:
        EXPECT_TRUE(claw_output_proto_->intake_close());
        EXPECT_FALSE(claw_output_proto_->intake_open());
        EXPECT_NEAR(claw_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case OUTTAKE_SLOW:
        EXPECT_FALSE(claw_output_proto_->intake_open());
        EXPECT_TRUE(claw_output_proto_->intake_close());
        EXPECT_NEAR(claw_output_proto_->intake_voltage(), kSlowOuttakeVoltage, 1e-9);
        break;
      case OUTTAKE_FAST:
        EXPECT_FALSE(claw_output_proto_->intake_open());
        EXPECT_TRUE(claw_output_proto_->intake_close());
        EXPECT_NEAR(claw_output_proto_->intake_voltage(), kFastOuttakeVoltage, 1e-9);
        break;
      case DROP:
        EXPECT_TRUE(claw_output_proto_->intake_open());
        EXPECT_FALSE(claw_output_proto_->intake_close());
        EXPECT_NEAR(claw_output_proto_->intake_voltage(), 0., 1e-9);
        break;
    }
  }

  void RunFor(int ticks) {
    for (int i = 0; i < ticks; i++) {
      Update();
      EXPECT_TRUE(NotBroken());
    }
    CheckGoal();
  }

  void SetGoal(double goal, IntakeMode intake_goal = INTAKE_NONE) {
    intake_goal_ = intake_goal;
    claw_.SetGoal(goal, intake_goal);
  }

  void SetInput(double position, bool hall) {
    claw_input_proto_->set_wrist_encoder(position);
    claw_input_proto_->set_wrist_hall(hall);
  }

  void CalibrateDisabled(double offset = 0) {
    offset_ = offset;
    outputs_enabled_ = false;
    calibrate_test_ = true;

    for (int i = 0; i < 2000; i++) {
      double h = i * .0005 + offset;
      calibrate_angle_ = h - offset;
      claw_input_proto_->set_wrist_encoder(h);
      Update();
    }
    EXPECT_TRUE(claw_status_proto_->wrist_calibrated());
    EXPECT_NEAR(claw_status_proto_->wrist_angle(), 1., 1e-3);
  }

  ScoreSubsystemInputProto claw_input_proto_;
  ScoreSubsystemStatusProto claw_status_proto_;
  ScoreSubsystemOutputProto claw_output_proto_;

  bool outputs_enabled_;
  bool has_cube_ = false;
  double voltage_ = kEncoderFaultMinVoltage - 0.1;

 protected:
  Claw claw_;

 private:
  double prev_position_;
  double offset_;
  double calibrate_angle_;

  bool calibrate_test_;
  IntakeMode intake_goal_;
};

TEST_F(ClawTest, NotEnabled) {
  SetGoal(1);

  outputs_enabled_ = false;

  Update();

  EXPECT_EQ(claw_status_proto_->wrist_angle(), 0.);
  EXPECT_EQ(claw_output_proto_->wrist_output_type(), OPEN_LOOP);
  EXPECT_NEAR(claw_output_proto_->wrist_setpoint(), 0., 1e-3);
}

TEST_F(ClawTest, CalibrateDisabled) { CalibrateDisabled(1.); }

TEST_F(ClawTest, AllAngles) {
  CalibrateDisabled();

  claw_input_proto_->set_wrist_encoder(0);
  claw_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(0.6);
  RunFor(1000);
  CheckGoal();

  SetGoal(kMaxAngle);
  RunFor(1000);
  CheckGoal();

  SetGoal(0);
  RunFor(1000);
  CheckGoal();

  SetGoal(0.3);
  RunFor(1000);
  CheckGoal();
}

TEST_F(ClawTest, EncoderFault) {
  CalibrateDisabled();

  claw_input_proto_->set_wrist_encoder(0);
  claw_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(kMaxAngle);

  for (int i = 0; i < 400; i++) {
    claw_input_proto_->set_wrist_encoder(0);
    claw_input_proto_->set_wrist_voltage(12.);
    claw_.Update(claw_input_proto_, &claw_output_proto_, &claw_status_proto_,
                 outputs_enabled_);
  }

  EXPECT_TRUE(claw_status_proto_->wrist_encoder_fault());
  EXPECT_NEAR(claw_status_proto_->wrist_angle(), 0, 1e-3);
}

TEST_F(ClawTest, Cap) {
  CalibrateDisabled();

  claw_input_proto_->set_wrist_encoder(0);
  claw_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(4000);
  RunFor(1000);
  CheckGoal();

  SetGoal(-4000);
  RunFor(1000);
  CheckGoal();
}

TEST_F(ClawTest, IntakeModes) {
  claw_input_proto_->set_wrist_encoder(0);
  claw_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, INTAKE);
  Update();
  CheckGoal();

  SetGoal(0.0, INTAKE_OPEN);
  Update();
  CheckGoal();

  SetGoal(0.0, INTAKE_CLOSE);
  Update();
  CheckGoal();

  SetGoal(0.0, SETTLE);
  Update();
  CheckGoal();

  SetGoal(0.0, OUTTAKE_SLOW);
  Update();
  CheckGoal();

  SetGoal(0.0, OUTTAKE_FAST);
  Update();
  CheckGoal();

  SetGoal(0.0, DROP);
  Update();
  CheckGoal();

  // Tests PinchState as well
  SetGoal(0.0, INTAKE_NONE);
  has_cube_ = true;
  /* RunFor(1); */
  /* EXPECT_FALSE(claw_status_proto_->has_cube()); */
  /* CheckGoal(); */

  RunFor(600);
  EXPECT_TRUE(claw_status_proto_->has_cube());
  CheckGoal();
}

}  // namespace claw
}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018
