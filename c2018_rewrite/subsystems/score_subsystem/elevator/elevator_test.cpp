#include "c2018_rewrite/subsystems/score_subsystem/elevator/elevator.h"
#include "gtest/gtest.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {
namespace elevator {

class ElevatorTest : public ::testing::Test {
 public:
  void Update() {
    elevator_input_proto_->set_intake_proxy(has_cube_);
    elevator_input_proto_->set_elevator_voltage(voltage_);
    if (elevator_output_proto_->elevator_output_type() == POSITION) {
      elevator_input_proto_->set_elevator_encoder(
          elevator_output_proto_->elevator_setpoint() + offset_);
      elevator_input_proto_->set_elevator_velocity(
          (elevator_output_proto_->elevator_setpoint() - prev_position_) /
          0.01);
      prev_position_ = elevator_input_proto_->elevator_encoder();
    }
    if (calibrate_test_) {
      elevator_input_proto_->set_elevator_hall(
          std::abs(calibrate_height_ - kHallEffectHeight) < 2e-2);
    } else {
      elevator_input_proto_->set_elevator_hall(
          std::abs(elevator_status_proto_->elevator_height() -
                   kHallEffectHeight) < 2e-2);
    }
    elevator_.Update(elevator_input_proto_, &elevator_output_proto_,
                     &elevator_status_proto_, outputs_enabled_);
  }

  bool NotBroken() {
    return (elevator_status_proto_->elevator_height() > kMinHeight - 1e-6 &&
            elevator_status_proto_->elevator_height() < kMaxHeight + 1e-6);
  }

  void CheckGoal() {
    EXPECT_NEAR(elevator_status_proto_->elevator_height(),
                elevator_status_proto_->elevator_profiled_goal(), 1e-9);
    EXPECT_NEAR(elevator_status_proto_->elevator_height(),
                elevator_status_proto_->elevator_unprofiled_goal(), 1e-9);
  }

  void RunFor(int ticks) {
    for (int i = 0; i < ticks; i++) {
      Update();
      EXPECT_TRUE(NotBroken());
    }
    CheckGoal();
  }

  void SetGoal(double goal) { elevator_.SetGoal(goal); }

  void SetInput(double position, bool hall) {
    elevator_input_proto_->set_elevator_encoder(position);
    elevator_input_proto_->set_elevator_hall(hall);
  }

  void CalibrateDisabled(double offset = 0) {
    offset_ = offset;
    outputs_enabled_ = false;
    calibrate_test_ = true;

    for (int i = 0; i < 2000; i++) {
      double h = i * .0005 + offset;
      calibrate_height_ = h - offset;
      elevator_input_proto_->set_elevator_encoder(h);
      Update();
    }
    EXPECT_TRUE(elevator_status_proto_->elevator_calibrated());
    EXPECT_NEAR(elevator_status_proto_->elevator_height(), 1., 1e-3);
  }

  ScoreSubsystemInputProto elevator_input_proto_;
  ScoreSubsystemStatusProto elevator_status_proto_;
  ScoreSubsystemOutputProto elevator_output_proto_;

  bool outputs_enabled_;
  bool has_cube_ = false;
  double voltage_ = kEncoderFaultMinVoltage - 0.1;

 protected:
  Elevator elevator_;

 private:
  double prev_position_;
  double offset_;
  double calibrate_height_;

  bool calibrate_test_;
};

TEST_F(ElevatorTest, NotEnabled) {
  SetGoal(1);

  outputs_enabled_ = false;

  Update();

  EXPECT_EQ(elevator_status_proto_->elevator_height(), 0.);
  EXPECT_EQ(elevator_output_proto_->elevator_output_type(), OPEN_LOOP);
  EXPECT_NEAR(elevator_output_proto_->elevator_setpoint(), 0., 1e-3);
}

TEST_F(ElevatorTest, CalibrateDisabled) { CalibrateDisabled(1.); }

TEST_F(ElevatorTest, AllHeights) {
  CalibrateDisabled();

  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(0.6);
  RunFor(1000);
  CheckGoal();

  SetGoal(kMaxHeight);
  RunFor(1000);
  CheckGoal();

  EXPECT_TRUE(elevator_status_proto_->elevator_at_top());

  SetGoal(0);
  RunFor(1000);
  CheckGoal();

  SetGoal(0.3);
  RunFor(1000);
  CheckGoal();
}

TEST_F(ElevatorTest, EncoderFault) {
  CalibrateDisabled();

  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(kMaxHeight);

  for (int i = 0; i < 400; i++) {
    elevator_input_proto_->set_elevator_encoder(0);
    elevator_input_proto_->set_elevator_voltage(12.);
    elevator_.Update(elevator_input_proto_, &elevator_output_proto_,
                     &elevator_status_proto_, outputs_enabled_);
  }

  EXPECT_TRUE(elevator_status_proto_->elevator_encoder_fault());
  EXPECT_NEAR(elevator_status_proto_->elevator_height(), 0, 1e-3);
}

TEST_F(ElevatorTest, Cap) {
  CalibrateDisabled();

  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(4000);
  RunFor(1000);
  CheckGoal();

  SetGoal(-4000);
  RunFor(1000);
  CheckGoal();
}

}  // namespace elevator
}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018
