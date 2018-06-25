#include "c2018_rewrite/subsystems/score_subsystem/score_subsystem.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {
namespace subsystems {
namespace score_subsystem {

class ScoreSubsystemTest : public ::testing::Test {
 public:
  void UpdateInput() {
    score_subsystem_input_proto_->set_intake_proxy(has_cube_);

    score_subsystem_input_proto_->set_wrist_voltage(wrist_voltage_);

    if (score_subsystem_output_proto_->wrist_output_type() == POSITION) {
      score_subsystem_input_proto_->set_wrist_encoder(
          score_subsystem_output_proto_->wrist_setpoint() + wrist_offset_);
      score_subsystem_input_proto_->set_wrist_velocity(
          (score_subsystem_output_proto_->wrist_setpoint() -
           prev_wrist_position_) /
          0.01);
      prev_wrist_position_ = score_subsystem_input_proto_->wrist_encoder();
    }
    if (calibrate_test_) {
      score_subsystem_input_proto_->set_wrist_hall(
          std::abs(calibrate_angle_ - claw::kHallEffectAngle) < 2e-2);
    } else {
      score_subsystem_input_proto_->set_wrist_hall(
          std::abs(score_subsystem_status_proto_->wrist_angle() -
                   claw::kHallEffectAngle) < 2e-2);
    }

    score_subsystem_input_proto_->set_elevator_voltage(elevator_voltage_);
    if (score_subsystem_output_proto_->elevator_output_type() == POSITION) {
      score_subsystem_input_proto_->set_elevator_encoder(
          score_subsystem_output_proto_->elevator_setpoint() + wrist_offset_);
      score_subsystem_input_proto_->set_elevator_velocity(
          (score_subsystem_output_proto_->elevator_setpoint() -
           prev_elevator_position_) /
          0.01);
      prev_elevator_position_ =
          score_subsystem_input_proto_->elevator_encoder();
    }
    if (calibrate_test_) {
      score_subsystem_input_proto_->set_elevator_hall(
          std::abs(calibrate_height_ - elevator::kHallEffectHeight) < 2e-2);
    } else {
      score_subsystem_input_proto_->set_elevator_hall(
          std::abs(score_subsystem_status_proto_->elevator_height() -
                   elevator::kHallEffectHeight) < 2e-2);
    }
  }

  void CalibrateDisabled() {
    calibrate_test_ = true;
    driver_station_proto_->set_is_sys_active(false);

    for (int i = 0; i < 2500; i++) {
      double h = i * 5e-4 + elevator_offset_;
      double theta = i * 5e-4 + wrist_offset_;
      calibrate_angle_ = h - elevator_offset_;
      calibrate_height_ = theta - wrist_offset_;
      score_subsystem_input_proto_->set_elevator_encoder(h);
      score_subsystem_input_proto_->set_wrist_encoder(theta);

      Update();
      if (i < 1000) {
        EXPECT_EQ(score_subsystem_status_proto_->score_state(),
                  ScoreState::CALIBRATING);
      }
    }

    EXPECT_TRUE(score_subsystem_status_proto_->elevator_calibrated());
    EXPECT_TRUE(score_subsystem_status_proto_->wrist_calibrated());
    EXPECT_EQ(score_subsystem_status_proto_->score_state(),
              ScoreState::HOLDING);

    calibrate_test_ = false;
  }

  void Update() {
    UpdateInput();

    WriteMessages();

    score_subsystem_.Update();

    ReadMessages();
  }

  void RunFor(int num_ticks) {
    for (int i = 0; i < num_ticks; i++) {
      Update();
    }
    LogicCheck();
  }

  void LogicCheck() {
    if (score_subsystem_status_proto_->elevator_height() < 0.89 ||
        score_subsystem_status_proto_->elevator_unprofiled_goal() < 0.89) {
      EXPECT_LE(score_subsystem_status_proto_->wrist_unprofiled_goal(),
                kWristSafeAngle);
      EXPECT_LE(score_subsystem_status_proto_->wrist_angle(), kWristSafeAngle);
    }

    if (score_subsystem_status_proto_->wrist_angle() > M_PI / 2) {
      EXPECT_GE(score_subsystem_status_proto_->elevator_unprofiled_goal(),
                kElevatorWristSafeHeight);
      EXPECT_GE(score_subsystem_status_proto_->elevator_height(),
                kElevatorWristSafeHeight);
    }
  }

  void ReadMessages() {
    score_subsystem_output_queue_.ReadLastMessage(
        &score_subsystem_output_proto_);
    score_subsystem_status_queue_.ReadLastMessage(
        &score_subsystem_status_proto_);
  }

  void WriteMessages() {
    score_subsystem_input_queue_->WriteMessage(score_subsystem_input_proto_);
    score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoal(ScoreGoal score_goal, IntakeMode intake_goal,
               bool outputs_enabled) {
    score_subsystem_goal_proto_->set_score_goal(score_goal);
    score_subsystem_goal_proto_->set_intake_goal(intake_goal);
    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetInput(double elevator_encoder, bool elevator_hall,
                double wrist_encoder, bool wrist_hall) {
    score_subsystem_input_proto_->set_elevator_encoder(elevator_encoder);
    score_subsystem_input_proto_->set_wrist_encoder(wrist_encoder);
    score_subsystem_input_proto_->set_elevator_hall(elevator_hall);
    score_subsystem_input_proto_->set_wrist_hall(wrist_hall);
  }

  bool outputs_enabled_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  ScoreSubsystemGoalQueue* score_subsystem_goal_queue_ =
      muan::queues::QueueManager<ScoreSubsystemGoalProto>::Fetch();

  ScoreSubsystemInputQueue* score_subsystem_input_queue_ =
      muan::queues::QueueManager<ScoreSubsystemInputProto>::Fetch();

  ScoreSubsystemStatusQueue::QueueReader score_subsystem_status_queue_ =
      muan::queues::QueueManager<ScoreSubsystemStatusProto>::Fetch()
          ->MakeReader();

  ScoreSubsystemOutputQueue::QueueReader score_subsystem_output_queue_ =
      muan::queues::QueueManager<ScoreSubsystemOutputProto>::Fetch()
          ->MakeReader();

  muan::wpilib::DriverStationProto driver_station_proto_;
  ScoreSubsystemGoalProto score_subsystem_goal_proto_;
  ScoreSubsystemInputProto score_subsystem_input_proto_;
  ScoreSubsystemStatusProto score_subsystem_status_proto_;
  ScoreSubsystemOutputProto score_subsystem_output_proto_;

 protected:
  double elevator_voltage_ = elevator::kEncoderFaultMinVoltage - 0.1;
  double wrist_voltage_ = claw::kEncoderFaultMinVoltage - 0.1;
  double elevator_offset_ = 0.0;
  double wrist_offset_ = 0.0;
  double prev_elevator_position_;
  double prev_wrist_position_;
  double calibrate_angle_;
  double calibrate_height_;

  bool has_cube_ = false;
  bool calibrate_test_ = false;

  void CheckGoal(double elevator, double wrist) const {
    EXPECT_NEAR(score_subsystem_status_proto_->elevator_unprofiled_goal(),
                elevator, 1e-3);
    EXPECT_NEAR(score_subsystem_status_proto_->wrist_unprofiled_goal(), wrist,
                1e-3);

    EXPECT_NEAR(score_subsystem_status_proto_->elevator_height(), elevator,
                1e-3);
    EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), wrist, 1e-3);
  }

 private:
  ScoreSubsystem score_subsystem_;
};

TEST_F(ScoreSubsystemTest, Disabled) {
  SetGoal(ScoreGoal::INTAKE_2, IntakeMode::INTAKE, false);
  SetInput(0, false, 0, false);

  Update();

  EXPECT_EQ(score_subsystem_output_proto_->elevator_setpoint(), 0);
  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_output_proto_->wrist_setpoint(), 0);
  EXPECT_FALSE(score_subsystem_output_proto_->intake_open());
  EXPECT_FALSE(score_subsystem_output_proto_->intake_close());
}

TEST_F(ScoreSubsystemTest, DisabledCalibrates) {
  elevator_offset_ = 1.0;
  wrist_offset_ = 1.0;
  CalibrateDisabled();
  RunFor(100);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), calibrate_angle_,
              1e-2);
  EXPECT_NEAR(score_subsystem_status_proto_->elevator_height(),
              calibrate_height_, 1e-2);
}

TEST_F(ScoreSubsystemTest, MoveTo) {
  CalibrateDisabled();

  // Intake 0
  SetGoal(ScoreGoal::INTAKE_0, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorIntake0, kWristForwardAngle);

  // Intake 1
  SetGoal(ScoreGoal::INTAKE_1, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorIntake1, kWristForwardAngle);

  // Intake 2
  SetGoal(ScoreGoal::INTAKE_2, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorIntake2, kWristForwardAngle);

  // Force stow
  SetGoal(ScoreGoal::STOW, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorStow, kWristStowAngle);

  // Switch
  SetGoal(ScoreGoal::SWITCH, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorSwitch, kWristForwardAngle);

  // Scale low forward
  SetGoal(ScoreGoal::SCALE_LOW_FORWARD, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorBaseHeight, kWristForwardAngle);

  // Scale low reverse
  SetGoal(ScoreGoal::SCALE_LOW_REVERSE, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorBaseHeight + kElevatorReversedOffset, kWristBackwardAngle);

  // Scale mid forward
  SetGoal(ScoreGoal::SCALE_MID_FORWARD, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorBaseHeight + kCubeHeight, kWristForwardAngle);

  // Scale mid reverse
  SetGoal(ScoreGoal::SCALE_MID_REVERSE, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorBaseHeight + kCubeHeight + kElevatorReversedOffset,
            kWristBackwardAngle);

  // Scale high forward
  SetGoal(ScoreGoal::SCALE_HIGH_FORWARD, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(elevator::kMaxHeight, kWristForwardAngle);

  // Scale high reverse
  SetGoal(ScoreGoal::SCALE_HIGH_REVERSE, IntakeMode::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kElevatorBaseHeight + 2 * kCubeHeight + kElevatorReversedOffset,
            kWristBackwardAngle);
}

TEST_F(ScoreSubsystemTest, IntakeManual) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_0, IntakeMode::INTAKE, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            claw::kIntakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), INTAKING_TO_STOW);
  EXPECT_EQ(score_subsystem_status_proto_->intake_state(), INTAKE);

  SetGoal(ScoreGoal::INTAKE_0, IntakeMode::INTAKE_NONE, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), HOLDING);
}

TEST_F(ScoreSubsystemTest, OuttakeManual) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_0, IntakeMode::OUTTAKE_FAST, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            claw::kFastOuttakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), HOLDING);

  SetGoal(ScoreGoal::INTAKE_0, IntakeMode::INTAKE_NONE, true);
  Update();

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), HOLDING);
}

// When we're intaking on the ground, it should go to stow afterwards
// automatically
TEST_F(ScoreSubsystemTest, IntakeToStow) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_0, IntakeMode::INTAKE, true);
  RunFor(10);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE, true);

  RunFor(500);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            claw::kIntakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), INTAKING_TO_STOW);
  EXPECT_EQ(score_subsystem_status_proto_->intake_state(), INTAKE);

  has_cube_ = true;
  RunFor(600);

  EXPECT_TRUE(score_subsystem_status_proto_->has_cube());
  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            claw::kHoldingVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), HOLDING);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_unprofiled_goal(),
              kWristStowAngle, 1e-3);
}

// When we're not intaking on the ground, it should stay in place when it gets a
// cube but stop intaking
TEST_F(ScoreSubsystemTest, IntakeToHolding) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::INTAKE_1, IntakeMode::INTAKE, true);
  RunFor(10);
  SetGoal(ScoreGoal::SCORE_NONE, IntakeMode::INTAKE, true);

  RunFor(500);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            claw::kIntakeVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), INTAKING_ONLY);
  EXPECT_EQ(score_subsystem_status_proto_->intake_state(), INTAKE);

  has_cube_ = true;
  RunFor(600);

  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(),
            claw::kHoldingVoltage);
  EXPECT_EQ(score_subsystem_status_proto_->score_state(), HOLDING);

  EXPECT_NEAR(score_subsystem_status_proto_->wrist_unprofiled_goal(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->elevator_unprofiled_goal(),
              kElevatorIntake1, 1e-3);
}

}  // namespace score_subsystem
}  // namespace subsystems
}  // namespace c2018
