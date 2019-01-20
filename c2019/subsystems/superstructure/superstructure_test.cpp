#include "c2019/subsystems/superstructure/superstructure.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {

namespace superstructure {

class SuperstructureTest : public ::testing::Test {
 public:
  SuperstructureTest() {}

  muan::wpilib::DriverStationProto driver_station_proto_;
  SuperstructureGoalProto superstructure_goal_proto_;
  SuperstructureInputProto superstructure_input_proto_;
  SuperstructureStatusProto superstructure_status_proto_;
  SuperstructureOutputProto superstructure_output_proto_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  SuperstructureInputQueue* superstructure_input_queue_ =
      muan::queues::QueueManager<SuperstructureInputProto>::Fetch();

  SuperstructureGoalQueue* superstructure_goal_queue_ =
      muan::queues::QueueManager<SuperstructureGoalProto>::Fetch();

  SuperstructureStatusQueue::QueueReader superstructure_status_queue_ =
      muan::queues::QueueManager<SuperstructureStatusProto>::Fetch()
          ->MakeReader();

  SuperstructureOutputQueue::QueueReader superstructure_output_queue_ =
      muan::queues::QueueManager<SuperstructureOutputProto>::Fetch()
          ->MakeReader();

  bool outputs_enabled_;

  void UpdateInput() {
    superstructure_input_queue_->WriteMessage(superstructure_input_proto_);
  }

  void CalibrateDisabled() {
    driver_station_proto_->set_is_sys_active(false);
    superstructure_input_proto_->set_elevator_encoder(0);
    superstructure_input_proto_->set_elevator_zeroed(true);
    // TODO(Hanson) make this work
    for (int i = 0; i < 2500; i++) {
      superstructure_input_proto_->set_wrist_encoder(i * 5e-4);
      SetInput(0, true, i * 5e-4);
      Update();
    }

    EXPECT_TRUE(superstructure_status_proto_->elevator_is_calibrated());
    EXPECT_TRUE(superstructure_status_proto_->wrist_is_calibrated());
    EXPECT_EQ(superstructure_status_proto_->state(),
              SuperstructureState::HOLDING);
  }

  void Update() {
    UpdateInput();
    WriteMessages();
    superstructure_.Update();
    ReadMessages();
  }

  void RunFor(int num_ticks) {
    for (int i = 0; i < num_ticks; i++) {
      Update();
    }
    LogicCheck();
  }

  void LogicCheck() {
    // TODO(Hanson) figure out actual safe heights and angles
    if (superstructure_status_proto_->elevator_height() < 0.89 ||
        superstructure_status_proto_->elevator_goal() < 0.89) {
      EXPECT_LE(superstructure_status_proto_->wrist_goal(), kWristSafeAngle);
      EXPECT_LE(superstructure_status_proto_->wrist_angle(), kWristSafeAngle);
    }

    if (superstructure_status_proto_->wrist_angle() > M_PI / 2) {
      EXPECT_GE(superstructure_status_proto_->elevator_goal(),
                kElevatorSafeHeight);
      EXPECT_GE(superstructure_status_proto_->elevator_height(),
                kElevatorSafeHeight);
    }

    EXPECT_NEAR(superstructure_output_proto_->elevator_setpoint(), 0, 12);
    EXPECT_NEAR(superstructure_output_proto_->wrist_setpoint(), 0, 12);
  }

  void ReadMessages() {
    superstructure_output_queue_.ReadLastMessage(&superstructure_output_proto_);
    superstructure_status_queue_.ReadLastMessage(&superstructure_status_proto_);
  }

  void WriteMessages() {
    superstructure_input_queue_->WriteMessage(superstructure_input_proto_);
    superstructure_goal_queue_->WriteMessage(superstructure_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoal(ScoreGoal score_goal, IntakeGoal intake_goal,
               bool outputs_enabled) {
    superstructure_goal_proto_->set_score_goal(score_goal);
    superstructure_goal_proto_->set_intake_goal(intake_goal);
    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetInput(double elevator_encoder, bool elevator_zeroed,
                double wrist_encoder) {
    superstructure_input_proto_->set_elevator_encoder(elevator_encoder);
    superstructure_input_proto_->set_wrist_encoder(wrist_encoder);
    superstructure_input_proto_->set_elevator_zeroed(elevator_zeroed);
    superstructure_input_proto_->set_wrist_hall(
        std::abs(wrist_encoder - wrist::kHallEffectAngle) < 1e-3);
  }

  void SetIntakeInputs(bool has_ground_hatch, bool has_hp_hatch,
                       bool has_cargo) {
    superstructure_input_proto_->set_hatch_ground_current(5000000 *
                                                          has_ground_hatch);
    superstructure_input_proto_->set_hatch_intake_proxy(has_hp_hatch);
    superstructure_input_proto_->set_cargo_proxy(has_cargo);
    WriteMessages();
    std::cout << "has_ground_hatch input: "
              << superstructure_input_proto_->hatch_ground_current()
              << std::endl;
    std::cout << "has_hp_hatch input: "
              << superstructure_input_proto_->hatch_intake_proxy() << std::endl;
    std::cout << "has_cargo input: "
              << superstructure_input_proto_->cargo_proxy() << std::endl;
    if (superstructure_status_proto_->has_ground_hatch() != has_ground_hatch ||
        superstructure_status_proto_->has_hp_hatch() != has_hp_hatch ||
        superstructure_status_proto_->has_cargo() != has_cargo) {
      if (superstructure_status_proto_->has_ground_hatch() !=
          has_ground_hatch) {
        std::cout << "has_ground_hatch doesn't comply with input" << std::endl;
      }
      if (superstructure_status_proto_->has_hp_hatch() != has_hp_hatch) {
        std::cout << "has_hp_hatch doesn't comply with input" << std::endl;
      }
      if (superstructure_status_proto_->has_cargo() != has_cargo) {
        std::cout << "has_cargo doesn't comply with input" << std::endl;
      }
    }
  }

 protected:
  void CheckGoal(double elevator, double wrist) const {
    EXPECT_NEAR(superstructure_status_proto_->elevator_goal(), elevator, 1e-3);
    EXPECT_NEAR(superstructure_status_proto_->wrist_goal(), wrist, 1e-3);

    EXPECT_NEAR(superstructure_status_proto_->elevator_height(), elevator,
                1e-3);
    EXPECT_NEAR(superstructure_status_proto_->wrist_angle(), wrist, 1e-3);
  }

  void CheckIntake(bool has_ground_hatch, bool has_hp_hatch, bool has_cargo,
                   bool arrow_solenoid, bool backplate_solenoid,
                   bool snap_down) const {
    EXPECT_EQ(superstructure_status_proto_->has_ground_hatch(),
              has_ground_hatch);
    EXPECT_EQ(superstructure_status_proto_->has_hp_hatch(), has_hp_hatch);
    EXPECT_EQ(superstructure_status_proto_->has_cargo(), has_cargo);

    EXPECT_EQ(superstructure_output_proto_->arrow_solenoid(), arrow_solenoid);
    EXPECT_EQ(superstructure_output_proto_->backplate_solenoid(),
              backplate_solenoid);
    EXPECT_EQ(superstructure_output_proto_->snap_down(), snap_down);
  }

 private:
  Superstructure superstructure_;
};

TEST_F(SuperstructureTest, CalibrateDisabled) { CalibrateDisabled(); }

/*TEST_F(SuperstructureTest, Disabled) {
  SetGoal(ScoreGoal::HATCH_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, false);
  SetInput(0, false, 0, false);

  Update();

  CheckIntake(false, false, false, false, false, false);
  EXPECT_EQ(superstructure_output_proto_->elevator_setpoint(), 0);
  EXPECT_EQ(superstructure_output_proto_->wrist_setpoint(), 0);
  EXPECT_EQ(superstructure_output_proto_->cargo_roller_voltage(), 0);
  EXPECT_EQ(superstructure_output_proto_->hatch_roller_voltage(), 0);
  EXPECT_EQ(superstructure_output_proto_->winch_voltage(), 0);
  EXPECT_EQ(superstructure_output_proto_->drop_forks(), false);
}
*/
// TODO(Hanson) is this test even necessary
/*TEST_F(SuperstructureTest, DisabledCalibrates) {
  CalibrateDisabled();
  RunFor(100);

  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), wrist_plant_.x(0),
              1e-2);
  EXPECT_NEAR(score_subsystem_status_proto_->elevator_height(),
              elevator_plant_.x(0), 1e-2);
}*/
/*
TEST_F(SuperstructureTest, ScoreGoals) {
  CalibrateDisabled();

  // CARGO_ROCKET_FIRST
  SetGoal(ScoreGoal::CARGO_ROCKET_FIRST, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoRocketFirstHeight, kCargoRocketFirstAngle);

  // CARGO_ROCKET_BACKWARDS
  SetGoal(ScoreGoal::CARGO_ROCKET_BACKWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoRocketBackwardsHeight, kCargoRocketBackwardsAngle);

  // CARGO_ROCKET_SECOND
  SetGoal(ScoreGoal::CARGO_ROCKET_SECOND, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoRocketSecondHeight, kCargoRocketSecondAngle);

  // CARGO_ROCKET_THIRD
  SetGoal(ScoreGoal::CARGO_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoRocketThirdHeight, kCargoRocketThirdAngle);

  // HATCH_ROCKET_FIRST
  SetGoal(ScoreGoal::HATCH_ROCKET_FIRST, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchRocketFirstHeight, kHatchForwardsAngle);

  // HATCH_ROCKET_BACKWARDS
  SetGoal(ScoreGoal::HATCH_ROCKET_BACKWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchRocketBackwardsHeight, kHatchBackwardsAngle);

  // HATCH_ROCKET_SECOND
  SetGoal(ScoreGoal::HATCH_ROCKET_SECOND, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchRocketSecondHeight, kHatchForwardsAngle);

  // HATCH_ROCKET_THIRD
  SetGoal(ScoreGoal::HATCH_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchRocketThirdHeight, kHatchForwardsAngle);

  // CARGO_SHIP_FORWARDS
  SetGoal(ScoreGoal::CARGO_SHIP_FORWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoShipForwardsHeight, kCargoShipForwardsAngle);

  // CARGO_SHIP_BACKWARDS
  SetGoal(ScoreGoal::CARGO_SHIP_BACKWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoShipBackwardsHeight, kCargoShipBackwardsAngle);

  // HATCH_SHIP_FORWARDS
  SetGoal(ScoreGoal::HATCH_SHIP_FORWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchShipForwardsHeight, kHatchForwardsAngle);

  // HATCH_SHIP_BACKWARDS
  SetGoal(ScoreGoal::HATCH_SHIP_BACKWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchShipBackwardsHeight, kHatchBackwardsAngle);

  // STOW
  SetGoal(ScoreGoal::STOW, IntakeGoal::INTAKE_NONE, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kStowHeight, kStowAngle);
} */

/*TEST_F(SuperstructureTest, TTF) {
  SetIntakeInputs(false, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_GROUND_HATCH, true);
  RunFor(1);
  SetIntakeInputs(true, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(50);
  CheckIntake(true, false, false, false, false, false);

  SetGoal(ScoreGoal::HANDOFF, IntakeGoal::POP, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(50);
  CheckGoal(kHandoffHeight, kHandoffAngle);

  SetIntakeInputs(true, true, false);
  RunFor(50);
  CheckIntake(true, true, false, true, true, false);

  SetGoal(ScoreGoal::HANDOFF, IntakeGoal::SPIT, true);
  RunFor(55);

  SetIntakeInputs(false, true, false);

  CheckIntake(false, true, false, true, true, false);
}*/
/*
TEST_F(SuperstructureTest, spitout) {
  SetIntakeInputs(false, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_GROUND_HATCH, true);
  RunFor(1);
  SetIntakeInputs(true, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(50);
  CheckIntake(true, false, false, false, false, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_GROUND_HATCH, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(50);
  CheckIntake(false, false, false, false, false, true);
}

TEST_F(SuperstructureTest, Handoff) {
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_GROUND_HATCH, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  CheckIntake(false, false, false, false, false, true);  // should snap down

  SetIntakeInputs(true, false, false);  // set ground hatch current to 5million
  RunFor(50);
  CheckIntake(true, false, false, false, false, false);  // has ground hatch

  SetGoal(ScoreGoal::HANDOFF, IntakeGoal::POP, true);
  RunFor(1);
  CheckGoal(kHandoffHeight, kHandoffAngle);

  SetIntakeInputs(true, true, false);  // arrow stabbed the hatch

  CheckIntake(true, true, false, false, false, false);
  SetGoal(ScoreGoal::HANDOFF, IntakeGoal::SPIT, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(50);

  EXPECT_EQ(superstructure_status_proto_->state(), HANDING_OFF);

  SetIntakeInputs(false, true, false);  // ground intake spits out
  RunFor(50);

  CheckIntake(false, true, false, true, true, false);
}*/

/*TEST_F(SuperstructureTest, IntakeGoals) {
  CalibrateDisabled();

  // INTAKE_HATCH
  SetIntakeInputs(false, false, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_HATCH, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchLoadingStationHeight, kHatchForwardsAngle);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_HATCH);

  SetIntakeInputs(false, true, false);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(false, true, false, true, true, false);

  // INTAKE_GROUND_HATCH
  SetIntakeInputs(false, false, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_GROUND_HATCH, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kHatchGroundHeight, kHatchForwardsAngle);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_GROUND_HATCH);

  SetIntakeInputs(true, false, false);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(true, false, false, false, false, true);

  // INTAKE_CARGO
  SetIntakeInputs(false, false, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_CARGO, true);
  RunFor(1);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  CheckGoal(kCargoGroundHeight, kCargoGroundAngle);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_CARGO);

  SetIntakeInputs(false, false, true);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(false, false, true, false, false, false);

  // OUTTAKE_HATCH
  SetIntakeInputs(false, true, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_HATCH, true);
  RunFor(1000);

  SetIntakeInputs(false, false, false);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(false, false, false, false, false, false);

  // OUTTAKE_GROUND_HATCH
  SetIntakeInputs(false, false, true);

  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_GROUND_HATCH, true);
  RunFor(1000);

  SetIntakeInputs(false, false, false);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(false, false, false, false, false, false);

  // OUTTAKE_CARGO
  SetIntakeInputs(true, false, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_CARGO, true);
  RunFor(1000);

  SetIntakeInputs(false, false, false);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(false, false, false, false, false, false);

  // POP
  SetIntakeInputs(true, false, false);

  SetGoal(ScoreGoal::NONE, IntakeGoal::POP, true);
  RunFor(1000);

  SetIntakeInputs(false, true, false);

  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  CheckIntake(false, true, false, false, false, false);
}*/

}  // namespace superstructure
}  // namespace c2019
