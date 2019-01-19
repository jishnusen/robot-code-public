#include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "gtest/gtest.h"

namespace c2019 {
namespace subsystems {
namespace wrist {

class WristTest : public ::testing::Test {
 public:
  void Update() {
    wrist_input_proto_->set_intake_proxy(has_cube_);
    wrist_input_proto_->set_wrist_voltage(voltage_);
    if (wrist_output_proto_->wrist_output_type() == POSITION) {
      wrist_input_proto_->set_wrist_encoder(wrist_output_proto_->wrist_setpoint() +
                                        offset_);
      wrist_input_proto_->set_wrist_velocity(
          (wrist_output_proto_->wrist_setpoint() - prev_position_) / 0.01);
      prev_position_ = wrist_input_proto_->wrist_encoder();
    }
    wrist_input_proto_->set_wrist_hall(
        std::abs(wrist_status_proto_->wrist_angle()) < 2e-2);
    WriteMessages();
    wrist_.Update();
    ReadMessages();
  }

  bool NotBroken() {
    return (wrist_status_proto_->wrist_angle() > kMinAngle - 1e-6 &&
            wrist_status_proto_->wrist_angle() < kMaxAngle + 1e-6);
  }

  void CheckGoal() {
    EXPECT_NEAR(wrist_status_proto_->wrist_angle(),
                wrist_status_proto_->wrist_profiled_goal(), 1e-9);
    EXPECT_NEAR(wrist_status_proto_->wrist_angle(),
                wrist_status_proto_->wrist_unprofiled_goal(), 1e-9);
    switch (intake_goal_) {
      case INTAKE_NONE:
        EXPECT_TRUE(wrist_output_proto_->intake_close());
        EXPECT_FALSE(wrist_output_proto_->intake_open());
        if (wrist_status_proto_->has_cube()) {
          EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kHoldingVoltage,
                      1e-9);
        } else {
          EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0., 1e-9);
        }
        break;
      case INTAKE:
        EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case INTAKE_OPEN:
        EXPECT_TRUE(wrist_output_proto_->intake_open());
        EXPECT_FALSE(wrist_output_proto_->intake_close());
        EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case INTAKE_CLOSE:
      case SETTLE:
        EXPECT_TRUE(wrist_output_proto_->intake_close());
        EXPECT_FALSE(wrist_output_proto_->intake_open());
        EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case OUTTAKE_SLOW:
        EXPECT_FALSE(wrist_output_proto_->intake_open());
        EXPECT_TRUE(wrist_output_proto_->intake_close());
        EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kSlowOuttakeVoltage,
                    1e-9);
        break;
      case OUTTAKE_FAST:
        EXPECT_FALSE(wrist_output_proto_->intake_open());
        EXPECT_TRUE(wrist_output_proto_->intake_close());
        EXPECT_NEAR(wrist_output_proto_->intake_voltage(), kFastOuttakeVoltage,
                    1e-9);
        break;
      case DROP:
        EXPECT_TRUE(wrist_output_proto_->intake_open());
        EXPECT_FALSE(wrist_output_proto_->intake_close());
        EXPECT_NEAR(wrist_output_proto_->intake_voltage(), 0., 1e-9);
        break;
    }
  }

  void WriteMessages() {
    ds_proto_->set_is_sys_active(outputs_enabled_);
    wrist_input_queue_->WriteMessage(wrist_input_proto_);
    wrist_goal_queue_->WriteMessage(wrist_goal_proto_);
    ds_queue_->WriteMessage(ds_proto_);
  }

  void ReadMessages() {
    wrist_status_reader_.ReadLastMessage(&wrist_status_proto_);
    wrist_output_reader_.ReadLastMessage(&wrist_output_proto_);
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
    wrist_goal_proto_->set_wrist_angle(goal);
    wrist_goal_proto_->set_intake_mode(intake_goal);
  }

  void SetInput(double position, bool hall) {
    wrist_input_proto_->set_wrist_encoder(position);
    wrist_input_proto_->set_wrist_hall(hall);
  }

  void CalibrateDisabled(double offset = 0) {
    offset_ = offset;
    outputs_enabled_ = false;
    calibrate_test_ = true;

    wrist_input_proto_->set_wrist_encoder(offset);
    wrist_input_proto_->set_zeroed(true);
    Update();

    EXPECT_TRUE(wrist_status_proto_->wrist_calibrated());
    EXPECT_NEAR(wrist_status_proto_->wrist_angle(), offset, 1e-3);
  }

  WristInputProto wrist_input_proto_;
  WristStatusProto wrist_status_proto_;
  WristOutputProto wrist_output_proto_;
  WristGoalProto wrist_goal_proto_;
  muan::wpilib::DriverStationProto ds_proto_;

  bool outputs_enabled_;
  bool has_cube_ = false;
  double voltage_ = kEncoderFaultMinVoltage - 0.1;

 protected:
  Wrist wrist_;

 private:
  WristStatusQueue::QueueReader wrist_status_reader_{
      muan::queues::QueueManager<WristStatusProto>::Fetch()->MakeReader()};
  WristOutputQueue::QueueReader wrist_output_reader_{
      muan::queues::QueueManager<WristOutputProto>::Fetch()->MakeReader()};
  WristInputQueue* wrist_input_queue_{
      muan::queues::QueueManager<WristInputProto>::Fetch()};
  WristGoalQueue* wrist_goal_queue_{
      muan::queues::QueueManager<WristGoalProto>::Fetch()};
  muan::wpilib::DriverStationQueue* ds_queue_{
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch()};

  double prev_position_;
  double offset_;

  bool calibrate_test_;
  IntakeMode intake_goal_;
};

TEST_F(WristTest, NotEnabled) {
  SetGoal(1);

  outputs_enabled_ = false;

  Update();

  EXPECT_EQ(wrist_status_proto_->wrist_angle(), 0.);
  EXPECT_EQ(wrist_output_proto_->wrist_output_type(), OPEN_LOOP);
  EXPECT_NEAR(wrist_output_proto_->wrist_setpoint(), 0., 1e-3);
}

TEST_F(WristTest, CalibrateDisabled) { CalibrateDisabled(1.); }

TEST_F(WristTest, AllAngles) {
  CalibrateDisabled();

  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
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

TEST_F(WristTest, Cap) {
  CalibrateDisabled();

  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
  outputs_enabled_ = true;

  SetGoal(4000);
  RunFor(1000);
  CheckGoal();

  SetGoal(-4000);
  RunFor(1000);
  CheckGoal();
}

TEST_F(WristTest, IntakeModes) {
  wrist_input_proto_->set_wrist_encoder(0);
  wrist_input_proto_->set_wrist_hall(false);
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

  SetGoal(0.0, INTAKE_NONE);
  has_cube_ = true;

  RunFor(600);
  EXPECT_TRUE(wrist_status_proto_->has_cube());
  CheckGoal();
}

}  // namespace wrist
}  // namespace subsystems
}  // namespace c2019
