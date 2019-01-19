#include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "gtest/gtest.h"

namespace c2019 {
namespace wrist {

class WristTest : public ::testing::Test {
 public:
  void Update() {
    wrist_input_proto_->set_wrist_voltage(voltage_);
    if (wrist_output_proto_->wrist_output_type() == POSITION) {
      wrist_input_proto_->set_wrist_encoder(wrist_output_proto_->wrist_setpoint() +
                                        offset_);
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
                wrist_status_proto_->wrist_goal(), 1e-9);
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
  bool has_cargo_ = false;
  bool has_panel_ = false;
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
};

/****************************************************************************/

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

}  // namespace wrist
}  // namespace c2019
