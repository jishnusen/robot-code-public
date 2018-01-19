#include "elevator_controller.h"
#include "gtest/gtest.h"

class ElevatorControllerTest : public ::testing::Test {
 public:
  ElevatorControllerTest() {}

  void Update() {
    elevator_.Update(elevator_input_proto_, &elevator_output_proto_, &elevator_status_proto_,
                     outputs_enabled_);
  }

  void ReadMessages() {
    elevator_output_queue_.ReadLastMessage(&elevator_output_proto_);
    elevator_status_queue_.ReadLastMessage(&elevator_status_proto_);
  }

  void WriteMessages() {
    elevator_input_queue_->WriteMessage(elevator_input_proto_);
    elevator_goal_queue_->WriteMessage(elevator_goal_proto_);
  }

  void SetGoal() { elevator_.SetGoal(elevator_goal_proto_); }

  void SetInput(double position, bool hall) {
    elevator_input_proto_->set_elevator_encoder(position);
    elevator_input_proto_->set_elevator_hall(hall);
  }

  c2018::score_subsystem::ScoreSubsystemGoalQueue* elevator_goal_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemGoalProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemInputQueue* elevator_input_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader elevator_status_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemStatusProto>::Fetch()->MakeReader();

  c2018::score_subsystem::ScoreSubsystemOutputQueue::QueueReader elevator_output_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemOutputProto>::Fetch()->MakeReader();

  c2018::score_subsystem::ScoreSubsystemGoalProto elevator_goal_proto_;
  c2018::score_subsystem::ScoreSubsystemInputProto elevator_input_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto elevator_status_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto elevator_output_proto_;

  bool outputs_enabled_;

 private:
  c2018::score_subsystem::elevator::ElevatorController elevator_;
};

TEST_F(ElevatorControllerTest, NotEnabled) {
  elevator_goal_proto_->set_elevator_height(c2018::score_subsystem::HEIGHT_1);
  SetGoal();

  outputs_enabled_ = false;

  WriteMessages();
  Update();
  ReadMessages();

  EXPECT_EQ(elevator_status_proto_->elevator_actual_height(), 0.);
}

TEST_F(ElevatorControllerTest, Calibration) {
  elevator_goal_proto_->set_elevator_height(c2018::score_subsystem::HEIGHT_SCORE);
  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);

  SetGoal();
  WriteMessages();
  Update();

  for (int i = 0; i < 200; i++) {
    elevator_input_proto_->set_elevator_encoder(i);
    Update();
  }

  elevator_input_proto_->set_elevator_hall(true);
  Update();

  ReadMessages();

  EXPECT_TRUE(elevator_status_proto_->elevator_calibrated());
}
