#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CARRIAGE_CANIFIER_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CARRIAGE_CANIFIER_H_

#include "ctre/Phoenix.h"

namespace c2018 {
namespace subsystems {
namespace carriage_canifier {

constexpr int kId = 10;

struct CarriageCanifierInput {
  bool elevator_hall_effect;

  bool wrist_hall_effect;
  bool cube_proxy;
};

class CarriageCanifier {
 public:
  static CarriageCanifier& GetInstance();
  inline CarriageCanifierInput input() { return input_; }

 private:
  CarriageCanifier();
  void Update();
  void ReadInputs();

  CANifier* canifier_ = new CANifier(kId);

  CarriageCanifierInput input_;

  double elevator_hall_pwm_[];
  double wrist_hall_pwm_[];
  double cube_proxy_pwm_[];
};

}  // namespace carriage_canifier
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CARRIAGE_CANIFIER_H_
