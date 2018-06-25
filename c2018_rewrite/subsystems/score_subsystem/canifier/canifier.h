#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CANIFIER_CANIFIER_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CANIFIER_CANIFIER_H_

#include "ctre/Phoenix.h"

namespace c2018 {
namespace subsystems {
namespace canifier {

constexpr int kId = 20;

struct CanifierInput {
  bool elevator_hall_effect = false;

  bool wrist_hall_effect = false;
  bool cube_proxy = false;
};

class Canifier {
 public:
  static Canifier& GetInstance();
  inline CanifierInput input() { return input_; }
  void Update();

 private:
  Canifier();
  void ReadInputs();

  CANifier canifier_{kId};

  CanifierInput input_;
};

}  // namespace canifier
}  // namespace subsystems
}  // namespace c2018

#endif  // C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CANIFIER_CANIFIER_H_
