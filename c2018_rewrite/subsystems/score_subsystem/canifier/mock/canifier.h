#ifndef C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CANIFIER_MOCK_CANIFIER_H_
#define C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CANIFIER_MOCK_CANIFIER_H_

namespace c2018 {
namespace subsystems {
namespace canifier {

struct CanifierInput {
  bool elevator_hall_effect = false;

  bool wrist_hall_effect = false;
  bool cube_proxy = false;
};

class Canifier {
 public:
  static Canifier& GetInstance() {
    static Canifier instance;
    return instance;
  }

  void SetInput(CanifierInput input) { input_ = input; }

  inline CanifierInput input() { return input_; }
  void Update() { return; }

 private:
  Canifier() = default;

  CanifierInput input_;
};

}  // namespace canifier
}  // namespace subsystems
}  // namespace c2018

#endif  //  C2018_REWRITE_SUBSYSTEMS_SCORE_SUBSYSTEM_CANIFIER_MOCK_CANIFIER_H_
