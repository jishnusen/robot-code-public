#include "c2018_rewrite/subsystems/score_subsystem/canifier/canifier.h"

namespace c2018 {
namespace subsystems {
namespace canifier {

Canifier::Canifier() {}

Canifier& Canifier::GetInstance() {
  static Canifier instance = Canifier();
  return instance;
}

void Canifier::ReadInputs() {
  input_.elevator_hall_effect =
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CLK_PWM0P);
  input_.wrist_hall_effect =
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MOSI_PWM1P);
  input_.cube_proxy =
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MISO_PWM2P);
}

void Canifier::Update() { ReadInputs(); }

}  // namespace canifier
}  // namespace subsystems
}  // namespace c2018
