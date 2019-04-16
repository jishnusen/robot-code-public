#ifndef C2019_COMMANDS_ROCKET_H_
#define C2019_COMMANDS_ROCKET_H_

#include "muan/logging/logger.h"
#include "c2019/commands/command_base.h"

namespace c2019 {
namespace commands {

class Rocket : public c2019::commands::CommandBase {
 public:
  void LeftBackRocket();
  void LeftCargoRocket();
  void LeftDoubleRocket();
  void LeftFrontCargoShip();
  void LeftSideCargoShip();
  void RightBackRocket();
  void RightCargoRocket();
  void RightDoubleRocket();
  void RightFrontCargoShip();
  void RightSideCargoShip();
};

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_ROCKET_H_
