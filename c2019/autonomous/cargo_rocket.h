#ifndef C2019_AUTONOMOUS_CARGO_ROCKET_H_
#define C2019_AUTONOMOUS_CARGO_ROCKET_H_

#include "c2019/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"

namespace c2019 {
namespace autonomous {

class CargoRocket : public c2019::autonomous::AutonomousBase {
 public:
  void LeftSide();
};

}  // namespace autonomous
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_CARGO_ROCKET_H_
