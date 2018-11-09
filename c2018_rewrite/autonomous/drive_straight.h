#ifndef C2018_REWRITE_AUTONOMOUS_DRIVE_STRAIGHT_H_
#define C2018_REWRITE_AUTONOMOUS_DRIVE_STRAIGHT_H_

#include "c2018_rewrite/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"

namespace c2018 {
namespace autonomous {

class DriveStraight : public c2018::autonomous::AutonomousBase {
 public:
  void Drive();
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_REWRITE_AUTONOMOUS_DRIVE_STRAIGHT_H_
