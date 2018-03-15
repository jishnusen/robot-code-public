#ifndef C2018_AUTONOMOUS_SCALE_ONLY_H_
#define C2018_AUTONOMOUS_SCALE_ONLY_H_

#include "c2018/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"

namespace c2018 {
namespace autonomous {

class ScaleOnly : public c2018::autonomous::AutonomousBase {
 public:
  void RightScale();
  void LeftScale();
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_SCALE_ONLY_H_
