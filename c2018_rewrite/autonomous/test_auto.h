#ifndef C2018_REWRITE_AUTONOMOUS_TEST_AUTO_H_
#define C2018_REWRITE_AUTONOMOUS_TEST_AUTO_H_

#include "c2018_rewrite/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"
#include "muan/units/units.h"

namespace c2018 {
namespace autonomous {

class TestAuto : public c2018::autonomous::AutonomousBase {
 public:
  void Run();
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_REWRITE_AUTONOMOUS_TEST_AUTO_H_
