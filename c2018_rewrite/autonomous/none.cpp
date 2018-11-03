#include "c2018_rewrite/autonomous/none.h"

namespace c2018 {
namespace autonomous {

void None::NoneAuto() {
  SetFieldPosition(1678.1678, 1678.1678, 0.0);
  LOG(INFO, "Running NONE auto");

  Wait(1678);  // :)
}

}  // namespace autonomous
}  // namespace c2018
