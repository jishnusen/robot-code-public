#include "muan/lime/lime.h"
#include <cmath>
#include "gtest/gtest.h"

namespace muan {
namespace lime {

// TESTS
TEST(LimeTest, CubeDistance) {
  muan::lime::Lime lime{0.92, 30, 0.28};
  double result = lime.ObjectDistance(18);

  EXPECT_NEAR(result, 0.71079200949068355, 1e-3);
}

}  // namespace lime
}  // namespace muan
