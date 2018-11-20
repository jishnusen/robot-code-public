#include "muan/lime/lime.h"
#include <cmath>
#include "gtest/gtest.h"

namespace muan {
namespace lime {

// TESTS
TEST(LimeTest, CubeDistance) {
  muan::lime::Lime lime{1.0, 30,
                        0.5};  // Limelight is 1 m off ground, at a 30 degree
                               // angle aimed at an object that is 0.5 m tall
  double result = lime.ObjectDistance(
      20);  // Object is at a 20 degree angle with respect to the limelight

  double expected =
      (1.0 - 0.5) *
      (std::tan(
          (30 + 20) *
          (M_PI / 180.)));  // Simple trig, one leg of triangle is the offset
                            // from the limelight height to the object height,
                            // and the angle is the object angle with respect to
                            // the limelight + the limelight offset angle

  EXPECT_NEAR(result, expected, 1e-3);
}

}  // namespace lime
}  // namespace muan
