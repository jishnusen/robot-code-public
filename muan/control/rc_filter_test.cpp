#include "gtest/gtest.h"
#include <cmath>
#include "muan/control/rc_filter.h"

TEST(RcFilterTest, SeededValue) {
  // Seed it with an output of 1.0, so it shouldn't change
  muan::control::RcFilter filter(1.0, 0.005, 1.0);
  for (int i = 0; i < 100; i++) {
    EXPECT_NEAR(filter.Update(1.0), 1.0, 1e-6);
  }
}

TEST(RcFilterTest, StepInput) {
  // Seed it with 0.0, but then give it 1.0 for 5s and make sure it moves to
  // match
  muan::control::RcFilter filter(0.1, 0.005, 0.0);
  for (int i = 0; i < 1000; i++) {
    EXPECT_LT(filter.Update(1.0), 1.0);
  }
  EXPECT_NEAR(filter.Update(1.0), 1.0, 1e-6);
}

TEST(RcFilterTest, RampInput) {
  // Seed it with 0.0, but then give it a ramp input and make sure it lags by a constant amount in steady-state
  muan::control::RcFilter filter(0.5, 0.005, 0.0);
  for (int i = 0; i < 1000; i++) {
    filter.Update(i * 0.01);
  }
  double offset = 1000 * 0.01 - filter.Update(1000 * 0.01);
  for (int i = 1001; i < 2000; i++) {
    EXPECT_NEAR(i * 0.01 - filter.Update(i * 0.01), offset, 1e-3);
  }
}
