#include "muan/utils/history.h"
#include "muan/utils/timer.h"
#include "muan/utils/timing_utils.h"
#include "gtest/gtest.h"

using muan::Timer;
using muan::History;
using muan::sleep_until;
using muan::sleep_for;
using muan::now;

TEST(TimeUtils, TimerPositive) {
  using namespace muan::units;
  Timer t;
  t.Start();
  for (int i = 0; i < 10000; i++) {
    t.Get();
  }
  EXPECT_GT(t.Get(), 0 * s);
}

TEST(TimeUtils, TimerReset) {
  using namespace muan::units;
  Timer t;
  t.Start();
  for (int i = 0; i < 10000; i++) {
    t.Get();
  }
  t.Reset();
  EXPECT_LT(t.Get(), 0.1 * s);
}

TEST(TimeUtils, TimerAndDelay) {
  using namespace muan::units;
  Timer t;
  t.Start();
  sleep_for(.2 * s);
  EXPECT_NEAR(convert(t.Get(), s), .2, .01);
}

TEST(TimeUtils, SleepUntil) {
  using namespace muan::units;
  Time start = now();
  sleep_until(start + .5 * s);
  EXPECT_NEAR(convert(now(), s), convert(start, s) + .5, .01);
}

TEST(History, WorksCorrectly) {
  using namespace muan::units;
  History<int, 200> hist(.01 * s);
  for (int i = 0; i < 100; i++) {
    hist.Update(i);
  }

  // We don't really care about off-by-one errors
  for (Time t = .01 * s; t < 1 * s; t += .01 * s) {  // NOLINT
    EXPECT_NEAR(hist.GoBack(t), 100 - static_cast<int>(convert(t, .01 * s)), 1);
  }
}