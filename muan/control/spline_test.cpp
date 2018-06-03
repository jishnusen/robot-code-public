#include "muan/control/spline.h"
#include "gtest/gtest.h"

namespace muan {
namespace control {

TEST(Spline, StartToFinish) {
  Pose initial_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 0, 0, M_PI / 2.).finished());
  Pose final_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 1, 1, M_PI).finished());
  HermiteSpline spline = HermiteSpline(initial_pose, final_pose);

  EXPECT_NEAR(spline.PointAt(0)(0), initial_pose.translational()(0), 1e-10);
  EXPECT_NEAR(spline.PointAt(0)(1), initial_pose.translational()(1), 1e-10);
  EXPECT_NEAR(spline.PointAt(1)(0), final_pose.translational()(0), 1e-10);
  EXPECT_NEAR(spline.PointAt(1)(1), final_pose.translational()(1), 1e-10);
  EXPECT_NEAR(spline.HeadingAt(0), initial_pose.heading(), 1e-10);
  EXPECT_NEAR(spline.HeadingAt(1), final_pose.heading(), 1e-10);
}

}  // namespace control
}  // namespace muan
