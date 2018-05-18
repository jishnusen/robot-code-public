#include "muan/control/pose.h"
#include "gtest/gtest.h"

namespace muan {
namespace control {

TEST(Pose, HeadingWrapping) {
  Pose a = (Eigen::Vector3d() << 1.0, 2.0, 0.0).finished();
  Pose b = a.RotateBy(M_PI);
  EXPECT_NEAR(b.translational()(0), -1.0, 1e-6);
  EXPECT_NEAR(b.translational()(1), -2.0, 1e-6);

  // Either the heading should be -pi or pi, it doesn't matter
  EXPECT_NEAR(::std::abs(b.heading()), M_PI, 1e-6);

  Pose c = (Eigen::Vector3d() << 0.0, 0.0, -1.0).finished();
  EXPECT_NEAR((b + c).heading(), M_PI - 1.0, 1e-6);
}

TEST(Pose, Compose) {
  Pose a = (Eigen::Vector3d() << 2.0, 0.0, M_PI / 4.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 2.0, M_PI).finished();
  Pose c = a.Compose(b);

  EXPECT_NEAR(c.translational()(0), 2.0 - ::std::sqrt(2.0) / 2.0, 1e-6);
  EXPECT_NEAR(c.translational()(1), 3.0 * ::std::sqrt(2.0) / 2.0, 1e-6);
  EXPECT_NEAR(c.heading(), -3.0 * M_PI / 4.0, 1e-6);
}

TEST(TimedPose, Interpolate) {
  TimedPose<Pose> a = TimedPose<Pose>(
      Pose((Eigen::Vector3d() << 0., 0., 0.).finished()), 0., 0., 1.);
  TimedPose<Pose> b = TimedPose<Pose>(
      Pose((Eigen::Vector3d() << 0.5, 0., 0.).finished()), 1., 1., 0.);

  EXPECT_TRUE(a.pose().Get().isApprox(a.Interpolate(b, 0.).pose().Get(), 1e-9));
  EXPECT_TRUE(b.pose().Get().isApprox(a.Interpolate(b, 1.).pose().Get(), 1e-9));
  EXPECT_TRUE(b.pose().Get().isApprox(b.Interpolate(a, 0.).pose().Get(), 1e-9));
  EXPECT_TRUE(a.pose().Get().isApprox(b.Interpolate(a, 1.).pose().Get(), 1e-9));

  TimedPose<Pose> c = a.Interpolate(b, 0.5);

  EXPECT_NEAR(0.5, c.t(), 1e-9);
  EXPECT_NEAR(a.acceleration(), c.acceleration(), 1e-9);
  EXPECT_NEAR(0.5, c.velocity(), 1e-9);
  EXPECT_NEAR(0.125, c.pose().translational()(0), 1e-9);
}

}  // namespace control
}  // namespace muan
