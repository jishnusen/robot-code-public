#include "muan/utils/spline_utils.h"
#include "gtest/gtest.h"

constexpr double kMaxDx = 0.05;
constexpr double kMaxDy = 0.005;
constexpr double kMaxDTheta = 5. * 2;

namespace muan {
namespace utils {

TEST(SplineUtils, OneSpline) {
  Pose initial_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 0, 0, M_PI / 2.).finished());
  Pose final_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 1, 1, M_PI).finished());
  HermiteSpline spline = HermiteSpline(initial_pose, final_pose);

  std::vector<PoseWithCurvature> samples =
      ParametrizeSpline(spline, kMaxDx, kMaxDy, kMaxDTheta);
  PoseWithCurvature previous = samples.front();

  EXPECT_NEAR(samples.front().pose().Get()(0), spline.PointAt(0)(0), 5e-3);
  EXPECT_NEAR(samples.front().pose().Get()(1), spline.PointAt(0)(1), 5e-3);
  EXPECT_NEAR(samples.back().pose().Get()(0), spline.PointAt(1)(0), 5e-3);
  EXPECT_NEAR(samples.back().pose().Get()(1), spline.PointAt(1)(1), 5e-3);

  samples.erase(samples.begin());
  for (PoseWithCurvature sample : samples) {
    Eigen::Vector3d delta = (sample.pose() - previous.pose()).Get();
    EXPECT_LT(::std::abs(delta(0)), kMaxDx);
    EXPECT_LT(::std::abs(delta(1)), kMaxDy);
    EXPECT_LT(delta(2), kMaxDTheta);
    previous = sample;
  }
}

TEST(SplineUtils, TwoSplines) {
  Pose initial_pose_a =
      Pose((Eigen::Matrix<double, 3, 1>() << 0, 0, M_PI / 2.).finished());
  Pose final_pose_a =
      Pose((Eigen::Matrix<double, 3, 1>() << 1, 1, M_PI).finished());
  HermiteSpline spline_a = HermiteSpline(initial_pose_a, final_pose_a);
  Pose initial_pose_b = final_pose_a;
  Pose final_pose_b =
      Pose((Eigen::Matrix<double, 3, 1>() << 3, 2, M_PI / 2.).finished());
  HermiteSpline spline_b = HermiteSpline(initial_pose_b, final_pose_b);

  std::vector<HermiteSpline> splines = {spline_a, spline_b};
  std::vector<PoseWithCurvature> samples =
      ParametrizeSplines(splines, kMaxDx, kMaxDy, kMaxDTheta);
  PoseWithCurvature previous = samples.front();

  EXPECT_NEAR(samples.front().pose().Get()(0), spline_a.PointAt(0)(0), 5e-3);
  EXPECT_NEAR(samples.front().pose().Get()(1), spline_a.PointAt(0)(1), 5e-3);
  EXPECT_NEAR(samples.back().pose().Get()(0), spline_b.PointAt(1)(0), 5e-3);
  EXPECT_NEAR(samples.back().pose().Get()(1), spline_b.PointAt(1)(1), 5e-3);

  samples.erase(samples.begin());
  for (PoseWithCurvature sample : samples) {
    Eigen::Vector3d delta = (sample.pose() - previous.pose()).Get();
    EXPECT_LT(delta(0), kMaxDx);
    EXPECT_LT(delta(1), kMaxDy * 1.5);  // Some leniency because of the size of
                                        // the constraint, and the gap between
                                        // splines
    EXPECT_LT(delta(2), kMaxDTheta);
    previous = sample;
  }
}

}  // namespace utils
}  // namespace muan
