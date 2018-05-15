#include "muan/control/trajectory.h"
#include "gtest/gtest.h"
#include "muan/control/spline.h"

#include <fstream>

namespace muan {
namespace control {

TEST(Trajectory, Interpolating) {
  std::ofstream csvout;
  csvout.open("/tmp/traj.csv");
  Pose initial_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 0, 0, M_PI / 2.).finished());
  Pose final_pose =
      Pose((Eigen::Matrix<double, 3, 1>() << 1, 1, M_PI).finished());
  HermiteSpline spline = HermiteSpline(initial_pose, final_pose);

  SplineGenerator generator;
  std::vector<PoseWithCurvature> parametrized_spline = generator.ParametrizeSpline(spline);
  Trajectory<PoseWithCurvature> trajectory =
      Trajectory<PoseWithCurvature>(parametrized_spline);
  for (double i = 0; i < trajectory.length() - 1; i += 0.1) {
    PoseWithCurvature state = trajectory.Interpolate(i).state;
    int index = static_cast<int>(floor(i));
    EXPECT_NEAR(state.pose().translational()(0), parametrized_spline.at(index).Interpolate(parametrized_spline.at(index+1), i - index).pose().translational()(0), 1e-2);
    EXPECT_NEAR(state.pose().translational()(1), parametrized_spline.at(index).Interpolate(parametrized_spline.at(index+1), i - index).pose().translational()(1), 1e-2);
    EXPECT_NEAR(state.pose().heading(), parametrized_spline.at(index).Interpolate(parametrized_spline.at(index+1), i - index).pose().heading(), 1e-2);
  }
}

}  // namespace control
}  // namespace muan
