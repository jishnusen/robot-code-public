#include "third_party/frc971/control_loops/state_feedback_loop.h"

#include "gtest/gtest.h"

namespace testing {

// Tests that everything compiles and nothing crashes even if
// number_of_inputs!=number_of_outputs.
// There used to be lots of bugs in this area.
TEST(StateFeedbackLoopTest, UnequalSizes) {
  // In general, most (all?) errors will make these statements either not
  // compile or have assertion failures at runtime.
  const StateFeedbackPlantCoefficients<2, 4, 7> coefficients(
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 2>::Identity(),
      Eigen::Matrix<double, 2, 4>::Identity(),
      Eigen::Matrix<double, 7, 2>::Identity(),
      Eigen::Matrix<double, 7, 4>::Identity(),
      Eigen::Matrix<double, 4, 1>::Constant(1),
      Eigen::Matrix<double, 4, 1>::Constant(-1));

  // Build a plant.
  ::std::vector<::std::unique_ptr<StateFeedbackPlantCoefficients<2, 4, 7>>>
      v_plant;
  v_plant.emplace_back(
      new StateFeedbackPlantCoefficients<2, 4, 7>(coefficients));
  StateFeedbackPlant<2, 4, 7> plant(&v_plant);
  plant.Update(Eigen::Matrix<double, 4, 1>::Zero());
  plant.Reset();
  plant.CheckU(Eigen::Matrix<double, 4, 1>::Zero());

  // Now build a controller.
  ::std::vector<::std::unique_ptr<StateFeedbackControllerCoefficients<2, 4, 7>>>
      v_controller;
  v_controller.emplace_back(new StateFeedbackControllerCoefficients<2, 4, 7>(
      Eigen::Matrix<double, 4, 2>::Identity(),
      Eigen::Matrix<double, 4, 2>::Identity()));
  StateFeedbackController<2, 4, 7> controller(&v_controller);

  ::std::vector<::std::unique_ptr<StateFeedbackObserverCoefficients<2, 4, 7>>>
      v_observer;
  v_observer.emplace_back(new StateFeedbackObserverCoefficients<2, 4, 7>(
      Eigen::Matrix<double, 2, 7>::Identity()));
  StateFeedbackObserver<2, 4, 7> observer(&v_observer);

  StateFeedbackLoop<2, 4, 7> test_loop(
      ::std::move(plant), ::std::move(controller), ::std::move(observer));
  test_loop.Correct(Eigen::Matrix<double, 7, 1>::Identity());
  test_loop.Update(false);
  test_loop.CapU();
}

}  // namespace testing
