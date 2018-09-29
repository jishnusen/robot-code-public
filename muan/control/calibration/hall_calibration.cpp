#include "muan/control/calibration/hall_calibration.h"

namespace muan {
namespace control {

HallCalibration::HallCalibration(double magnet_position)
    : max_hall_true_(0),
      min_hall_true_(0),
      max_overall_(0),
      min_overall_(0),
      first_time_(true),
      magnet_found_(false),
      calibrated_(false),
      offset_(0),
      magnet_position_(magnet_position) {}

double HallCalibration::Update(double main_sensor_value, bool hall_value) {
  if (hall_value) {
    // Update the max and min values for when the hall sensor is triggered. Set
    // them to the current value if it is the first time seeing the magnet.

    if (main_sensor_value > max_hall_true_ || !magnet_found_) {
      max_hall_true_ = main_sensor_value;
    }
    if (main_sensor_value < min_hall_true_ || !magnet_found_) {
      min_hall_true_ = main_sensor_value;
    }
    magnet_found_ = true;
  }
  // Update the max and min overall values. Set the to the current value if
  // it is the first time running Update()
  if (main_sensor_value > max_overall_ || first_time_) {
    max_overall_ = main_sensor_value;
  }
  if (main_sensor_value < min_overall_ || first_time_) {
    min_overall_ = main_sensor_value;
  }
  // Return the best estimate known. If the magnet is not found or the edges of
  // the magnet's range have not been reached, there is no best estimate. In
  // the event that calibrated_ is true, do not set it to false even if the
  // condition is not currently met, as that could reset any portions of code
  // that assume calibration is complete.

  if ((magnet_found_ && max_overall_ > max_hall_true_ &&
       min_overall_ < min_hall_true_) ||
      calibrated_) {
    // The center of the magnet's range is magnet_position_, so the offset if
    // magnet_position_ - the raw value of the center of the magnet
    offset_ = magnet_position_ - (max_hall_true_ + min_hall_true_) / 2;
    calibrated_ = true;
  }

  first_time_ = false;
  return main_sensor_value + offset_;
}

bool HallCalibration::is_calibrated() const { return calibrated_; }

double HallCalibration::offset() const { return offset_; }

}  // namespace control
}  // namespace muan
