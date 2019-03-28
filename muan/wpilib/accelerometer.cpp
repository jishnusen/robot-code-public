#include "muan/wpilib/accelerometer.h"

namespace muan {
namespace wpilib {

Accelerometer::Accelerometer(AccelerometerQueue* accelerometer_queue,
                             AccelerometerAxis robot_x_positive,
                             AccelerometerAxis robot_y_positive,
                             AccelerometerAxis robot_z_positive)
    : accelerometer_(::Accelerometer::Range::kRange_8G),
      accelerometer_queue_(accelerometer_queue),
      robot_x_positive_(robot_x_positive),
      robot_y_positive_(robot_y_positive),
      robot_z_positive_(robot_z_positive) {}

void Accelerometer::Send() {
  AccelerometerProto input;
  double x = 0, y = 0, z = 0;
  switch (robot_x_positive_) {
    case kXPositive:
      x = accelerometer_.GetX();
      break;
    case kYPositive:
      x = accelerometer_.GetY();
      break;
    case kZPositive:
      x = accelerometer_.GetZ();
      break;
    case kXNegative:
      x = -accelerometer_.GetX();
      break;
    case kYNegative:
      x = -accelerometer_.GetY();
      break;
    case kZNegative:
      x = -accelerometer_.GetZ();
      break;
  }
  switch (robot_y_positive_) {
    case kXPositive:
      y = accelerometer_.GetX();
      break;
    case kYPositive:
      y = accelerometer_.GetY();
      break;
    case kZPositive:
      y = accelerometer_.GetZ();
      break;
    case kXNegative:
      y = -accelerometer_.GetX();
      break;
    case kYNegative:
      y = -accelerometer_.GetY();
      break;
    case kZNegative:
      y = -accelerometer_.GetZ();
      break;
  }
  switch (robot_z_positive_) {
    case kXPositive:
      z = accelerometer_.GetX();
      break;
    case kYPositive:
      z = accelerometer_.GetY();
      break;
    case kZPositive:
      z = accelerometer_.GetZ();
      break;
    case kXNegative:
      z = -accelerometer_.GetX();
      break;
    case kYNegative:
      z = -accelerometer_.GetY();
      break;
    case kZNegative:
      z = -accelerometer_.GetZ();
      break;
  }
  input->set_x_acceleration(x * 9.81);
  input->set_y_acceleration(y * 9.81);
  input->set_z_acceleration(z * 9.81);
  accelerometer_queue_->WriteMessage(input);
}

}  // namespace wpilib

}  // namespace muan
