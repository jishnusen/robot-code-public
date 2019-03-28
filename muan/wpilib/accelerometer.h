#ifndef MUAN_WPILIB_ACCELEROMETER_H_
#define MUAN_WPILIB_ACCELEROMETER_H_

#include "WPILib.h"
#include "muan/wpilib/queue_types.h"

namespace muan {
namespace wpilib {

enum AccelerometerAxis {
  kXPositive,
  kYPositive,
  kZPositive,
  kXNegative,
  kYNegative,
  kZNegative,
};

class Accelerometer {
 public:
  explicit Accelerometer(AccelerometerQueue* accelerometer_queue,
                         AccelerometerAxis robot_x_positive,
                         AccelerometerAxis robot_y_positive,
                         AccelerometerAxis robot_z_positve);
  void Send();

 private:
  BuiltInAccelerometer accelerometer_;
  AccelerometerQueue* accelerometer_queue_;
  // How to transform the accelerometer values to robot direction
  // Accelerometer: +x is right, +y is forward, +z is up
  // where forward is towards the USB ports
  // Robot: +x is forward, +y is left, +z is up
  AccelerometerAxis robot_x_positive_;
  AccelerometerAxis robot_y_positive_;
  AccelerometerAxis robot_z_positive_;
};

}  // namespace wpilib
}  // namespace muan

#endif  // MUAN_WPILIB_ACCELEROMETER_H_
