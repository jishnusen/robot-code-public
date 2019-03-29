#include "c2019/pwm_wpilib/wpilib_interface.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace wpilib {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::PdpMessage;
using muan::wpilib::gyro::GyroMessageProto;

DEFINE_int32(gyro_time, 10, "How long to calibrate the gyro for.");

WpilibInterface::WpilibInterface()
    : gyro_{QueueManager<GyroMessageProto>::Fetch(),
            QueueManager<DriverStationProto>::Fetch(), FLAGS_gyro_time, false} {
  std::thread gyro_thread(std::ref(gyro_));
  gyro_thread.detach();
}

void WpilibInterface::WriteActuators() { drivetrain_.WriteActuators(); }

void WpilibInterface::ReadSensors() { drivetrain_.ReadSensors(); }

}  // namespace wpilib
}  // namespace c2019
