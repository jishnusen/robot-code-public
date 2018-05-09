#include "o2018/subsystems/subsystem_spawner.h"

namespace o2018 {

void SpawnSubsystems() {
  drivetrain = Drive::GetInstance();
}

}
