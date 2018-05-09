#ifndef O2018_CITRUS_ROBOT_CITRUS_ROBOT_H_
#define O2018_CITRUS_ROBOT_CITRUS_ROBOT_H_

namespace o2018 {

namespace citrus_robot {

class CitrusRobot {
 public:
   CitrusRobot();
   void operator()();
   void UpdateTeleop();
   void UpdateAutonomous();

 private:
   subsystems::Drivetrain drivetrain_;
}

}

}


#endif  //  O2018_CITRUS_ROBOT_CITRUS_ROBOT_H_
