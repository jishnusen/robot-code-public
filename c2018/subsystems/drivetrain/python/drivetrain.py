#!/usr/bin/python

from third_party.frc971.control_loops.python import drivetrain
from third_party.frc971.control_loops.python import control_loop
import sys

import gflags
import glog

FLAGS = gflags.FLAGS 
gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

mass = 63.0
distribution_radius = 0.42

kDrivetrain = drivetrain.DrivetrainParams(J = mass * distribution_radius ** 2.0,
                                          mass = mass,
                                          robot_radius = 0.36,
                                          wheel_radius = 4.0 * 0.0254 / 2.0,
                                          G_high = (1./6.),
                                          G_low = (1./6.),
                                          motor_type = control_loop.CIM(),
                                          num_motors = 3,
                                          q_pos_low = 0.135,
                                          q_pos_high = 0.135,
                                          q_vel_low = 1.0,
                                          q_vel_high = 1.0,
                                          efficiency_high = 0.8,
                                          efficiency_low = 0.8,
                                          )

def main(argv):
  argv = FLAGS(argv)
  glog.init()

  if FLAGS.plot:
    drivetrain.PlotDrivetrainMotions(kDrivetrain)
  elif len(argv) != 5:
    print("Expected .h file name and .cc file name")
  else:
    # Write the generated constants out to a file.
    drivetrain.WriteDrivetrain(argv[1:3], argv[3:5], 'c2018', kDrivetrain)

if __name__ == '__main__':
  sys.exit(main(sys.argv))
