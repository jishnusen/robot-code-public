#!/usr/bin/python

from third_party.frc971.control_loops.python import drivetrain
from third_party.frc971.control_loops.python import cim
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

#TODO(Neil): Update robot moment of inertia, mass, and robot radius
kDrivetrain = drivetrain.DrivetrainParams(J = 6.0,
                                          mass = 55,
                                          robot_radius = 0.59055 / 2.0,
                                          wheel_radius = 4 * 0.0254 / 2,
                                          G_high = (12.0 / 50.0) * (24.0 / 40.0) * (50.0 / 34.0),
                                          G_low = (12.0 / 50.0) * (24.0 / 40.0) * (34.0 / 50.0),
                                          motor_type = cim.MiniCIM(),
                                          num_motors = 3,
                                          q_pos_low = 0.12,
                                          q_pos_high = 0.14,
                                          q_vel_low = 1.0,
                                          q_vel_high = 0.95,
                                          )

def main(argv):
  argv = FLAGS(argv)
  glog.init()

  if FLAGS.plot:
    drivetrain.PlotDrivetrainMotions(kDrivetrain)
  elif len(argv) != 5:
    print "Expected .h file name and .cc file name"
  else:
    # Write the generated constants out to a file.
    drivetrain.WriteDrivetrain(argv[1:3], argv[3:5], 'c2018', kDrivetrain)

if __name__ == '__main__':
  sys.exit(main(sys.argv))
