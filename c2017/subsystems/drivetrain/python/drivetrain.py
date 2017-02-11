#!/usr/bin/python

from third_party.frc971.control_loops.python import control_loop
from third_party.frc971.control_loops.python import controls
import numpy
import numpy.linalg
import sys
import argparse
from matplotlib import pylab

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

class CIM(control_loop.ControlLoop):
  def __init__(self):
    super(CIM, self).__init__("CIM")
    # Stall Torque in N m
    self.stall_torque = 2.42
    # Stall Current in Amps
    self.stall_current = 133
    # Free Speed in RPM
    self.free_speed = 4650.0
    # Free Current in Amps
    self.free_current = 2.7
    # Moment of inertia of the CIM in kg m^2
    self.J = 0.0001
    # Resistance of the motor, divided by 2 to account for the 2 motors
    self.resistance = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
              (12.0 - self.resistance * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Control loop time step
    self.dt = 0.005

    # State feedback matrices
    self.A_continuous = numpy.matrix(
        [[-self.Kt / self.Kv / (self.J * self.resistance)]])
    self.B_continuous = numpy.matrix(
        [[self.Kt / (self.J * self.resistance)]])
    self.C = numpy.matrix([[1]])
    self.D = numpy.matrix([[0]])

    self.A, self.B = self.ContinuousToDiscrete(self.A_continuous,
                                               self.B_continuous, self.dt)

    self.PlaceControllerPoles([0.01])
    self.PlaceObserverPoles([0.01])

    self.U_max = numpy.matrix([[12.0]])
    self.U_min = numpy.matrix([[-12.0]])

    self.InitializeState()

class Drivetrain(control_loop.ControlLoop):
  def __init__(self, name="Drivetrain", left_low=True, right_low=True):
    super(Drivetrain, self).__init__(name)
    # Number of motors per side
    self.num_motors = 2
    # Stall Torque in N m
    self.stall_torque = 2.42 * self.num_motors * 0.60
    # Stall Current in Amps
    self.stall_current = 133.0 * self.num_motors
    # Free Speed in RPM. Used number from last year.
    self.free_speed = 5500.0
    # Free Current in Amps
    self.free_current = 4.7 * self.num_motors
    # Moment of inertia of the drivetrain in kg m^2
    self.J = 0.35
    # Mass of the robot, in kg.
    self.m = 22
    # Radius of the robot, in meters (requires tuning by hand)
    self.rb = 0.46
    # Radius of the wheels, in meters.
    self.r = 0.041275
    # Resistance of the motor, divided by the number of motors.
    self.resistance = 12.0 / self.stall_current
    # Motor velocity constant
    self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
               (12.0 - self.resistance * self.free_current))
    # Torque constant
    self.Kt = self.stall_torque / self.stall_current
    # Gear ratios
    self.gear = 1 / 4.55
    # Control loop time step
    self.dt = 0.005

    # State feedback matrices
    # X will be of the format
    # [[positionl], [velocityl], [positionr], [velocityr]]
    self.A_c_anglin = numpy.matrix([
        [0., 1., 0., 0.],
        [0., -2.324, 0., 0.],
        [0., 0., 0., 1.],
        [0., 0., 0., -7.5],
    ])
    self.B_c_anglin = numpy.matrix([
        [0., 0.],
        [0.5, 0.5],
        [0., 0.],
        [-1.2, 1.2],
    ])
    self.C = numpy.matrix([[1, 0, 0, 0],
                           [0, 0, 1, 0]])
    self.D = numpy.matrix([[0, 0],
                           [0, 0]])

    af_to_lr = numpy.matrix([
        [1.0, 0.0, -self.rb, 0.0],
        [0.0, 1.0, 0.0, -self.rb],
        [1.0, 0.0, self.rb, 0.0],
        [0.0, 1.0, 0.0, self.rb],
    ])

    # We're modelling the system from empirical constants in the state-space of [forward, forward_velocity, angular,
    # angular_velocity]. Then, the af_to_lr matrix above converts all of the matrices from angular+forward to a
    # left+right state-space, which is what 971's code actually uses.
    self.A_continuous = af_to_lr * self.A_c_anglin * numpy.linalg.inv(af_to_lr)
    self.B_continuous = af_to_lr * self.B_c_anglin

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    q_pos = 0.1
    q_vel = 2.0

    self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0, 0.0, 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0)), 0.0, 0.0],
                           [0.0, 0.0, (1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, 0.0, 0.0, (1.0 / (q_vel ** 2.0))]])

    self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0)), 0.0],
                           [0.0, (1.0 / (12.0 ** 2.0))]])
    self.K = controls.dlqr(self.A, self.B, self.Q, self.R)

    glog.debug('DT q_pos %f q_vel %s %s', q_pos, q_vel, name)
    glog.debug(str(numpy.linalg.eig(self.A - self.B * self.K)[0]))
    glog.debug('K %s', repr(self.K))

    self.hlp = 0.3
    self.llp = 0.4
    self.PlaceObserverPoles([self.hlp, self.hlp, self.llp, self.llp])

    self.U_max = numpy.matrix([[12.0], [12.0]])
    self.U_min = numpy.matrix([[-12.0], [-12.0]])

    self.InitializeState()

    self.Qff = numpy.matrix(numpy.zeros((4, 4)))
    self.Qff[1, 1] = self.Qff[3,3] = 1.0
    self.Kff = controls.TwoStateFeedForwards(self.B, self.Qff)

class KFDrivetrain(Drivetrain):
  def __init__(self, name="KFDrivetrain", left_low=True, right_low=True):
    super(KFDrivetrain, self).__init__(name, left_low, right_low)

    self.unaugmented_A_continuous = self.A_continuous
    self.unaugmented_B_continuous = self.B_continuous

    # The states are
    # The practical voltage applied to the wheels is
    #   V_left = U_left + left_voltage_error
    #
    # [left position, left velocity, right position, right velocity,
    #  left voltage error, right voltage error, angular_error]
    #
    # The left and right positions are filtered encoder positions and are not
    # adjusted for heading error.
    # The turn velocity as computed by the left and right velocities is
    # adjusted by the gyro velocity.
    # The angular_error is the angular velocity error between the wheel speed
    # and the gyro speed.
    self.A_continuous = numpy.matrix(numpy.zeros((7, 7)))
    self.B_continuous = numpy.matrix(numpy.zeros((7, 2)))
    self.A_continuous[0:4,0:4] = self.unaugmented_A_continuous
    self.A_continuous[0:4,4:6] = self.unaugmented_B_continuous
    self.B_continuous[0:4,0:2] = self.unaugmented_B_continuous
    self.A_continuous[0,6] = 1
    self.A_continuous[2,6] = -1

    self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, self.dt)

    self.C = numpy.matrix([[1, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0],
                           [0, -0.5 / self.rb, 0, 0.5 / self.rb, 0, 0, 0]])

    self.D = numpy.matrix([[0, 0],
                           [0, 0],
                           [0, 0]])

    q_pos = 0.1
    q_vel = 2.0
    q_voltage = 1.0
    q_encoder_uncertainty = 2.00

    self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, (q_pos ** 2.0), 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, (q_vel ** 2.0), 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0), 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, (q_voltage ** 2.0), 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (q_encoder_uncertainty ** 2.0)]])

    r_pos =  0.0001
    r_gyro = 0.000001
    self.R = numpy.matrix([[(r_pos ** 2.0), 0.0, 0.0],
                           [0.0, (r_pos ** 2.0), 0.0],
                           [0.0, 0.0, (r_gyro ** 2.0)]])

    # Solving for kf gains.
    self.KalmanGain, self.Q_steady = controls.kalman(
        A=self.A, B=self.B, C=self.C, Q=self.Q, R=self.R)

    self.L = self.A * self.KalmanGain

    unaug_K = self.K

    # Implement a nice closed loop controller for use by the closed loop
    # controller.
    self.K = numpy.matrix(numpy.zeros((self.B.shape[1], self.A.shape[0])))
    self.K[0:2, 0:4] = unaug_K
    self.K[0, 4] = 1.0
    self.K[1, 5] = 1.0

    self.Qff = numpy.matrix(numpy.zeros((4, 4)))
    qff_pos = 0.005
    qff_vel = 1.00
    self.Qff[0, 0] = 1.0 / qff_pos ** 2.0
    self.Qff[1, 1] = 1.0 / qff_vel ** 2.0
    self.Qff[2, 2] = 1.0 / qff_pos ** 2.0
    self.Qff[3, 3] = 1.0 / qff_vel ** 2.0
    self.Kff = numpy.matrix(numpy.zeros((2, 7)))
    self.Kff[0:2, 0:4] = controls.TwoStateFeedForwards(self.B[0:4,:], self.Qff)

    self.InitializeState()


def main(argv):
  argv = FLAGS(argv)
  glog.init()

  # Simulate the response of the system to a step input.
  drivetrain = Drivetrain(left_low=False, right_low=False)
  simulated_left = []
  simulated_right = []
  for _ in xrange(100):
    drivetrain.Update(numpy.matrix([[12.0], [12.0]]))
    simulated_left.append(drivetrain.X[0, 0])
    simulated_right.append(drivetrain.X[2, 0])

  if FLAGS.plot:
    pylab.plot(range(100), simulated_left)
    pylab.plot(range(100), simulated_right)
    pylab.suptitle('Acceleration Test')
    pylab.show()

  # Simulate forwards motion.
  drivetrain = Drivetrain(left_low=False, right_low=False)
  close_loop_left = []
  close_loop_right = []
  left_power = []
  right_power = []
  R = numpy.matrix([[1.0], [0.0], [1.0], [0.0]])
  for _ in xrange(300):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])
    left_power.append(U[0, 0])
    right_power.append(U[1, 0])

  if FLAGS.plot:
    pylab.plot(range(300), close_loop_left, label='left position')
    pylab.plot(range(300), close_loop_right, label='right position')
    pylab.plot(range(300), left_power, label='left power')
    pylab.plot(range(300), right_power, label='right power')
    pylab.suptitle('Linear Move')
    pylab.legend()
    pylab.show()

  # Try turning in place
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[-1.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  if FLAGS.plot:
    pylab.plot(range(100), close_loop_left)
    pylab.plot(range(100), close_loop_right)
    pylab.suptitle('Angular Move')
    pylab.show()

  # Try turning just one side.
  drivetrain = Drivetrain()
  close_loop_left = []
  close_loop_right = []
  R = numpy.matrix([[0.0], [0.0], [1.0], [0.0]])
  for _ in xrange(100):
    U = numpy.clip(drivetrain.K * (R - drivetrain.X_hat),
                   drivetrain.U_min, drivetrain.U_max)
    drivetrain.UpdateObserver(U)
    drivetrain.Update(U)
    close_loop_left.append(drivetrain.X[0, 0])
    close_loop_right.append(drivetrain.X[2, 0])

  if FLAGS.plot:
    pylab.plot(range(100), close_loop_left)
    pylab.plot(range(100), close_loop_right)
    pylab.suptitle('Pivot')
    pylab.show()

  # Write the generated constants out to a file.
  drivetrain_low_low = Drivetrain(
      name="DrivetrainLowLow", left_low=True, right_low=True)
  drivetrain_low_high = Drivetrain(
      name="DrivetrainLowHigh", left_low=True, right_low=False)
  drivetrain_high_low = Drivetrain(
      name="DrivetrainHighLow", left_low=False, right_low=True)
  drivetrain_high_high = Drivetrain(
      name="DrivetrainHighHigh", left_low=False, right_low=False)

  kf_drivetrain_low_low = KFDrivetrain(
      name="KFDrivetrainLowLow", left_low=True, right_low=True)
  kf_drivetrain_low_high = KFDrivetrain(
      name="KFDrivetrainLowHigh", left_low=True, right_low=False)
  kf_drivetrain_high_low = KFDrivetrain(
      name="KFDrivetrainHighLow", left_low=False, right_low=True)
  kf_drivetrain_high_high = KFDrivetrain(
      name="KFDrivetrainHighHigh", left_low=False, right_low=False)

  if len(argv) != 5:
    print "Expected .h file name and .cc file name"
  else:
    namespaces = ['c2017', 'subsystems', 'drivetrain']
    dog_loop_writer = control_loop.ControlLoopWriter(
        "Drivetrain", [drivetrain_low_low, drivetrain_low_high,
                       drivetrain_high_low, drivetrain_high_high],
        namespaces = namespaces)
    dog_loop_writer.AddConstant(control_loop.Constant("kDt", "%f",
          drivetrain_low_low.dt))
    dog_loop_writer.AddConstant(control_loop.Constant("kStallTorque", "%f",
          drivetrain_low_low.stall_torque))
    dog_loop_writer.AddConstant(control_loop.Constant("kStallCurrent", "%f",
          drivetrain_low_low.stall_current))
    dog_loop_writer.AddConstant(control_loop.Constant("kFreeSpeedRPM", "%f",
          drivetrain_low_low.free_speed))
    dog_loop_writer.AddConstant(control_loop.Constant("kFreeCurrent", "%f",
          drivetrain_low_low.free_current))
    dog_loop_writer.AddConstant(control_loop.Constant("kJ", "%f",
          drivetrain_low_low.J))
    dog_loop_writer.AddConstant(control_loop.Constant("kMass", "%f",
          drivetrain_low_low.m))
    dog_loop_writer.AddConstant(control_loop.Constant("kRobotRadius", "%f",
          drivetrain_low_low.rb))
    dog_loop_writer.AddConstant(control_loop.Constant("kWheelRadius", "%f",
          drivetrain_low_low.r))
    dog_loop_writer.AddConstant(control_loop.Constant("kR", "%f",
          drivetrain_low_low.resistance))
    dog_loop_writer.AddConstant(control_loop.Constant("kV", "%f",
          drivetrain_low_low.Kv))
    dog_loop_writer.AddConstant(control_loop.Constant("kT", "%f",
          drivetrain_low_low.Kt))
    dog_loop_writer.AddConstant(control_loop.Constant("kGearRatio", "%f",
          drivetrain_low_low.gear))

    dog_loop_writer.Write(argv[1], argv[2])

    kf_loop_writer = control_loop.ControlLoopWriter(
        "KFDrivetrain", [kf_drivetrain_low_low, kf_drivetrain_low_high,
                         kf_drivetrain_high_low, kf_drivetrain_high_high],
        namespaces = namespaces)
    kf_loop_writer.Write(argv[3], argv[4])

if __name__ == '__main__':
  sys.exit(main(sys.argv))
