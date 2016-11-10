set -e
bazel test --test_output=errors //muan/... -- -//muan/wpilib/... -//muan/teleop/...
bazel build //muan/... --cpu=roborio
bazel test --test_output=errors //o2016/... -- -//o2016:frc1678 -//o2016/wpilib/... -//o2016/subsystem_runner/... -//o2016/teleop/...
bazel build //o2016/... --cpu=roborio
#./check-format.py
