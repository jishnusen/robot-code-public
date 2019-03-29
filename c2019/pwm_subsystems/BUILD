cc_library(
    name = "subsystem_runner",
    srcs = ["subsystem_runner.cpp"],
    hdrs = ["subsystem_runner.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2019/pwm_subsystems/drivetrain:drivetrain_base",
        "//c2019/pwm_wpilib:wpilib_interface",
        "//muan/utils:threading_utils",
        "//muan/queues:queue_manager",
        "//third_party/aos/common:time",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_lib",
    ],
    restricted_to = ["//tools:roborio"],
)
