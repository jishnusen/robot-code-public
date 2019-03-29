cc_library(
    name = "wpilib_interface",
    srcs = [
        "drivetrain_interface.cpp",
        "wpilib_interface.cpp",
    ],
    hdrs = [
        "drivetrain_interface.h",
        "wpilib_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/queues:queue_manager",
        "//muan/wpilib:accelerometer",
        "//muan/wpilib/gyro:gyro",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_queues",
        "//third_party/wpilibsuite:wpilib",
    ],
    restricted_to = ["//tools:roborio"],
)
