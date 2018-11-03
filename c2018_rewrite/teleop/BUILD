cc_library(
    name = "teleop",
    srcs = ["teleop.cpp"],
    hdrs = ["teleop.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//c2018_rewrite/subsystems/score_subsystem:score_subsystem_queues",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
        "//muan/teleop:joystick",
        "//muan/wpilib:ds_sender",
        "//muan/lime:lime_queues",
        "//muan/subsystems/drivetrain:drivetrain_queues",
    ],
    restricted_to = ["//tools:roborio"],
)
