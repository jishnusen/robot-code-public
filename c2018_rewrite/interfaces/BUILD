package(default_visibility = ["//visibility:public"])

cc_library(
    name = "score_interface",
    srcs = ["score_interface.cpp"],
    hdrs = ["score_interface.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/queues:queue_manager",
        "//c2018_rewrite/subsystems/score_subsystem:score_subsystem_queues",
        "//muan/wpilib:can_wrapper",
        "//muan/phoenix:talon_wrapper",
        "//muan/phoenix:victor_wrapper",
    ],
)

cc_library(
    name = "drive_interface",
    srcs = ["drive_interface.cpp"],
    hdrs = ["drive_interface.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:can_wrapper",
        "//muan/phoenix:talon_wrapper",
        "//muan/phoenix:victor_wrapper",
    ],
)

cc_library(
    name = "interface_runner",
    srcs = ["interface_runner.cpp"],
    hdrs = ["interface_runner.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":score_interface",
        ":drive_interface",
    ],
)
