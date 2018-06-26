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
