package(default_visibility = ["//visibility:public"])

cc_library(
    name = "elevator",
    srcs = ["elevator.cpp"],
    hdrs = ["elevator.h"],
    deps = [
        "//c2018_rewrite/subsystems/score_subsystem:score_subsystem_queues",
        "//muan/control:motion_profiles",
        "//muan/control/calibration:hall_calibration",
        "//muan/logging:logger",
        "//muan/units",
        "//muan/utils:math_utils",
    ],
)

cc_test(
    name = "elevator_test",
    srcs = ["elevator_test.cpp"],
    deps = [
        ":elevator",
        "//third_party/google/googletest",
    ],
)
