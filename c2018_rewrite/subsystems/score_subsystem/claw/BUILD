package(default_visibility = ["//visibility:public"])

cc_library(
    name = "claw",
    srcs = ["claw.cpp"],
    hdrs = ["claw.h"],
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
    name = "claw_test",
    srcs = ["claw_test.cpp"],
    deps = [
        ":claw",
        "//third_party/google/googletest",
    ],
)
