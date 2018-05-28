package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wrist",
    srcs = ["wrist.cpp"],
    hdrs = ["wrist.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//c2018_rewrite/subsystems/score_subsystem/carriage_canifier",
        "//muan/control:motion_profiles",
        "//muan/control/calibration:hall_calibration",
        "//muan/phoenix:talon_wrapper",
        "//muan/units",
        "//muan/utils:math_utils",
        "//third_party/phoenixsuite:phoenix",
    ],
)
