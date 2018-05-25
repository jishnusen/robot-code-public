package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wrist",
    srcs = ["wrist.cpp"],
    hdrs = ["wrist.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//c2018_rewrite/subsystems:constants",
        "//muan/control:motion_profiles",
        "//muan/control/calibration:hall_calibration",
        "//muan/units",
        "//muan/utils:math_utils",
        "//third_party/phoenixsuite:phoenix",
    ],
)
