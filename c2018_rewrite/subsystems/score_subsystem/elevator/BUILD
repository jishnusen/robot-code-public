package(default_visibility = ["//visibility:public"])

cc_library(
    name = "elevator",
    srcs = ["elevator.cpp"],
    hdrs = ["elevator.h"],
    deps = [
        "//muan/control:motion_profiles",
        "//muan/control:state_space",
        "//muan/control/calibration:hall_calibration",
        "//muan/units",
        "//muan/utils:math_utils",
    ],
)
