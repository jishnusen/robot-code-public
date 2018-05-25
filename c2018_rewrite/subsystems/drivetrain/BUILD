package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivetrain",
    srcs = ["drivetrain.cpp"],
    hdrs = ["drivetrain.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//c2018_rewrite/subsystems:constants",
        "//muan/control:pose",
        "//muan/control:trajectory",
        "//muan/utils:math_utils",
        "//third_party/eigen",
        "//third_party/phoenixsuite:phoenix",
    ],
)
