package(default_visibility = ["//visibility:public"])

cc_library(
    name = "drivetrain",
    srcs = ["drivetrain.cpp"],
    hdrs = ["drivetrain.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//muan/control:pose",
        "//muan/control:trajectory",
        "//muan/utils:math_utils",
        "//o2018/subsystems:constants",
        "//third_party/eigen",
        "//third_party/phoenixsuite:phoenix",
    ],
)
