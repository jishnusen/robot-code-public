package(default_visibility = ["//visibility:public"])

cc_library(
    name = "differential_drive",
    srcs = [
        "differential_drive.cpp",
    ],
    hdrs = [
        "differential_drive.h",
    ],
    linkopts = ["-lm"],
    linkstatic = 1,
    deps = [
        ":dc_motor",
        "//muan/utils:math_utils",
    ],
)

cc_library(
    name = "dc_motor",
    srcs = [
        "dc_motor.cpp",
    ],
    hdrs = [
        "dc_motor.h",
    ],
    linkopts = ["-lm"],
    linkstatic = 1,
)
