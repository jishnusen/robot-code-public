cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    # restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
    ],
)

cc_library(
    name = "autonomous",
    deps = [
        ":autonomous_base",
        ":hatch_rocket_front",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "autonomous_base",
    srcs = ["autonomous_base.cpp"],
    hdrs = ["autonomous_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:wpilib_queues",
        "//c2019/subsystems/limelight:limelight_queues",
        "//c2019/subsystems/superstructure:superstructure_queues",
    ],
)

cc_test(
    name = "autonomous_base_test",
    srcs = ["autonomous_base_test.cpp"],
    linkopts = ["-lpthread"],
    deps = [
        ":autonomous_base",
        ":test_auto",
        "//third_party/google/googletest",
    ],
)

cc_library(
    name = "hatch_rocket_front",
    srcs = ["hatch_rocket_front.cpp"],
    hdrs = ["hatch_rocket_front.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)
