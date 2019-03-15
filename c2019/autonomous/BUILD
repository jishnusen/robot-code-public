cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    deps = [
        "//c2019/commands",
                "//muan/wpilib:wpilib_queues",

        "//muan/queues:queue_manager",
        "//muan/logging:logger",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "autonomous",
    deps = [
        ":autonomous_base",
        ":hatch_rocket_front",
        ":cargo_ship",
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

cc_library(
    name = "cargo_ship",
    srcs = ["cargo_ship.cpp"],
    hdrs = ["cargo_ship.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)
