cc_library(
    name = "autonomous_runner",
    srcs = ["autonomous_runner.cpp"],
    hdrs = ["autonomous_runner.h"],
    deps = [
        ":autonomous_base",
        ":test_auto",
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
    ],
    visibility = ["//visibility:public"],
    restricted_to = ["//tools:roborio"],
)

cc_library(
    name = "autonomous_base",
    srcs = ["autonomous_base.cpp"],
    hdrs = ["autonomous_base.h"],
    deps = [
        "//muan/queues:queue_manager",
        "//muan/logging:logger",
        "//muan/wpilib:wpilib_queues",
        "//muan/subsystems/drivetrain:drivetrain_queues",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "autonomous_base_test",
    srcs = ["autonomous_base_test.cpp"],
    deps = [
        ":autonomous_base",
        "//third_party/google/googletest:googletest",
    ],
)

cc_library(
    name = "test_auto",
    srcs = ["test_auto.cpp"],
    hdrs = ["test_auto.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":autonomous_base",
    ],
)
