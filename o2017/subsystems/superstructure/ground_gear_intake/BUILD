load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "ground_gear_intake",
    srcs = ["ground_gear_intake.cpp"],
    hdrs = ["ground_gear_intake.h"],
    visibility = ["//visibility:public"],
    deps = [
        "ground_gear_intake_queues",
        "//muan/wpilib:wpilib_queues",
        "//o2017/queue_manager:queue_manager",
    ],
)

cc_library(
    name = "ground_gear_intake_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "ground_gear_intake_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "ground_gear_intake_proto",
    srcs = ["ground_gear_intake.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)

cc_test(
    name = "ground_gear_intake_test",
    srcs = ["ground_gear_intake_test.cpp"],
    deps = [
        ":ground_gear_intake",
        ":ground_gear_intake_queues",
        "//third_party/google/googletest",
    ],
)
