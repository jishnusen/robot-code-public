package(default_visibility = ["//visibility:public"])

load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
    name = "drivetrain_proto",
    srcs = ["drivetrain.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "drivetrain_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "drivetrain_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_library(
    name = "drivetrain",
    srcs = [
        "drivetrain.cpp",
        "open_loop_drive.cpp",
    ],
    hdrs = [
        "drivetrain.h",
        "drivetrain_config.h",
        "open_loop_drive.h",
    ],
    linkopts = ["-lm"],
    deps = [
        ":drivetrain_queues",
        "//muan/control:drivetrain_model",
        "//muan/queues:queue_manager",
        "//muan/wpilib:wpilib_queues",
    ],
)
