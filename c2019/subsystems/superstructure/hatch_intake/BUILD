load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "hatch_intake",
    srcs = ["hatch_intake.cpp"],
    hdrs = ["hatch_intake.h"],
    visibility = ["//visibility:public"],
    deps = [
        "hatch_intake_queues",
        "hatch_intake_proto",
        "//muan/queues:queue_manager",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_library(
    name = "hatch_intake_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        "hatch_intake_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "hatch_intake_proto",
    srcs = ["hatch_intake.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)
