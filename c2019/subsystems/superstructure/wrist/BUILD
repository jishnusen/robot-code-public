load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "wrist",
    srcs = ["wrist.cpp"],
    hdrs = ["wrist.h"],
    deps = [
        ":wrist_queues",
        "//muan/control/calibration:hall_calibration",
        "//muan/utils:monitor",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_library(
    name = "wrist_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":wrist_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
    ],
)

cc_proto_library(
    name = "wrist_proto",
    srcs = ["wrist.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)

cc_test(
     name = "wrist_test",
     srcs = ["wrist_test.cpp"],
     deps = [
         ":wrist",
         "//third_party/google/googletest",
     ],
)
