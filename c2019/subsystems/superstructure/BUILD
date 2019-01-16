load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "superstructure_proto",
    srcs = ["superstructure.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "superstructure",
    srcs = ["superstructure.cpp"],
    hdrs = ["superstructure.h"],
    deps = [:superstructure_queues,
    "elevator:elevator_queues",
    "wrist:wrist_queues",
    "ground_hatch_intake:ground_hatch_intake_queues",
    "hatch_intake:hatch_intake_queues",
    "cargo_intake:cargo_intake_queues",
    "winch:winch_queues",
    "//muan/queues:queue_manager",
    "//muan/wpilib:wpilib_queues",
    ]
)

cc_library(
    name = "superstructure_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "superstructure_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "superstructure_proto",
    srcs = ["superstructure.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "superstructure",
    hdrs = ["superstructure.h"],
    deps = 
    )

cc_library(
    name = "superstructure_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "superstructure_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

)
