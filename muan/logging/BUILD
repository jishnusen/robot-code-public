package(default_visibility = ["//visibility:public"])

load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "logger",
    srcs = [
        "logger.hpp",
        "logger.cpp"
    ],
    hdrs = ["logger.h"],
    deps = [
        "filewriter",
        ":textlogger",
        "//muan/queues:queues",
        "//muan/utils:proto_utils",
        "//muan/utils:threading_utils",
        "//third_party/aos/linux_code:init",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/common:time",
        "//third_party/google/protobuf",
        "//third_party/optional",
        "//third_party/google/googletest:googletest"
    ],
    linkstatic = 1
)

cc_library(
    name = "filewriter",
    srcs = ["filewriter.cpp"],
    hdrs = ["filewriter.h"],
    deps = [
        "//third_party/boost:filesystem",
        "//third_party/gflags:gflags"
    ],
)

cc_library(
    name = "textlogger",
    srcs = [
        "textlogger.hpp",
        "textlogger.cpp"
    ],
    hdrs = ["textlogger.h"],
    deps = [
        "//muan/queues",
        "//muan/utils:threading_utils",
    ],
)

cc_test(
    name = "logger_test",
    srcs = ["logger_test.cpp"],
    deps = [
        "filewriter",
        ":logger",
        "logger_test_protobufs",
        "//muan/proto:stack_proto",
        "//third_party/google/googletest",
        "//third_party/google/googletest:googlemock",
    ],
)

cc_proto_library(
    name = "logger_test_protobufs",
    srcs = ["logger_test.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "textlogger_test",
    srcs = ["textlogger_test.cpp"],
    deps = [
        "textlogger",
        "//third_party/google/googletest",
        "//muan/logging:logger",
    ],
)
