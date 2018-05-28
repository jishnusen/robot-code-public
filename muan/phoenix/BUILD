package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sp_factory",
    srcs = ["sp_factory.cpp"],
    hdrs = ["sp_factory.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//third_party/phoenixsuite:phoenix",
    ],
)

cc_library(
    name = "talon_wrapper",
    srcs = ["talon_wrapper.cpp"],
    hdrs = ["talon_wrapper.h"],
    restricted_to = ["//tools:roborio"],
    deps = [
        ":sp_factory",
    ],
)
