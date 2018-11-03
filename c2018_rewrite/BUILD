load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = [
        "main.cpp",
    ],
    restricted_to = ["//tools:roborio"],
    deps = [
        "//c2018_rewrite/subsystems:subsystem_runner",
        "//c2018_rewrite/teleop",
        "//c2018_rewrite/autonomous:autonomous_runner",
        "//c2018_rewrite/webdash",
        "//third_party/gflags",
        "//third_party/wpilibsuite:wpilib",
    ],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)
