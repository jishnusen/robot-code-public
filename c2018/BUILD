load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = ["main.cpp"],
    deps = [
        "//c2018/subsystems:subsystem_runner",
        "//c2018/teleop:main",
        "//c2018/autonomous:autonomous_runner",
        "//c2018/webdash:webdash_c2018",
        "//third_party/frc971/control_loops/drivetrain:drivetrain_queues",
        "//muan/wpilib:ds_sender",
        "//muan/queues:queue_manager",
        "//third_party/gflags:gflags",
        "//third_party/wpilibsuite:wpilib",
    ],
    data = [
        "//c2018/webdash:json",
    ],
    restricted_to = ["//tools:roborio"],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)
