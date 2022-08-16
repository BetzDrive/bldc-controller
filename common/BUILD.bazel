load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party:variables.bzl", "project_inc_paths")

filegroup(
    name = "board_config_hdrs",
    srcs = ["board.h"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "board_config_srcs",
    srcs = ["board.c"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "common",
    srcs = glob([
        "src/*.c",
        "src/*.cpp",
    ]),
    hdrs = (
        glob([
            "include/*.h",
            "include/*.hpp",
        ]) + select({
            "//toolchains/programs:bootloader": ["//bootloader:peripheral_config"],
            "//toolchains/programs:firmware": ["//firmware:peripheral_config"],
            "//conditions:default": [],
        })
    ),
    copts = project_inc_paths,
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:chibios_hal",
    ],
    alwayslink = True,
)
