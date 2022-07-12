load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party:variables.bzl", "chibios_inc_paths")

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
    srcs = glob(["src/*.c"]) + glob(["src/*.cpp"]),
    hdrs = glob(["include/*.h"]) + glob(["include/*.hpp"]),
    copts = chibios_inc_paths,
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = [
        "//third_party:chibios_hal",
    ],
)
