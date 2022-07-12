load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//third_party:variables.bzl", "chibios_inc_paths")
load("//:platforms.bzl", "cc_f4_binary")

cc_f4_binary(
    name = "bootloader",
    srcs = glob(["src/*.c"]) + glob(["src/*.cpp"]),
    copts = chibios_inc_paths + [
        "-O2",
        "-ggdb",
        "-ffunction-sections",
        "-fdata-sections",
        "-fno-common",
    ],
    linkopts = [
        "-T $(location :link.ld)",
        "-lc",
        "-lm",
        "-lnosys",
        "-u _printf_float",
    ],
    target_compatible_with = [
        "@platforms//cpu:armv7e-m",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":link.ld",
        ":main_libs",
        "//third_party:chibios_hal",
    ],
)

cc_library(
    name = "main_libs",
    hdrs = glob(
        ["include/*.h"],
        exclude = [
            "include/stm32f4xx_conf.h",
            "include/chconf.h",
            "include/halconf.h",
            "include/mcuconf.h",
        ],
    ) + glob(
        ["include/*.hpp"],
        exclude = [
            "include/peripherals.hpp",
        ],
    ),
    copts = chibios_inc_paths,
    strip_include_prefix = "include",
    deps = [
        "//common",
    ],
)

cc_library(
    name = "os_config",
    hdrs = [
        "include/chconf.h",
        "include/halconf.h",
        "include/mcuconf.h",
        "include/peripherals.hpp",
        "include/stm32f4xx_conf.h",
    ],
    includes = ["include"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)
