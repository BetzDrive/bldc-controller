load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "stm32f4xx_hal_driver",
    srcs = glob(
        ["Src/*.c"],
        exclude = ["Src/*_template.c"],
    ),
    hdrs = glob(["Inc/*.h", "Inc/Legacy/*.h"]),
    includes = ["Inc", "Inc/Legacy"],
    copts = ["-Wno-unused-parameter"],
    visibility = ["//visibility:public"],
    deps = [
        "@cmsis_device_f4//:cmsis_device_f4",
        "@//third_party:stm32f4xx_hal_conf",
    ],
)
