load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "cmsis_device_f4",
    hdrs = glob(["Include/*.h"]),
    includes = ["Include"],
    defines = ["STM32F405xx"],
    visibility = ["//visibility:public"],
    deps = ["@cmsis_5//:cmsis_core"],
)
