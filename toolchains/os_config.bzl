load("@rules_cc//cc:defs.bzl", "cc_library")

def os_config(name, program, **kwargs):
    cc_library(
        name = name + "_lib",
        hdrs = [
            "include/chconf.h",
            "include/halconf.h",
            "include/mcuconf.h",
            "include/stm32f4xx_conf.h",
        ],
        includes = ["include"],
        visibility = ["//visibility:public"],
    )

    program_select(
        name = name + "_transition",
        program = program,
    )

def _program_transition_impl(settings, attr):
    _ignore = settings, attr
    return {"//toolchains/programs": "//toolchains/programs:" + attr.program}

program_transition = transition(
    implementation = _program_transition_impl,
    inputs = [],
    outputs = ["//toolchains/programs"],
)

def _program_select_impl(ctx):
    pass

program_select = rule(
    implementation = _program_select_impl,
    attrs = {
        "dep": attr.label(cfg = program_transition),
        "program": attr.string(),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)
