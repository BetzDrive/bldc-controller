load("@rules_meta//meta:defs.bzl", "meta")

cc_f4_binary = meta.wrap_with_transition(
    native.cc_binary,
    {
        "platforms": meta.replace_with(["@bazel_embedded//platforms:cortex_m4_fpu"]),
    },
    executable = True,
)
