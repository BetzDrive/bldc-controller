load("@rules_meta//meta:defs.bzl", "meta")
load("@rules_cc//cc:find_cc_toolchain.bzl", "find_cc_toolchain")
load("//toolchains:transitions.bzl", "switch_to_cortex_m4")

firmware_binary = meta.wrap_with_transition(
    native.cc_binary,
    {
        "platforms": meta.replace_with([
            "@bazel_embedded//platforms:cortex_m4_fpu",
        ]),
        "compilation_mode": "dbg",
    },
    executable = True,
)

bootloader_binary = meta.wrap_with_transition(
    native.cc_binary,
    {
        "platforms": meta.replace_with([
            "@bazel_embedded//platforms:cortex_m4_fpu",
        ]),
        "compilation_mode": "fastbuild",
    },
    executable = True,
)

def _impl(ctx):
    src = ctx.attr.src.files.to_list()[0]
    toolchain = find_cc_toolchain(ctx)

    # The raw binary is actually an elf file. We copy it to a .elf file extension.
    ctx.actions.run_shell(
        command = "cp {raw} {elf_out}".format(
            elf_out = ctx.outputs.elf.path,
            raw = src.path,
        ),
        outputs = [ctx.outputs.elf],
        inputs = [src],
    )

    ctx.actions.run_shell(
        command = "{objcopy} -O binary {elf_in} {cc_bin}".format(
            objcopy = toolchain.cc.objcopy_executable,
            elf_in = ctx.outputs.elf.path,
            cc_bin = ctx.outputs.bin.path,
        ),
        outputs = [ctx.outputs.bin],
        inputs = [ctx.outputs.elf],
        execution_requirements = {
            "local": "1",
        },
    )

gen_binary = rule(
    implementation = _impl,
    toolchains = [
        "@rules_cc//cc:toolchain_type",
    ],
    attrs = {
        "_cc_toolchain": attr.label(
            default = Label("@rules_cc//cc:current_cc_toolchain"),
          cfg=switch_to_cortex_m4,
        ),
        "src": attr.label(allow_single_file = True),
        "_allowlist_function_transition": attr.label(
             default = "@bazel_tools//tools/allowlists/function_transition_allowlist"
        ),
    },
    outputs = {
        "elf": "%{name}.elf",
        "bin": "%{name}.bin",
    },
    # We set the config to transition to the cortex m4 platform
    incompatible_use_toolchain_transition = True,
)
