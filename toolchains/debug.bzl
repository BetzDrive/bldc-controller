def _debug_gdb_impl(ctx):
    script_template = """
gdb-multiarch -ex "target remote localhost:{port}" {elf}
"""
    script = ctx.actions.declare_file("%s.sh" % ctx.label.name)

    script_content = script_template.format(
        port = ctx.attr.port,
        elf = ctx.file.elf.short_path,
    )
    ctx.actions.write(script, script_content, is_executable = True)
    runfiles = ctx.runfiles(files = [ctx.file.elf])
    return [DefaultInfo(executable = script, runfiles = runfiles)]

debug_gdb = rule(
    implementation = _debug_gdb_impl,
    doc = """
Used to debug the flashed image. Make sure to flash prior to running this
commmand so the elf is in sync with the binary.
Example:
    debug_gdb(
        name = "debug_gdb",
        elf = ":firmware.elf",
    )
""",
    attrs = {
        "elf": attr.label(
            doc = "Elf corresponding to flashed binary.",
            mandatory = True,
            allow_single_file = True,
        ),
        "port": attr.string(
            doc = "port opened by openocd",
            mandatory = False,
            default = "3333",
        ),
    },
    executable = True,
)
