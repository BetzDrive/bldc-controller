def _flash_remote_impl(ctx):
    # Write firmware to memory location 0x8000000 i.e. start of executable flash
    if ctx.attr.upload_type not in ["bootloader", "firmware"]:
        print("Invalid upload type {}".format(ctx.attr.upload_type))
        return
    script_template = """
python3 -m bd_tools upload_{upload_type} {interface} {devices} {binary}
"""
    script = ctx.actions.declare_file("%s.sh" % ctx.label.name)

    script_content = script_template.format(
        upload_type = ctx.attr.upload_type,
        interface = ctx.attr.interface,
        devices = ctx.attr.devices,
        binary = ctx.file.image.short_path,
    )
    ctx.actions.write(script, script_content, is_executable = True)
    runfiles = ctx.runfiles(files = [ctx.file.image])
    return [DefaultInfo(executable = script, runfiles = runfiles)]

flash_remote = rule(
    implementation = _flash_remote_impl,
    doc = """
Used to flash a binary image to a betzdrive over protocol v3
Example:
    flash_remote(
        name = "upload",
        upload_type = "firmware",
        image = ":firmware",
    )
""",
    attrs = {
        "image": attr.label(
            doc = "Binary image to flash",
            mandatory = True,
            allow_single_file = True,
        ),
        "upload_type": attr.string(
            doc = "whether firmware or bootloader is being uploaded",
            mandatory = True,
        ),
        "interface": attr.string(
            doc = "path to serial interface over which to flash",
            mandatory = False,
            default = "/dev/ttyUSB0",
        ),
        "devices": attr.string(
            doc = "csv of betzdrive device ids to flash (i.e. 1,2,3)",
            mandatory = False,
            default = "1",
        ),
    },
    executable = True,
)
