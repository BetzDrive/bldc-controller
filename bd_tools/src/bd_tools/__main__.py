"""The betz drive tools package."""
import dataclasses
from typing import Union

import dcargs

import bd_tools


def cli_main(
    cmd: Union[
        bd_tools.bin.calibrate_encoder.Calibrate,
        bd_tools.bin.control_motor.Control,
    ]
):
    cmd.main()


def main():
    dcargs.cli(cli_main)


if __name__ == "__main__":
    main()
