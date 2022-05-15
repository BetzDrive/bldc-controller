"""The betz drive tools package."""
import argparse
import sys
from pathlib import Path

BIN_PATH = Path(__file__).parent / "bin"


def get_tools():
    """Returns all scripts in the bin directory"""
    return [tool.stem for tool in BIN_PATH.glob("[!__]*.py")]


def parser_args(tools):
    parser = argparse.ArgumentParser(description="BetzDrive Tools Package")
    parser.add_argument(
        "tool", type=str, choices=tools, help="Name of the tool to use."
    )
    return parser.parse_args()


def action(args):
    file_name = BIN_PATH / (args.tool + ".py")
    with open(file_name) as f:
        # NOTE(greg): Shifts argv down one (and deletes the 0th arg) so the
        # sub-tool does not see its own name as the 1st arg.
        sys.argv = sys.argv[1:]
        # Correctly make arg0 the path to the file we're executing.
        sys.argv[0] = str(file_name)
        code = compile(f.read(), file_name.name, "exec")
        exec(code, globals())


if __name__ == "__main__":
    # NOTE(greg): We have to hack the help to make sure it only operates on
    # this argparser if its the first arg.
    tool_help = False
    if "-h" in sys.argv or "--help" in sys.argv:
        if not (sys.argv[1] == "-h" or sys.argv[1] == "--help"):
            tool_help = True
            if "-h" in sys.argv:
                sys.argv.remove("-h")
            if "--help" in sys.argv:
                sys.argv.remove("--help")

    args = parser_args(get_tools())

    # If we requested a tool help, inject it back into the sys args for the
    # sub-tool to process.
    if tool_help:
        sys.argv.append("--help")

    action(args)
