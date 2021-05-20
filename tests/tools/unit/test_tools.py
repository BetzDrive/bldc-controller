import dataclasses

import pytest

from bd_tools import (
    calibrate_encoder,
    comms,
    control_motor,
    read_sensor,
    update_calibration,
    upload_bootloader,
    upload_firmware,
    view_control_loop,
)


@dataclasses.dataclass
class BaseArgs:
    """Default arguments used by all bd_tools."""

    serial: str = "/dev/ttyUSB0"
    board_ids: str = "1"
    baud_rate: int = comms.COMM_DEFAULT_BAUD_RATE


@pytest.fixture
def default_mock_comms(mocker):
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.enumerateBoards", lambda self, x: x
    )
    mocker.patch("bd_tools.comms.BLDCControllerClient.confirmBoards", lambda self, x: x)
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.leaveBootloader", lambda self, x: x
    )
    mocker.patch("bd_tools.comms.BLDCControllerClient.checkWDGRST", lambda self: [])
    mocker.patch("bd_tools.comms.BLDCControllerClient.clearWDGRST", lambda self, x: x)


def test_calibrate_encoder(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

        duty_cycle: float = 0.3
        max_steps: int = 5
        board_id: int = 1
        delay: float = 0.1

    args = Args()

    mocker.patch("bd_tools.calibrate_encoder.serial")
    mocker.patch("bd_tools.boards.initMotor")
    mocker.patch("bd_tools.boards.driveMotor")
    mocker.patch("bd_tools.comms.BLDCControllerClient.writeRegisters")
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.resetRecorderBuffer",
        lambda self, bids: [1],
    )
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.startRecorder", lambda self, bids: [1]
    )
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.getRecorderLength", lambda self, bids: [1]
    )
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.getRecorderElement",
        lambda self, bids, indexes: [(0.0,) * comms.COMM_NUM_RECORDER_ELEMENTS],
    )

    # Fake a slope to stop the math from breaking.
    def grad_generator():
        pos = 0

        def grad(client, bids):
            nonlocal pos
            pos += 5.0
            return [pos]

        return grad

    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.getRawRotorPosition", grad_generator()
    )

    mocker.patch("bd_tools.calibrate_encoder.input")
    mocker.patch("bd_tools.calibrate_encoder.open")
    mocker.patch("bd_tools.calibrate_encoder.json")

    calibrate_encoder.action(Args())


def test_control_motor(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

        num_iters: int = 1
        mode: str = "torque"
        actuations: str = "0.1"

    mocker.patch("bd_tools.control_motor.serial")
    mocker.patch("bd_tools.boards.initMotor")

    control_motor.action(Args())


def test_read_sensor(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

        sensor: str = "temp"
        num_iters: int = 1

    mocker.patch("bd_tools.read_sensor.serial")

    read_sensor.action(Args())


def test_update_calibration(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

    mocker.patch("bd_tools.update_calibration.serial")
    mocker.patch(
        "bd_tools.comms.BLDCControllerClient.getTorqueConstant",
        lambda x, y: [0.0] * len(y),
    )
    mocker.patch("bd_tools.comms.BLDCControllerClient.setWatchdogTimeout")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setDirectCurrentKp")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setDirectCurrentKi")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setQuadratureCurrentKp")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setQuadratureCurrentKi")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setVelocityKp")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setVelocityKd")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setPositionKp")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setPositionKd")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setCurrentLimit")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setTorqueLimit")
    mocker.patch("bd_tools.comms.BLDCControllerClient.setVelocityLimit")
    mocker.patch("bd_tools.comms.BLDCControllerClient.storeCalibration")

    update_calibration.action(Args())


def test_upload_bootloader(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

        bin_file: str = "file.bin"
        offset: int = 0x08000000

    mocker.patch("bd_tools.upload_bootloader.serial")
    mocker.patch("bd_tools.upload_bootloader.open")
    mocker.patch("bd_tools.comms.BLDCControllerClient.getFlashSectorMap")
    mocker.patch("bd_tools.comms.BLDCControllerClient.writeFlash")

    upload_bootloader.action(Args())


def test_upload_firmware(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

        bin_file: str = "file.bin"
        offset: int = 0x08000000

    mocker.patch("bd_tools.upload_firmware.serial")
    mocker.patch("bd_tools.upload_firmware.open")
    mocker.patch("bd_tools.comms.BLDCControllerClient.getFlashSectorMap")
    mocker.patch("bd_tools.comms.BLDCControllerClient.writeFlash")

    upload_firmware.action(Args())


def test_view_control_loop(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""

    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

        mode: str = "torque"
        actuations: str = "0.1"

    mocker.patch("bd_tools.view_control_loop.serial")
    mocker.patch("bd_tools.boards.initMotor")
    mocker.patch("bd_tools.boards.driveMotor")
    mocker.patch("bd_tools.livegraph.LiveGraph.start")

    view_control_loop.action(Args())
