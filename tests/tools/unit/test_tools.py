import dataclasses

import pytest

from tools import (calibrate_encoder, comms, control_motor, read_sensor,
                   update_calibration, upload_bootloader, upload_firmware,
                   view_control_loop)


@dataclasses.dataclass
class BaseArgs:
    """Default arguments used by all tools."""
    serial: str = '/dev/ttyUSB0'
    board_ids: str = '1'
    baud_rate: int = comms.COMM_DEFAULT_BAUD_RATE


@pytest.fixture
def default_mock_comms(mocker):
    mocker.patch('tools.comms.BLDCControllerClient.enumerateBoards',
                 lambda self, x: x)
    mocker.patch('tools.comms.BLDCControllerClient.confirmBoards',
                 lambda self, x: x)
    mocker.patch('tools.comms.BLDCControllerClient.leaveBootloader',
                 lambda self, x: x)
    mocker.patch('tools.comms.BLDCControllerClient.checkWDGRST',
                 lambda self: [])
    mocker.patch('tools.comms.BLDCControllerClient.clearWDGRST',
                 lambda self, x: x)


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

    mocker.patch('tools.calibrate_encoder.serial')
    mocker.patch('tools.boards.initMotor')
    mocker.patch('tools.boards.driveMotor')
    mocker.patch('tools.comms.BLDCControllerClient.writeRegisters')
    mocker.patch('tools.comms.BLDCControllerClient.resetRecorderBuffer',
                 lambda self, bids: [1])
    mocker.patch('tools.comms.BLDCControllerClient.startRecorder',
                 lambda self, bids: [1])
    mocker.patch('tools.comms.BLDCControllerClient.getRecorderLength',
                 lambda self, bids: [1])
    mocker.patch(
        'tools.comms.BLDCControllerClient.getRecorderElement',
        lambda self, bids, indexes: [0.0] * comms.COMM_NUM_RECORDER_ELEMENTS)

    # Fake a slope to stop the math from breaking.
    def grad_generator():
        pos = 0

        def grad(client, bids):
            nonlocal pos
            pos += 5.0
            return [pos]

        return grad

    mocker.patch('tools.comms.BLDCControllerClient.getRawRotorPosition',
                 grad_generator())

    mocker.patch('tools.calibrate_encoder.input')
    mocker.patch('tools.calibrate_encoder.open')
    mocker.patch('tools.calibrate_encoder.json')

    calibrate_encoder.action(Args())


def test_control_motor(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        num_iters: int = 1
        mode: str = 'torque'
        actuations: str = '0.1'

    mocker.patch('tools.control_motor.serial')
    mocker.patch('tools.boards.initMotor')

    control_motor.action(Args())


def test_read_sensor(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        sensor: str = 'temp'
        num_iters: int = 1

    mocker.patch('tools.read_sensor.serial')

    read_sensor.action(Args())


def test_update_calibration(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""

    mocker.patch('tools.update_calibration.serial')
    mocker.patch('tools.comms.BLDCControllerClient.getTorqueConstant',
                 lambda x, y: [0.0] * len(y))
    mocker.patch('tools.comms.BLDCControllerClient.setWatchdogTimeout')
    mocker.patch('tools.comms.BLDCControllerClient.setDirectCurrentKp')
    mocker.patch('tools.comms.BLDCControllerClient.setDirectCurrentKi')
    mocker.patch('tools.comms.BLDCControllerClient.setQuadratureCurrentKp')
    mocker.patch('tools.comms.BLDCControllerClient.setQuadratureCurrentKi')
    mocker.patch('tools.comms.BLDCControllerClient.setVelocityKp')
    mocker.patch('tools.comms.BLDCControllerClient.setVelocityKd')
    mocker.patch('tools.comms.BLDCControllerClient.setPositionKp')
    mocker.patch('tools.comms.BLDCControllerClient.setPositionKd')
    mocker.patch('tools.comms.BLDCControllerClient.setCurrentLimit')
    mocker.patch('tools.comms.BLDCControllerClient.setTorqueLimit')
    mocker.patch('tools.comms.BLDCControllerClient.setVelocityLimit')
    mocker.patch('tools.comms.BLDCControllerClient.storeCalibration')

    update_calibration.action(Args())


def test_upload_bootloader(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        bin_file: str = 'file.bin'
        offset: int = 0x08000000

    mocker.patch('tools.upload_bootloader.serial')
    mocker.patch('tools.upload_bootloader.open')
    mocker.patch('tools.comms.BLDCControllerClient.getFlashSectorMap')
    mocker.patch('tools.comms.BLDCControllerClient.writeFlash')

    upload_bootloader.action(Args())


def test_upload_firmware(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        bin_file: str = 'file.bin'
        offset: int = 0x08000000

    mocker.patch('tools.upload_firmware.serial')
    mocker.patch('tools.upload_firmware.open')
    mocker.patch('tools.comms.BLDCControllerClient.getFlashSectorMap')
    mocker.patch('tools.comms.BLDCControllerClient.writeFlash')

    upload_firmware.action(Args())


def test_view_control_loop(mocker, default_mock_comms):
    """Ensures script runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        mode: str = 'torque'
        actuations: str = '0.1'

    mocker.patch('tools.view_control_loop.serial')
    mocker.patch('tools.boards.initMotor')
    mocker.patch('tools.boards.driveMotor')
    mocker.patch('tools.livegraph.LiveGraph.start')

    view_control_loop.action(Args())
