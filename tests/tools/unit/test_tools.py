import dataclasses

import pytest

from tools import calibrate_encoder, comms, control_motor, read_sensor


@dataclasses.dataclass
class BaseArgs:
    """Default arguments used by all tools."""
    serial: str = '/dev/ttyUSB0'
    board_ids: str = '1'
    baud_rate: int = comms.COMM_DEFAULT_BAUD_RATE

def _default_mock_comms(mocker):
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

def test_calibrate_encoder(mocker):
    """Ensures read control motor runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        duty_cycle: float = 0.3
        max_steps: int = 5
        board_id: int = 1
        delay: float = 0.1

    args = Args()

    _default_mock_comms(mocker)

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
    mocker.patch('tools.comms.BLDCControllerClient.getRecorderElement',
                 lambda self, bids, indexes: [0.0] * comms.COMM_NUM_RECORDER_ELEMENTS)

    # Fake a slope to stop the math from breaking.
    def grad_generator():
        pos = 0
        def grad(client, bids):
            nonlocal pos
            pos += 5.0
            return [pos]
        return grad

    mocker.patch('tools.comms.BLDCControllerClient.getRawRotorPosition', grad_generator())

    mocker.patch('tools.calibrate_encoder.input')
    mocker.patch('tools.calibrate_encoder.json')

    calibrate_encoder.action(Args())


def test_control_motor(mocker):
    """Ensures read control motor runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        num_iters: int = 1
        mode: str = 'torque'
        actuations: str = '0.1'

    _default_mock_comms(mocker)

    mocker.patch('tools.control_motor.serial')
    mocker.patch('tools.boards.initMotor')

    control_motor.action(Args())


def test_read_sensor(mocker):
    """Ensures read sensor runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        sensor: str = 'temp'
        num_iters: int = 1

    _default_mock_comms(mocker)

    mocker.patch('tools.read_sensor.serial')

    read_sensor.action(Args())

