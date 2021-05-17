import dataclasses

import pytest

from tools import comms, control_motor, read_sensor


@dataclasses.dataclass
class BaseArgs:
    """Default arguments used by all tools."""
    serial: str = '/dev/ttyUSB0'
    board_ids: str = '1'
    baud_rate: int = comms.COMM_DEFAULT_BAUD_RATE


def test_read_sensor(mocker):
    """Ensures read sensor runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        sensor: str = 'temp'
        num_iters: int = 1

    mocker.patch('tools.read_sensor.serial')

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

    read_sensor.action(Args())


def test_control_motor(mocker):
    """Ensures read control motor runs when given valid arguments."""
    @dataclasses.dataclass
    class Args(BaseArgs):
        """Args for this tool."""
        num_iters: int = 1
        mode: str = 'torque'
        actuations: str = '0.1'

    mocker.patch('tools.control_motor.serial')

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

    mocker.patch('tools.boards.initMotor')

    control_motor.action(Args())
