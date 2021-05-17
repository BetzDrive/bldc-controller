import dataclasses

import pytest

from tools import comms, read_sensor


def test_read_sensor(mocker):
    """Ensures read sensor runs when given valid arguments."""
    @dataclasses.dataclass
    class Args:
        """Args for this tool."""
        serial: str = '/dev/ttyUSB0'
        board_ids: str = '1'
        sensor: str = 'temp'
        num_iters: int = 1
        baud_rate: int = comms.COMM_DEFAULT_BAUD_RATE

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
