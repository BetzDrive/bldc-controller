# To generate the Python protobuf file manually:
#   protoc --python_out=. motor_control.proto
# This will create motor_control_pb2.py in the same directory as this script.

import time
import sys
from py_shm_circular_buffer import (
    open_motor_server_shm, COMMAND_BUF_OFFSET, STATUS_BUF_OFFSET, BUF_SIZE, CircularBuffer
)
import os

# Force Python to use the user-installed protobuf
user_site = os.path.expanduser("~/.local/lib/python3.10/site-packages")
if user_site not in sys.path:
    sys.path.insert(0, user_site)

# Now import protobuf
import google.protobuf

# Import the generated protobufs (assume motor_control_pb2.py is in the same directory)
try:
    import motor_control_pb2
except ImportError as e:
    print("Could not import motor_control_pb2. Make sure you have generated it with protoc.")
    print("Error:", e)
    sys.exit(1)

# Open shared memory and buffers
shm = open_motor_server_shm()
cmd_buf = CircularBuffer(shm, COMMAND_BUF_OFFSET, BUF_SIZE)
status_buf = CircularBuffer(shm, STATUS_BUF_OFFSET, BUF_SIZE)

# Create a MotorCommand
cmd = motor_control_pb2.MotorCommand()
cmd.board_id = 1
cmd.mode = "torque"
cmd.values.extend([0.2])
cmd.timestamp = int(time.time() * 1e6)

# Serialize and send
payload = cmd.SerializeToString()
if not cmd_buf.write_framed(payload):
    print("Failed to write command (buffer full?)")
    sys.exit(1)
print("Command sent.")

# Wait for a status response
for _ in range(100):  # Try for up to ~1s
    status_bytes = status_buf.read_framed()
    if status_bytes:
        status = motor_control_pb2.MotorStatus()
        status.ParseFromString(status_bytes)
        print(f"Status: board_id={status.board_id} success={status.success} error='{status.error_message}' timestamp={status.timestamp}")
        break
    time.sleep(0.01)
else:
    print("No status response received.") 