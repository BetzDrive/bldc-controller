import mmap
import struct
import os
import ctypes
from typing import Optional

# Settings must match C++
SHM_NAME = "/motor_server_shm"
SHM_SIZE = 65536
BUF_SIZE = 32768

# Offsets in shared memory
COMMAND_BUF_OFFSET = 0
STATUS_BUF_OFFSET = BUF_SIZE

class CircularBuffer:
    def __init__(self, shm: mmap.mmap, offset: int, size: int):
        self.shm = shm
        self.offset = offset
        self.size = size
        self.head = ctypes.c_size_t.from_buffer(self.shm, self.offset + size)  # head after buffer
        self.tail = ctypes.c_size_t.from_buffer(self.shm, self.offset + size + ctypes.sizeof(ctypes.c_size_t))  # tail after head
        self.buf = (ctypes.c_uint8 * size).from_buffer(self.shm, self.offset)

    def _get_head(self):
        return self.head.value
    def _get_tail(self):
        return self.tail.value
    def _set_head(self, v):
        self.head.value = v
    def _set_tail(self, v):
        self.tail.value = v

    def available(self):
        head = self._get_head()
        tail = self._get_tail()
        if head >= tail:
            return head - tail
        return self.size - (tail - head)

    def free(self):
        return self.size - 1 - self.available()

    def write(self, data: bytes) -> bool:
        if len(data) > self.free():
            return False
        head = self._get_head()
        for i in range(len(data)):
            self.buf[(head + i) % self.size] = data[i]
        self._set_head((head + len(data)) % self.size)
        return True

    def read(self, n: int) -> bytes:
        avail = self.available()
        to_read = min(avail, n)
        tail = self._get_tail()
        out = bytearray(to_read)
        for i in range(to_read):
            out[i] = self.buf[(tail + i) % self.size]
        self._set_tail((tail + to_read) % self.size)
        return bytes(out)

    def peek(self, n: int) -> bytes:
        avail = self.available()
        to_read = min(avail, n)
        tail = self._get_tail()
        out = bytearray(to_read)
        for i in range(to_read):
            out[i] = self.buf[(tail + i) % self.size]
        return bytes(out)

    def clear(self):
        self._set_head(0)
        self._set_tail(0)

    # Framed message: [uint32 len][bytes]
    def write_framed(self, payload: bytes) -> bool:
        if self.free() < len(payload) + 4:
            return False
        length_bytes = struct.pack("I", len(payload))
        return self.write(length_bytes + payload)

    def read_framed(self) -> Optional[bytes]:
        if self.available() < 4:
            return None
        length_bytes = self.peek(4)
        msg_len = struct.unpack("I", length_bytes)[0]
        if msg_len == 0 or msg_len > self.size - 4:
            return None
        if self.available() < 4 + msg_len:
            return None
        self.read(4)  # consume length
        return self.read(msg_len)


def open_motor_server_shm():
    fd = os.open("/dev/shm" + SHM_NAME, os.O_RDWR)
    shm = mmap.mmap(fd, SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE | mmap.PROT_READ)
    return shm

# Usage example:
# shm = open_motor_server_shm()
# cmd_buf = CircularBuffer(shm, COMMAND_BUF_OFFSET, BUF_SIZE)
# status_buf = CircularBuffer(shm, STATUS_BUF_OFFSET, BUF_SIZE)
# cmd_buf.write_framed(b'...')
# msg = status_buf.read_framed() 