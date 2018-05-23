import struct
import time
import json
import crcmod

class ProtocolError(Exception):
    def __init__(self, message, errors=None):
        super(ProtocolError, self).__init__(message)

        self.errors = errors

COMM_ERRORS_NONE = 0
COMM_ERRORS_OP_FAILED = 1
COMM_ERRORS_MALFORMED = 2
COMM_ERRORS_INVALID_FC = 4
COMM_ERRORS_INVALID_ARGS = 8
COMM_ERRORS_BUF_LEN_MISMATCH = 16

COMM_FC_NOP = 0x00
COMM_FC_READ_REGS = 0x01
COMM_FC_WRITE_REGS = 0x02
COMM_FC_SYSTEM_RESET = 0x80
COMM_FC_JUMP_TO_ADDR = 0x81
COMM_FC_FLASH_SECTOR_COUNT = 0x82
COMM_FC_FLASH_SECTOR_START = 0x83
COMM_FC_FLASH_SECTOR_SIZE = 0x84
COMM_FC_FLASH_SECTOR_ERASE = 0x85
COMM_FC_FLASH_PROGRAM = 0x86
COMM_FC_FLASH_READ = 0x87
COMM_FC_FLASH_VERIFY = 0x88
COMM_FC_FLASH_VERIFY_ERASED = 0x89

COMM_FLAG_SEND = 0x00 
COMM_FLAG_FIRST_MESSAGE = 0x02
COMM_FLAG_LAST_MESSAGE = 0x04

COMM_BOOTLOADER_OFFSET = 0x08000000
COMM_NVPARAMS_OFFSET = 0x08004000
COMM_FIRMWARE_OFFSET = 0x08008000
COMM_DEFAULT_BAUD_RATE = 1000000

COMM_SINGLE_PROGRAM_LENGTH = 128
COMM_SINGLE_READ_LENGTH = 128
COMM_SINGLE_VERIFY_LENGTH = 128

CRC16IBM = 0x8005

class FlashSectorMap:
    def __init__(self, sector_count, sector_starts, sector_sizes):
        if len(sector_starts) != sector_count or len(sector_sizes) != sector_count:
            raise ValueError('sector_starts and sector_sizes must have the correct length')
        self._sector_count = sector_count
        self._sector_starts = sector_starts
        self._sector_sizes = sector_sizes

    def getFlashSectorCount(self):
        return self._sector_count

    def getFlashSectorStart(self, sector_num):
        return self._sector_starts[sector_num]

    def getFlashSectorEnd(self, sector_num):
        return self._sector_starts[sector_num] + self._sector_sizes[sector_num]

    def getFlashSectorSize(self, sector_num):
        return self._sector_sizes[sector_num]

    def getFlashSectorOfAddress(self, addr):
        for i in range(self.getFlashSectorCount()):
            if self.getFlashSectorStart(i) <= addr < self.getFlashSectorEnd(i):
                return i

        return None

    def getFlashSectorsOfAddressRange(self, addr, length):
        sector_nums = []
        offset = 0

        while offset < length:
            sector_num = self.getFlashSectorOfAddress(addr + offset)

            if sector_num is None:
                return None

            sector_nums.append(sector_num)
            offset = self.getFlashSectorEnd(sector_num) - addr

        return sector_nums

    def __str__(self):
        lines = ['num       start     size']
        for sector_num in range(self.getFlashSectorCount()):
            lines.append('{:3d}  0x{:08x} {:8d}'.format(sector_num, self.getFlashSectorStart(sector_num), self.getFlashSectorSize(sector_num)))
        return '\n'.join(lines)

class BLDCControllerClient:
    def __init__(self, ser, protocol=0):
        self._ser = ser
        self._protocol = protocol

    def setCurrent(self, id, value):
        ret = self.writeRegisters(id, 0x0102, 1, struct.pack('<f', value))
        return ret

    def getEncoder(self, id):
        print(id)
        angle = struct.unpack('<H', self.readRegisters(id, 0x100, 1))[0]
        return angle

    def setDuty(self, id, value):
        ret = self.writeRegisters(id, 0x0106, 1, struct.pack('<f', value ))
        return ret
    
    def setControlEnabled(self, id, logical):
        self.writeRegisters(id, 0x0102, 1, struct.pack('<B', logical))

    def leaveBootloader(self, server_id):
        self.jumpToAddress(server_id, COMM_FIRMWARE_OFFSET)

    def leaveMultiBootloader(self, server_id):
        for sid in server_id:
            self.jumpToAddress(sid, COMM_FIRMWARE_OFFSET)
            time.sleep(0.2)

    def enterBootloader(self, server_id):
        self.resetSystem(server_id)

    def readRegisters(self, server_id, start_addr, count):
        success, data = self.doTransaction(server_id, COMM_FC_READ_REGS, struct.pack('<HB', start_addr, count))
        if not success:
            raise IOError("Register read failed")
        return data

    def readMultiRegisters(self, server_id, start_addr, count):
        responses = self.doMultiTransaction(server_id, [COMM_FC_READ_REGS]*len(server_id), [struct.pack('<HB', start_addr, count)]*len(server_id))
        data = [0]*len(responses)
        for i in range(len(responses)):
            #print(responses)
            success = responses[i][0]
            if not success:
                raise IOError("Register read failed" + str(server_id[i]))
            data[i] = responses[i][1]
        return data

    def writeRegisters(self, server_id, start_addr, count, data):
        success, _ = self.doTransaction(server_id, COMM_FC_WRITE_REGS, struct.pack('<HB', start_addr, count) + data)
        return success

    def resetSystem(self, server_id):
        self.writeRequest(server_id, COMM_FLAG_FIRST_MESSAGE + COMM_FLAG_LAST_MESSAGE, COMM_FC_SYSTEM_RESET)
        return True

    def jumpToAddress(self, server_id, jump_addr=COMM_FIRMWARE_OFFSET):
        self.writeRequest(server_id, COMM_FLAG_SEND + COMM_FLAG_FIRST_MESSAGE + COMM_FLAG_LAST_MESSAGE, COMM_FC_JUMP_TO_ADDR, struct.pack('<I', jump_addr))
        return True

    def getFlashSectorCount(self, server_id):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_COUNT, '')
        return struct.unpack('<I', data)[0]

    def getFlashSectorStart(self, server_id, sector_num):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_START, struct.pack('<I', sector_num))
        return struct.unpack('<I', data)[0]

    def getFlashSectorSize(self, server_id, sector_num):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_SIZE, struct.pack('<I', sector_num))
        return struct.unpack('<I', data)[0]

    def eraseFlashSector(self, server_id, sector_num):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_SECTOR_ERASE, struct.pack('<I', sector_num))
        return success

    def programFlash(self, server_id, dest_addr, data):
        for i in range(0, len(data), COMM_SINGLE_PROGRAM_LENGTH):
            success = self._programFlashLimitedLength(server_id, dest_addr + i, data[i:i+COMM_SINGLE_PROGRAM_LENGTH])
            if not success:
                return False
        return True

    def _programFlashLimitedLength(self, server_id, dest_addr, data):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_PROGRAM, struct.pack('<I', dest_addr) + data)
        return success

    def readFlash(self, server_id, src_addr, length):
        data = ''
        for i in range(0, length, COMM_SINGLE_READ_LENGTH):
            read_len = min(length - i, COMM_SINGLE_READ_LENGTH)
            data_chunk = self._readFlashLimitedLength(server_id, src_addr + i, read_len)
            if len(data_chunk) != read_len:
                return False
            data += data_chunk
        return data

    def _readFlashLimitedLength(self, server_id, src_addr, length):
        _, data = self.doTransaction(server_id, COMM_FC_FLASH_READ, struct.pack('<II', src_addr, length))
        return data

    def verifyFlash(self, server_id, dest_addr, data):
        for i in range(0, len(data), COMM_SINGLE_VERIFY_LENGTH):
            success = self._verifyFlashLimitedLength(server_id, dest_addr + i, data[i:i+COMM_SINGLE_VERIFY_LENGTH])
            if not success:
                return False
        return True

    def _verifyFlashLimitedLength(self, server_id, dest_addr, data):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_VERIFY, struct.pack('<I', dest_addr) + data)
        return success

    def verifyFlashErased(self, server_id, dest_addr, length):
        success, _ = self.doTransaction(server_id, COMM_FC_FLASH_VERIFY_ERASED, struct.pack('<II', dest_addr, length))
        return success

    def eraseFlash(self, server_id, addr, length, sector_map=None):
        if sector_map is None:
            sector_map = self.getFlashSectorMap(server_id)

        # Find out which sectors need to be erased
        sector_nums = sector_map.getFlashSectorsOfAddressRange(addr, length)

        for sector_num in sector_nums:
            success = self.eraseFlashSector(server_id, sector_num)
            if not success:
                return False

        return True

    def readCalibration(self, server_id):
        l = struct.unpack('<H', self.readFlash(server_id, COMM_NVPARAMS_OFFSET+1, 2))[0]
        return json.loads(self.readFlash(server_id, COMM_NVPARAMS_OFFSET+3, l))

    def writeFlash(self, server_id, dest_addr, data, sector_map=None, print_progress=False):
        if sector_map is None:
            sector_map = self.getFlashSectorMap(server_id)
            print sector_map

        if print_progress:
            print "Erasing flash"

        success = self.eraseFlash(server_id, dest_addr, len(data), sector_map)
        if not success:
            return False

        if print_progress:
            print "Verifying flash was erased"

        success = self.verifyFlashErased(server_id, dest_addr, len(data))
        if not success:
            return False

        if print_progress:
            print "Programming flash"

        success = self.programFlash(server_id, dest_addr, data)
        if not success:
            return False

        if print_progress:
            print "Verifying flash was programmed"

        success = self.verifyFlash(server_id, dest_addr, data)
        if not success:
            return False

        return True

    def getFlashSectorMap(self, server_id):
        sector_count = self.getFlashSectorCount(server_id)
        sector_starts = []
        sector_sizes = []

        for sector_num in range(sector_count):
            sector_starts.append(self.getFlashSectorStart(server_id, sector_num))
            sector_sizes.append(self.getFlashSectorSize(server_id, sector_num))

        return FlashSectorMap(sector_count, sector_starts, sector_sizes)

    def doTransaction(self, server_id, func_code, data):
        self.writeRequest(server_id, COMM_FLAG_FIRST_MESSAGE + COMM_FLAG_LAST_MESSAGE, func_code, data)
        return self.readResponse(server_id, func_code)

    # Pass in a lists of server id's, func codes and data packets.
    def doMultiTransaction(self, server_id, func_code, data):
        # First message flag
        flags = COMM_FLAG_SEND
        self.writeMultiRequest(server_id, flags, func_code, data) 

        responses = [0]*len(server_id)
        for i in range(len(responses)):
            responses[i] = self.readResponse(server_id[i], func_code[i])

        return responses

    def writeRequest(self, server_id, flags, func_code, data=''):
        if self._protocol >= 3:
            message = struct.pack('BBB', server_id, flags, func_code) + data
        else:
            message = struct.pack('BB', server_id, func_code) + data

        if self._protocol >= 2:
            prefixed_message = struct.pack('BBH', 0xFF, 0xFF, len(message)) + message
        else:
            prefixed_message = struct.pack('B', len(message)) + message

        crc = self._computeCRC(message)
        datagram = prefixed_message + struct.pack('<H', crc) 

        #print (":".join("{:02x}".format(ord(c)) for c in datagram))

        self._ser.write(datagram)

    def writeMultiRequest(self, server_id, flags, func_code, data=[]):
        datagram = ''
        for i in range(len(server_id)):
            if i == len(server_id)-1:
                flags = flags + COMM_FLAG_LAST_MESSAGE
            message = struct.pack('BBB', server_id[i], flags, func_code[i]) + data[i]
            prefixed_message = struct.pack('BBH', 0xFF, 0xFF, len(message)) + message
            flags = COMM_FLAG_SEND

            crc = self._computeCRC(message)
            datagram = datagram + prefixed_message + struct.pack('<H', crc) 

        print (":".join("{:02x}".format(ord(c)) for c in datagram))

        self._ser.write(datagram)


    def readResponse(self, server_id, func_code, num_tries=1, try_interval=0.01):
        if self._protocol >= 2:
            for i in range(num_tries):
                sync = self._ser.read()

                if len(sync) == 1:
                    break

                time.sleep(try_interval)
            else:
                # Reached maximum number of tries
                # self._ser.reset_input_buffer()
                print "max tries"
                return False, None

            version = self._ser.read()
            if len(version) != 1 or version != "\xff":
                # self._ser.reset_input_buffer()
                return False, None

            length = self._ser.read(2)
            if length == None or len(length) == 0:
                return False, None

            message_len, = struct.unpack('H', length)
            message = self._ser.read(message_len)
        else:
            for i in range(num_tries):
                lb = self._ser.read()

                if len(lb) == 1:
                    break

                time.sleep(try_interval)
            else:
                # Reached maximum number of tries
                # self._ser.reset_input_buffer()
                print "max tries"
                return False, None

            if lb == None or len(lb) == 0:
                print "no data"
                return False, None

            message_len, = struct.unpack('B', lb)
            message = self._ser.read(message_len)

        #print (":".join("{:02x}".format(ord(c)) for c in message))

        if len(message) < message_len:
            # self._ser.reset_input_buffer()
            print "not enough data"
            return False, None

        crc_bytes = self._ser.read(2)

        if len(crc_bytes) < 2:
            # self._ser.reset_input_buffer()
            print "crc not good"
            return False, None

        if self._protocol >= 3:
            #print (":".join("{:02x}".format(ord(c)) for c in message))
            #print (struct.unpack('<BBBH', message[:5]))
            message_server_id, flags, message_func_code, errors = struct.unpack('<BBBH', message[:5])
        else:
            # print (":".join("{:02x}".format(ord(c)) for c in message))
            # print(struct.unpack('<BBH', message[:4]))
            message_server_id, message_func_code, errors = struct.unpack('<BBH', message[:4])

        if message_server_id != server_id or message_func_code != func_code:
            #raise ProtocolError('received unexpected server ID or function code')
            print ('received unexpected server ID or function code')

        message_crc, = struct.unpack('<H', crc_bytes)
        computed_crc = self._computeCRC(message)

        #board_crc, = struct.unpack('<H', self._ser.read(2))
        #print "board crc:"
        #print(hex(board_crc))

        if message_crc != computed_crc: #+ lb):
            #raise ProtocolError('received unexpected CRC')
            print "crc error:"
            print(hex(message_crc))
            print(hex(computed_crc))

        success = (errors & COMM_ERRORS_OP_FAILED) == 0

        # Raise an exception if another type of error occurred
        # if (errors & ~COMM_ERRORS_OP_FAILED) != 0:
        #     raise ProtocolError('other error flags set', errors)

        if (errors & COMM_ERRORS_OP_FAILED) != 0:
        #     raise ProtocolError('operation failed')
            print "operation failed"

        if (errors & COMM_ERRORS_MALFORMED) != 0:
            raise ProtocolError('malformed request')

        if (errors & COMM_ERRORS_INVALID_FC) != 0:
            raise ProtocolError('invalid function code')

        if (errors & COMM_ERRORS_INVALID_ARGS) != 0:
            raise ProtocolError('invalid arguments')

        if (errors & COMM_ERRORS_BUF_LEN_MISMATCH) != 0:
            raise ProtocolError('buffer length mismatch')

        if self._protocol >= 3:
            return success, message[5:]
        else:
            return success, message[4:]

    def _computeCRC(self, values):
        crc = crcmod.predefined.Crc('crc-16')
        crc.update(values)
        return crc.crcValue
