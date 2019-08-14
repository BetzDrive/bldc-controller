# BLDC Servo Controller Binary Protocol Version 3 Proposal

## Overview

#### Request Packet Format

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Flags | Packet Length | Sub-Messages | CRC |
|--------------|------------------|-------------------------|-------|---------------|----------------|-----|
| **Size (bytes)** | 1 | 1 | 1 | 2 | n | 2 |

The packet length is the total length of all sub-messages and the CRC.

#### Sub Message Format

|  | Message Length | Board ID | Function | Payload |
|--------------|--------------------|----------|---------------|-----------------|
| **Size (bytes)** | 2 | 1 | 1 | n |

The message length is the combined length of the board ID, flag, function code, and payload. The number of sub-messages is only limited by the input buffer on the boards receiving the packets. 

Setting the board ID to `0` will broadcast the request. All connected boards will listen to a broadcasted command and attempt to respond, so this should typically only be used with a single board connected.

#### Response Packet Format

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Flag | Packet Length | Sub Len | Board ID | Function | Errors | Payload | CRC |
|--------------|------------------|-------------------------|------|---------------|---------|----------|----------|--------|--------|-----|
| **Size (bytes)** | 1 | 1 | 1 | 2 | 2 | 1 | 1 | 2 | n | 2 |

The message length is the combined length of the board ID, function code, errors, and payload.


-------

## CRC

Polynomial: CRC-16-IBM (x^16 + x^15 + x^2 + 1).

The CRC is computed over the entire packet excluding the CRC itself.

## Function Codes

| Function | Name | Description | Request Payload | Response Payload |
|--------|-----------------------------|-------------------------------------------------------|-----------|---|
| `0x00` | `COMM_FC_NOP` | Do nothing | N/A | N/A |
| `0x01` | `COMM_FC_REG_READ` | Read registers | Start address (`uint16_t`), register count (`uint8_t`) | Register values (`void *`) |
| `0x02` | `COMM_FC_REG_WRITE` | Write registers | Start address (`uint16_t`), register count (`uint8_t`), register values (`void *`) | N/A |
| `0x03` | `COMM_FC_REG_READ_WRITE` | Read and write registers simultaneously | Read start address (`uint16_t`), read register count (`uint8_t`), write start address (`uint16_t`), write register count (`uint8_t`), write register values (`void *`) | Read register values (`void *`) |
| `0x10` | `COMM_FC_CLEAR_IWDGRST` | Clear the independent watchdog reset flag | N/A | N/A |
| `0x80` | `COMM_FC_SYSTEM_RESET` | Enter the bootloader | N/A | N/A |
| `0x81` | `COMM_FC_JUMP_TO_ADDR` | Jump to an address and execute code (used to leave bootloader) | Jump address (`uint32_t`) | N/A |
| `0x82` | `COMM_FC_FLASH_SECTOR_COUNT` | Get the number of flash sectors | N/A | Flash sector count (`uint32_t`) |
| `0x83` | `COMM_FC_FLASH_SECTOR_START` | Get the start address of a flash sector | Flash sector number (`uint32_t`) | Start address (`uint32_t`) |
| `0x84` | `COMM_FC_FLASH_SECTOR_SIZE` | Get the size of a flash sector | Flash sector number (`uint32_t`) | Flash sector size (`uint32_t`) |
| `0x85` | `COMM_FC_FLASH_SECTOR_ERASE` | Erase a flash sector | Flash sector number (`uint32_t`) | N/A |
| `0x86` | `COMM_FC_FLASH_PROGRAM` | Program flash memory (must be erased first) | Start address (`uint32_t`), memory values (`void *`) | N/A |
| `0x87` | `COMM_FC_FLASH_READ` | Read memory values | Start address (`uint32_t`), length (`uint32_t`) | Memory values (`void *`) |
| `0x88` | `COMM_FC_FLASH_VERIFY` | Verify memory values | Start address (`uint32_t`), memory values (`void *`) | N/A |
| `0x89` | `COMM_FC_FLASH_VERIFY_ERASED` | Verify that flash memory is erased (all `0xff`) | Start address (`uint32_t`), length (`uint32_t`) | N/A |
| `0xFE` | `COMM_FC_CONFIRM_ID` | Confirm ID following enumeration |  |  |
| `0xFF` | `COMM_FC_ENUMERATE` | Set next ID board in Disco Bus |  |  |

The errors field in the response packet will be zero if an operation succeeded.

-------

## Registers

All values that can be read from or sent to the board are abstracted into virtual registers.
These are used for things like:
- Sending commands (current, duty cycle, etc)
- Setting calibration values
- Reading from sensors (encoder position/velocity, accelerometer data, etc)
- Checking firmware version number
- Re-assigning board ID

### Reading/Writing
Registers can be read from and written to using the `COMM_FC_REG_READ_WRITE` command. The payload of the function request has five distinct fields:

|  | Read Start Address | Read Count | Write Start Address | Write Count | Write Values |
|------|------------|--------------------|-------------|---------------------|--------------|
| **Size (bytes)** | 2 | 1 | 2 | 1 | *n* |

By incrementing the *Read Count* and *Write Count* values, registers with consecutive addresses can be read from and written to in batches. Their values should be concatenated together in `Write Values` (for writes), as well as in the response payload (for reads).

The standalone `COMM_FC_REG_READ` and `COMM_FC_REG_WRITE` commands can be used in the same way, with the read fields omitted for writes and write fields omitted for reads.

### Register List

(WIP)

**System Registers `(0x0***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x0000` | Register Map Version | `uint16_t` |
| `0x0001` | Board ID | `uint8_t` |
| `0x0002` | Firmware Version | `uint16_t` |
| `0x0003` | Bootloader Version | `uint16_t` |
| `0x0004` | Store Calibration | `N/A` |
| `0x0005` | Clear Calibration | `N/A` |

**Calibration Registers `(0x1***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x1000` | Electrical Revolution Start | `uint16_t` |
| `0x1001` | Electrical Revolutions Per Mechanical Revolution | `uint8_t` |
| `0x1002` | Invert Phases | `uint8_t` |
| `0x1003` | Direct Current Controller Kp | `float` |
| `0x1004` | Direct Current Controller Ki | `float` |
| `0x1005` | Quadrature Current Controller Kp | `float` |
| `0x1006` | Quadrature Current Controller Ki | `float` |
| `0x1007` | Velocity Controller Kp | `float` |
| `0x1008` | Velocity Controller Kd | `float` |
| `0x1009` | Position Controller Kp | `float` |
| `0x100A` | Position Controller Kd | `float` |
| `0x1010` | Current Limit (A) | `float` |
| `0x1011` | Torque Limit (N*m) | `float` |
| `0x1012` | Velocity Limit (rad/s) | `float` |
| `0x1013` | Position Lower Limit (rad) | `float` |
| `0x1014` | Position Upper Limit (rad) | `float` |
| `0x1015` | Position Offset (rad) | `float` |
| `0x1020` | Motor Resistance (ohm) | `float` |
| `0x1021` | Motor Inductance (H) | `float` |
| `0x1022` | Motor Torque Constant (N*m/A) | `float` |
| `0x1030` | Control Watchdog Timeout (ms) | `uint16_t` |
| `0x1040` | Velocity Filter Parameter | `float` |

**Volatile Registers `(0x2***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x2000` | Control Mode | `uint8_t` |
| `0x2001` | Direct Current Command (A) | `float` |
| `0x2002` | Quadrature Current Command (A) | `float` |
| `0x2003` | Phase A Raw PWM Duty Cycle | `float` |
| `0x2004` | Phase B Raw PWM Duty Cycle | `float` |
| `0x2005` | Phase C Raw PWM Duty Cycle | `float` |
| `0x2006` | Torque Setpoint (N*m) | `float` |
| `0x2007` | Velocity Setpoint (rad/s) | `float` |
| `0x2008` | Position Setpoint (rad) | `float` |
| `0x2009` | Feed-Forward Setpoint (A) | `float` |
| `0x200A` | PWM Drive (V) | `float` |

**Read Only Registers `(0x3***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x3000` | Rotor Position (rad) | `float` |
| `0x3001` | Rotor Velocity (rad/sec) | `float` |
| `0x3002` | Direct Current Measurement (A) | `float` |
| `0x3003` | Quadrature Current Measurement (A) | `float` |
| `0x3004` | DC Supply Voltage (V) | `float` |
| `0x3005` | Board Temperature (°C) | `float` |
| `0x3006` | Accelerometer X (milli-g) | `int32_t` |
| `0x3007` | Accelerometer Y (milli-g) | `int32_t` |
| `0x3008` | Accelerometer Z (milli-g) | `int32_t` |
| `0x3009` | Recorder start | `bool` |
| `0x300A` | Recorder ready/length | `uint16_t` |
| `0x300B` | Recorder reset | `bool` |
| `0x3010` | Rotor Position (raw) | `uint16_t` |
| `0x3011` | Rotor Revolutions (count) | `uint16_t` |

-------

## Control Modes

| ID | Description | Relevant Registers |
|----|-------------|--------------------|
| 0 | FOC Current Control | Direct Current Command, Quadrature Current Command |
| 1 | Raw Phase PWM | Phase A/B/C Raw PWM Duty Cycle |
| 2 | Torque Control | Torque Setpoint |
| 3 | Velocity Control | Velocity Setpoint |
| 4 | Position Control | Position Setpoint |
| 5 | Position Velocity Control | Position Velocity Setpoints |
| 6 | Postion Feed-Forward | Position Feed-Forward Setpoints |
| 7 | PWM Drive | PWM Drive |

-------

## Flag Bits

| Bit | Description |
|-----|-------------|
| 0 | Packet Send/Receive (1 if packet coming from boards, 0 if from control) |
| 1 | Independent Watchdog Reset (1 if board has reset, 0 if no reset) |
| 2-7 | Unused |

-------

## Errors

(error codes)
