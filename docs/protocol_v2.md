# BLDC Servo Controller Binary Protocol Version 2 Proposal

## Overview

#### Request Message Format:

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Function | Payload | CRC* |
|--------------|------------------|-------------------------|----------------|----------|---------------|-----------------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |

#### Response Message Format:

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Errors | Payload | CRC* |
|--------------|------------------|-------------------------|----------------|----------|--------|--------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |

*not yet implemented

-------


## Functions

| Function | Name | Description | Request Payload | Response Payload |
|--------|-----------------------------|-------------------------------------------------------|-----------|---|
| `0x00` | `COMM_FC_NOP` |  |  |  |
| `0x01` | `COMM_FC_REG_RW` | Simultaneous register read/write. (see below) |  |  |
| `0x80` | `COMM_FC_SYSTEM_RESET` |  |  |  |
| `0x81` | `COMM_FC_JUMP_TO_ADDR` |  |  |  |
| `0x82` | `COMM_FC_FLASH_SECTOR_COUNT` |  |  |  |
| `0x83` | `COMM_FC_FLASH_SECTOR_START` |  |  |  |
| `0x84` | `COMM_FC_FLASH_SECTOR_SIZE` |  |  |  |
| `0x85` | `COMM_FC_FLASH_SECTOR_ERASE` |  |  |  |
| `0x86` | `COMM_FC_FLASH_PROGRAM` |  |  |  |
| `0x87` | `COMM_FC_FLASH_READ` |  |  |  |
| `0x88` | `COMM_FC_FLASH_VERIFY` |  |  |  |
| `0x89` | `COMM_FC_FLASH_VERIFY_ERASED` |  |  |  |


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
Registers can be read from and written to using the `COMM_FC_REG_RW` command. The payload of the function request has five distinct fields:

|  | Read Start Address | Read Count | Write Start Address | Write Count | Write Values |
|------|------------|--------------------|-------------|---------------------|--------------|
| **Size (bytes)** | 2 | 1 | 2 | 1 | *n* |

By incrementing the *Read Count* and *Write Count* values, registers with consecutive addresses can be read from and written to in batches. Their values should be concatenated together in `Write Values` (for writes), as well as in the response payload (for reads).

### Register List

(WIP)

**System Registers `(0x0***)`**

| Address | Description | Type |
|----------|---------------------------|---------|
| `0x0000`| Firmware Version | `uint16_t` |
| `0x0001`| Bootloader Version | `uint16_t` |

**Calibration Registers `(0x1***)`**

| Address | Description | Type |
|----------|---------------------------|---------|
| `0x1000` | Phase A Encoder Angle (radians) | `float` |
| `0x1001`| Invert Phases | `bool` |
| `0x1002`| FOC Kp (d) | `float` |
| `0x1003`| FOC Ki (d) | `float` |
| `0x1004`| FOC Kp (q) | `float` |
| `0x1005`| FOC Ki (q) | `float` |

**Volatile Registers `(0x2***)`**

| Address | Description | Type |
|----------|---------------------------|---------|
| `0x2000` | Mode | `uint8_t` |
| `0x2001` | Current Command | `float` |
| `0x2002` | Raw PWM A | `float` |
| `0x2003` | Raw PWM B | `float` |
| `0x2004` | Raw PWM C | `float` |

**Read Only Registers `(0x3***)`**

| Address | Description | Type |
|----------|---------------------------|---------|
| `0x3000` | Rotor Position (radians) | `float` |
| `0x3001` | Rotor Velocity (rad/sec) | `float` |
| `0x3002` | Motor Current (amps) | `float` |
| `0x3003` | Battery Current (amps) | `float` |
| `0x3004` | Accelerometer X (m/sec^2) | `float` |
| `0x3005` | Accelerometer Y (m/sec^2) | `float` |
| `0x3006` | Accelerometer Z (m/sec^2) | `float` |

-------

## Errors

(error codes)
