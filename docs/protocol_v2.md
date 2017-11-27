# BLDC Servo Controller Binary Protocol Version 2 Proposal

## Overview

#### Request Packet Format

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Function | Payload | CRC |
|--------------|------------------|-------------------------|----------------|----------|---------------|-----------------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |

The message length is the combined length of the board ID, function code, and payload.

Setting the board ID to `0` will broadcast the request. All connected boards will listen to a broadcasted command and attempt to respond, so this should typically only be used with a single board connected.

#### Response Packet Format

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Function | Errors | Payload | CRC |
|--------------|------------------|-------------------------|----------------|----------|----------|--------|--------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | 2 | n | 2 |

The message length is the combined length of the board ID, function code, errors, and payload.

-------

## CRC

Polynomial: CRC-16-IBM (x^16 + x^15 + x^2 + 1).

The CRC is computed over the entire packet excluding the CRC itself.

## Function Codes

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

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x0000` | Register Map Version | `uint16_t` |
| `0x0001` | Board ID | `uint8_t` |
| `0x0002` | Firmware Version | `uint16_t` |
| `0x0003` | Bootloader Version | `uint16_t` |

**Calibration Registers `(0x1***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x1000` | Phase A Encoder Angle (rad) | `float` |
| `0x1001` | Electrical Revolutions Per Mechanical Revolution | `uint8_t` |
| `0x1002` | Invert Phases | `uint8_t` |
| `0x1003` | Direct Current Controller Kp | `float` |
| `0x1004` | Direct Current Controller Ki | `float` |
| `0x1005` | Quadrature Current Controller Kp | `float` |
| `0x1006` | Quadrature Current Controller Ki | `float` |

**Volatile Registers `(0x2***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x2000` | Control Mode | `uint8_t` |
| `0x2001` | Direct Current Command (A) | `float` |
| `0x2002` | Quadrature Current Command (A) | `float` |
| `0x2003` | Phase A Raw PWM Duty Cycle | `float` |
| `0x2004` | Phase B Raw PWM Duty Cycle | `float` |
| `0x2005` | Phase C Raw PWM Duty Cycle | `float` |

**Read Only Registers `(0x3***)`**

| Address  | Description | Type |
|----------|---------------------------|---------|
| `0x3000` | Rotor Position (rad) | `float` |
| `0x3001` | Rotor Velocity (rad/sec) | `float` |
| `0x3002` | Direct Current Measurement (A) | `float` |
| `0x3003` | Quadrature Current Measurement (A) | `float` |
| `0x3004` | DC Supply Voltage (V) | `float` |
| `0x3005` | Board Temperature (°C) | `float` |
| `0x3006` | Accelerometer X (m/s^2) | `float` |
| `0x3007` | Accelerometer Y (m/s^2) | `float` |
| `0x3008` | Accelerometer Z (m/s^2) | `float` |
| `0x3009` | Gyroscope X (rad/s) | `float` |
| `0x300a` | Gyroscope Y (rad/s) | `float` |
| `0x300b` | Gyroscope Z (rad/s) | `float` |


-------

## Errors

(error codes)
