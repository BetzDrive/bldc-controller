# BLDC Servo Controller Binary Protocol Version 2 Proposal

## Overview

#### Request Message Format:

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Function | Payload | CRC |
|--------------|------------------|-------------------------|----------------|----------|---------------|-----------------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |

#### Response Message Format:

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Errors | Payload | CRC* |
|--------------|------------------|-------------------------|----------------|----------|--------|--------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |

*not yet implemented -- currently filled with zeros

-------


## Functions

| Function | Name | Description | Request Payload | Response Payload |
|--------|-----------------------------|-------------------------------------------------------|-----------|---|
| `0x00` | `COMM_FC_NOP` |  |  |  |
| `0x01` | `COMM_FC_REG_RW` | Simultaneous register read/write. (see below) |  |  |
| `0x80` | `COMM_FC_SYSTEM_RESET` | System reset. Must be done on boot to start firmware. |  |  |
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

### Manipulation
Registers can be read from and written to using the `COMM_FC_REG_RW` command. The payload of the function request has five distinct fields:

|  | Read Start Address | Read Count | Write Start Address | Write Count | Write Values |
|------|------------|--------------------|-------------|---------------------|--------------|
| **Size (bytes)** | 2 | 1 | 2 | 1 | *n* |

By incrementing the *Read Count* and *Write Count* values, registers with consecutive addresses can be read from and written to in batches. Their values should be concatenated together in `Write Values` (for writes), as well as in the response payload (for reads).

### Addresses

(register list)


-------

## Errors

(error codes)
