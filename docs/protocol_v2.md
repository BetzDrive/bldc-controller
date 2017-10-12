## BLDC Servo Controller Binary Protocol Version 2 Proposal

**Request Message Format:**

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Function Code | Payload | CRC |
|--------------|------------------|-------------------------|----------------|----------|---------------|-----------------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |

**Response Message Format:**

|  | Sync Flag (`0xFF`) | Protocol Version (`0xFF`) | Message Length | Board ID | Errors | Values | CRC |
|--------------|------------------|-------------------------|----------------|----------|--------|--------|-----|
| **Size (bytes)** | 1 | 1 | 2 | 1 | 1 | n | 2 |
