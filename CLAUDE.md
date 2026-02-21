# BLDC Controller - Claude Code Guidelines

## Project Overview
STM32F405-based BLDC motor controller with FOC control. Two firmware variants:
- `firmware/` - ChibiOS RTOS-based (original)
- `firmware_baremetal/` - Baremetal cooperative loop (new)

Both share the same bootloader, protocol, and flash memory layout.

## Build System
Bazel with ARM GCC cross-compilation via bazel-embedded.

```bash
# Build firmware
bazel build //firmware_baremetal:firmware_baremetal

# Run host-side unit tests
bazel test //firmware_baremetal:comms_test
bazel test //firmware_baremetal:control_test
bazel test //firmware_baremetal:state_test
bazel test //firmware_baremetal:led_test

# Flash via ST-LINK
bazel run //firmware_baremetal:flash

# Flash via RS485 bootloader
bazel run //firmware_baremetal:upload
```

## Architecture: Testability by Design

### HAL Abstraction Layer
All application logic is separated from hardware through abstract C interfaces in `firmware_baremetal/include/hal/`:

```
hal_gpio.h   - GPIO read/write
hal_uart.h   - UART with circular buffer semantics
hal_spi.h    - SPI transfers (encoder)
hal_i2c.h    - I2C transactions (sensors)
hal_pwm.h    - Motor and LED PWM
hal_adc.h    - ADC current/voltage sampling
hal_flash.h  - Flash read/write/erase
hal_timer.h  - Microsecond/millisecond timing
hal_iwdg.h   - Independent watchdog
```

### Build Targets
- **`:hal_interface`** - Abstract HAL headers (no implementation)
- **`:logic`** - All application logic (comms, control, state, sensors, LED). Depends only on `hal_interface`. **Fully unit testable on host.**
- **`:hal_stm32`** - STM32F405 register-level HAL implementations. **Not unit tested** (requires hardware).
- **`:hal_mock`** - Mock HAL implementations for tests. `testonly = True`.
- **`:firmware_baremetal`** - Final firmware binary linking `logic` + `hal_stm32`.

### Unit Testing & Code Coverage
- All logic in `:logic` must have unit test coverage
- Tests link against `:hal_mock` instead of `:hal_stm32`
- Only STM32-specific HAL implementations (register access, DMA, ISRs) are excluded from coverage
- The comms protocol FSM, FOC control math, state management, LED state machine, and sensor polling logic are all testable on the host
- Test files live in `firmware_baremetal/test/`

## Permissions
Claude has full permissions to read, write, and execute anything in this directory.
