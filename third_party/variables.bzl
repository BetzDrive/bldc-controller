chibios_stm32f4_port_inc_paths = [
    "-Ithird_party/chibios/os/ports/common/ARMCMx/CMSIS/include",
    "-Ithird_party/chibios/os/ports/common/ARMCMx",
    "-Ithird_party/chibios/os/ports/GCC/ARMCMx",
    "-Ithird_party/chibios/os/ports/GCC/ARMCMx/STM32F4xx",
]

chibios_stm32f4_platform_inc_paths = [
    "-Ithird_party/chibios/os/hal/platforms/STM32F4xx",
    "-Ithird_party/chibios/os/hal/platforms/STM32",
    "-Ithird_party/chibios/os/hal/platforms/STM32/GPIOv2",
    "-Ithird_party/chibios/os/hal/platforms/STM32/I2Cv1",
    "-Ithird_party/chibios/os/hal/platforms/STM32/OTGv1",
    "-Ithird_party/chibios/os/hal/platforms/STM32/RTCv2",
    "-Ithird_party/chibios/os/hal/platforms/STM32/SPIv1",
    "-Ithird_party/chibios/os/hal/platforms/STM32/TIMv1",
    "-Ithird_party/chibios/os/hal/platforms/STM32/USARTv1",
]

chibios_kernel_inc_paths = [
    "-Ithird_party/chibios/os/kernel/include",
]

chibios_hal_inc_paths = [
    "-Ithird_party/chibios/os/hal/include",
    "-Ibootloader/include",  # This is for getting the configuration files
    "-Icommon",  # This is for board.h
]

chibios_ext_stm32_inc_paths = [
    "-Ithird_party/chibios/ext/stm32lib/inc",
]

chibios_inc_paths = (
    chibios_stm32f4_port_inc_paths +
    chibios_stm32f4_platform_inc_paths +
    chibios_kernel_inc_paths +
    chibios_hal_inc_paths +
    chibios_ext_stm32_inc_paths
)
