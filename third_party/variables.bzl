chibios_stm32f4_port_inc_paths = [
    "chibios/os/ports/common/ARMCMx/CMSIS/include",
    "chibios/os/ports/common/ARMCMx",
    "chibios/os/ports/GCC/ARMCMx",
    "chibios/os/ports/GCC/ARMCMx/STM32F4xx",
]

chibios_stm32f4_platform_inc_paths = [
    "chibios/os/hal/platforms/STM32F4xx",
    "chibios/os/hal/platforms/STM32",
    "chibios/os/hal/platforms/STM32/GPIOv2",
    "chibios/os/hal/platforms/STM32/I2Cv1",
    "chibios/os/hal/platforms/STM32/OTGv1",
    "chibios/os/hal/platforms/STM32/RTCv2",
    "chibios/os/hal/platforms/STM32/SPIv1",
    "chibios/os/hal/platforms/STM32/TIMv1",
    "chibios/os/hal/platforms/STM32/USARTv1",
]

chibios_kernel_inc_paths = [
    "chibios/os/kernel/include",
]

chibios_hal_inc_paths = [
    "chibios/os/hal/include",
]

chibios_ext_stm32_inc_paths = [
    "chibios/ext/stm32lib/inc",
]

chibios_various_inc_paths = [
    "chibios/os/various",
]

chibios_inc_paths = (
    chibios_stm32f4_port_inc_paths +
    chibios_stm32f4_platform_inc_paths +
    chibios_kernel_inc_paths +
    chibios_hal_inc_paths +
    chibios_ext_stm32_inc_paths +
    chibios_various_inc_paths
)

project_inc_paths = [
    "-Ibootloader/include",  # This is for getting the configuration files
    "-Ifirmware/include",  # This is for getting the configuration files
    "-Iexternal/com_github_nanopb_nanopb",
    "-Icommon",  # This is for board.h
    "-Icommon/include",  # This is for common libs
]
