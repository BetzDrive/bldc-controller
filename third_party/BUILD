load("@rules_cc//cc:defs.bzl", "cc_library")
load(":variables.bzl", "chibios_inc_paths")

chibios_stm32f4_port_srcs = [
    "chibios/os/ports/GCC/ARMCMx/crt0.c",
    "chibios/os/ports/GCC/ARMCMx/STM32F4xx/vectors.c",
    "chibios/os/ports/GCC/ARMCMx/chcore.c",
    "chibios/os/ports/GCC/ARMCMx/chcore_v7m.c",
    "chibios/os/ports/common/ARMCMx/nvic.c",
]

chibios_stm32f4_port_incs = (
    glob(["chibios/os/ports/common/ARMCMx/CMSIS/include/*.h"]) + glob([
        "chibios/os/ports/common/ARMCMx/*.h",
    ]) + glob([
        "chibios/os/ports/GCC/ARMCMx/*.h",
    ]) + glob([
        "chibios/os/ports/GCC/ARMCMx/STM32F4xx/*.h",
    ])
)

chibios_stm32f4_platform_srcs = [
    "chibios/os/hal/platforms/STM32F4xx/stm32_dma.c",
    "chibios/os/hal/platforms/STM32F4xx/hal_lld.c",
    "chibios/os/hal/platforms/STM32F4xx/adc_lld.c",
    "chibios/os/hal/platforms/STM32F4xx/ext_lld_isr.c",
    "chibios/os/hal/platforms/STM32/can_lld.c",
    "chibios/os/hal/platforms/STM32/ext_lld.c",
    "chibios/os/hal/platforms/STM32/mac_lld.c",
    "chibios/os/hal/platforms/STM32/sdc_lld.c",
    "chibios/os/hal/platforms/STM32/GPIOv2/pal_lld.c",
    "chibios/os/hal/platforms/STM32/I2Cv1/i2c_lld.c",
    "chibios/os/hal/platforms/STM32/OTGv1/usb_lld.c",
    "chibios/os/hal/platforms/STM32/RTCv2/rtc_lld.c",
    "chibios/os/hal/platforms/STM32/SPIv1/spi_lld.c",
    "chibios/os/hal/platforms/STM32/TIMv1/gpt_lld.c",
    "chibios/os/hal/platforms/STM32/TIMv1/icu_lld.c",
    "chibios/os/hal/platforms/STM32/TIMv1/pwm_lld.c",
    "chibios/os/hal/platforms/STM32/USARTv1/serial_lld.c",
    "chibios/os/hal/platforms/STM32/USARTv1/uart_lld.c",
]

chibios_stm32f4_platform_incs = (
    glob(["chibios/os/hal/platforms/STM32F4xx/*.h"]) + glob([
        "chibios/os/hal/platforms/STM32/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/GPIOv2/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/I2Cv1/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/OTGv1/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/RTCv2/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/SPIv1/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/TIMv1/*.h",
    ]) + glob([
        "chibios/os/hal/platforms/STM32/USARTv1/*.h",
    ])
)

chibios_kernel_srcs = [
    "chibios/os/kernel/src/chsys.c",
    "chibios/os/kernel/src/chdebug.c",
    "chibios/os/kernel/src/chlists.c",
    "chibios/os/kernel/src/chvt.c",
    "chibios/os/kernel/src/chschd.c",
    "chibios/os/kernel/src/chthreads.c",
    "chibios/os/kernel/src/chdynamic.c",
    "chibios/os/kernel/src/chregistry.c",
    "chibios/os/kernel/src/chsem.c",
    "chibios/os/kernel/src/chmtx.c",
    "chibios/os/kernel/src/chcond.c",
    "chibios/os/kernel/src/chevents.c",
    "chibios/os/kernel/src/chmsg.c",
    "chibios/os/kernel/src/chmboxes.c",
    "chibios/os/kernel/src/chqueues.c",
    "chibios/os/kernel/src/chmemcore.c",
    "chibios/os/kernel/src/chheap.c",
    "chibios/os/kernel/src/chmempools.c",
]

chibios_kernel_incs = glob(["chibios/os/kernel/include/*.h"])

chibios_ext_stm32_srcs = [
    "chibios/ext/stm32lib/src/misc.c",
    "chibios/ext/stm32lib/src/stm32f4xx_dma.c",
    "chibios/ext/stm32lib/src/stm32f4xx_rcc.c",
    "chibios/ext/stm32lib/src/stm32f4xx_adc.c",
    "chibios/ext/stm32lib/src/stm32f4xx_exti.c",
    "chibios/ext/stm32lib/src/stm32f4xx_rng.c",
    "chibios/ext/stm32lib/src/stm32f4xx_flash.c",
    "chibios/ext/stm32lib/src/stm32f4xx_rtc.c",
    "chibios/ext/stm32lib/src/stm32f4xx_crc.c",
    "chibios/ext/stm32lib/src/stm32f4xx_fsmc.c",
    "chibios/ext/stm32lib/src/stm32f4xx_sdio.c",
    "chibios/ext/stm32lib/src/stm32f4xx_cryp_aes.c",
    "chibios/ext/stm32lib/src/stm32f4xx_spi.c",
    "chibios/ext/stm32lib/src/stm32f4xx_cryp.c",
    "chibios/ext/stm32lib/src/stm32f4xx_hash.c",
    "chibios/ext/stm32lib/src/stm32f4xx_syscfg.c",
    "chibios/ext/stm32lib/src/stm32f4xx_cryp_des.c",
    "chibios/ext/stm32lib/src/stm32f4xx_hash_md5.c",
    "chibios/ext/stm32lib/src/stm32f4xx_tim.c",
    "chibios/ext/stm32lib/src/stm32f4xx_cryp_tdes.c",
    "chibios/ext/stm32lib/src/stm32f4xx_hash_sha1.c",
    "chibios/ext/stm32lib/src/stm32f4xx_usart.c",
    "chibios/ext/stm32lib/src/stm32f4xx_dac.c",
    "chibios/ext/stm32lib/src/stm32f4xx_i2c.c",
    "chibios/ext/stm32lib/src/stm32f4xx_wwdg.c",
    "chibios/ext/stm32lib/src/stm32f4xx_dbgmcu.c",
    "chibios/ext/stm32lib/src/stm32f4xx_iwdg.c",
    "chibios/ext/stm32lib/src/stm32f4xx_dcmi.c",
    "chibios/ext/stm32lib/src/stm32f4xx_pwr.c",
    #"chibios/ext/stm32lib/src/stm32f4xx_gpio.c",
]

chibios_ext_stm32_incs = glob(["chibios/ext/stm32lib/inc/*.h"])

chibios_srcs = (
    chibios_kernel_srcs +
    chibios_stm32f4_port_srcs +
    chibios_stm32f4_platform_srcs +
    chibios_ext_stm32_srcs
)

chibios_hdrs = (
    chibios_stm32f4_port_incs +
    chibios_stm32f4_platform_incs +
    chibios_kernel_incs +
    chibios_ext_stm32_incs
)

cc_library(
    name = "chibios_hal",
    srcs = [
        "chibios/os/hal/src/hal.c",
        "chibios/os/hal/src/adc.c",
        "chibios/os/hal/src/can.c",
        "chibios/os/hal/src/ext.c",
        "chibios/os/hal/src/gpt.c",
        "chibios/os/hal/src/i2c.c",
        "chibios/os/hal/src/icu.c",
        "chibios/os/hal/src/mac.c",
        "chibios/os/hal/src/mmc_spi.c",
        "chibios/os/hal/src/mmcsd.c",
        "chibios/os/hal/src/pal.c",
        "chibios/os/hal/src/pwm.c",
        "chibios/os/hal/src/rtc.c",
        "chibios/os/hal/src/sdc.c",
        "chibios/os/hal/src/serial.c",
        "chibios/os/hal/src/serial_usb.c",
        "chibios/os/hal/src/spi.c",
        "chibios/os/hal/src/tm.c",
        "chibios/os/hal/src/uart.c",
        "chibios/os/hal/src/usb.c",
        "//common:board_config_srcs",
    ] + chibios_srcs,
    hdrs = (
        glob(["chibios/os/hal/include/*.h"]) +
        chibios_hdrs + [
            "//common:board_config_hdrs",
        ]
    ),
    copts = [
        "-DUSE_STDPERIPH_DRIVER",
    ] + chibios_inc_paths,
    visibility = ["//visibility:public"],
    deps = [
        "//bootloader:os_config",
    ],
)
