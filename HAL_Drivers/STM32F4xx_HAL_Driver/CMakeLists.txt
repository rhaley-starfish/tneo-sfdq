cmake_minimum_required(VERSION 3.19)
project(STM32F4xx_HAL_Driver C ASM)
set(CMAKE_C_STANDARD 11)


LIST(APPEND STM_HAL_SRC
    Src/stm32f4xx_hal_tim.c
    Src/stm32f4xx_hal_spi.c
    Src/stm32f4xx_hal_ltdc_ex.c
    Src/stm32f4xx_hal_fmpi2c.c
    Src/stm32f4xx_hal_fmpsmbus_ex.c
    Src/stm32f4xx_hal_rtc.c
    Src/stm32f4xx_hal_sd.c
    Src/stm32f4xx_hal_flash.c
    Src/stm32f4xx_hal_fmpi2c_ex.c
    Src/stm32f4xx_hal_iwdg.c
    Src/stm32f4xx_hal_pcd.c
    Src/stm32f4xx_hal_uart.c
#    Src/stm32f4xx_hal_timebase_tim_template.c
    Src/stm32f4xx_hal_adc_ex.c
    Src/stm32f4xx_hal_nor.c
    Src/stm32f4xx_hal_exti.c
    Src/stm32f4xx_hal_sai_ex.c
    Src/stm32f4xx_hal_wwdg.c
    Src/stm32f4xx_hal_i2c.c
    Src/stm32f4xx_hal_flash_ex.c
    Src/stm32f4xx_hal_smbus.c
    Src/stm32f4xx_hal_lptim.c
    Src/stm32f4xx_hal_dcmi_ex.c
    Src/stm32f4xx_hal_nand.c
    Src/stm32f4xx_hal_i2s_ex.c
    Src/stm32f4xx_hal_pccard.c
    Src/stm32f4xx_hal_rcc.c
    Src/stm32f4xx_hal_rtc_ex.c
#    Src/stm32f4xx_hal_timebase_rtc_alarm_template.c
    Src/stm32f4xx_hal_rcc_ex.c
    Src/stm32f4xx_hal_pwr.c
    Src/stm32f4xx_hal_irda.c
    Src/stm32f4xx_hal_cryp_ex.c
    Src/stm32f4xx_hal_cortex.c
    Src/stm32f4xx_hal_smartcard.c
    Src/stm32f4xx_hal_dma_ex.c
    Src/stm32f4xx_hal_pwr_ex.c
    Src/stm32f4xx_hal_cec.c
    Src/stm32f4xx_hal_ltdc.c
    Src/stm32f4xx_hal_cryp.c
    Src/stm32f4xx_hal_sai.c
    Src/stm32f4xx_hal_dac_ex.c
    Src/stm32f4xx_hal_dac.c
    Src/stm32f4xx_hal_usart.c
    Src/stm32f4xx_hal_dma.c
#    Src/stm32f4xx_hal_msp_template.c
    Src/stm32f4xx_hal_dcmi.c
    Src/stm32f4xx_hal_adc.c
    Src/stm32f4xx_hal_hash.c
#    Src/stm32f4xx_hal_timebase_rtc_wakeup_template.c
    Src/stm32f4xx_hal_spdifrx.c
    Src/stm32f4xx_hal_qspi.c
    Src/stm32f4xx_hal_i2c_ex.c
    Src/stm32f4xx_hal_dma2d.c
    Src/stm32f4xx_hal_sram.c
    Src/stm32f4xx_hal.c
    Src/stm32f4xx_hal_flash_ramfunc.c
    Src/stm32f4xx_hal_eth.c
    Src/stm32f4xx_hal_hcd.c
    Src/stm32f4xx_hal_rng.c
    Src/stm32f4xx_hal_hash_ex.c
    Src/stm32f4xx_hal_can.c
    Src/stm32f4xx_hal_tim_ex.c
    Src/stm32f4xx_hal_pcd_ex.c
    Src/stm32f4xx_hal_mmc.c
    Src/stm32f4xx_hal_i2s.c
    Src/stm32f4xx_hal_sdram.c
    Src/stm32f4xx_hal_dsi.c
    Src/stm32f4xx_hal_crc.c
    Src/stm32f4xx_hal_dfsdm.c
    Src/stm32f4xx_hal_fmpsmbus.c
    Src/stm32f4xx_hal_gpio.c
        )

add_library(${PROJECT_NAME} ${STM_HAL_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
        Inc
        ../CMSIS/Device/ST/STM32F4xx/Include/
        ../CMSIS/Include/
        ../../inc/
        )

target_compile_definitions(${PROJECT_NAME} PUBLIC
        STM32F446xx
        STM32F446RETx
        STM32F4
        USE_FULL_ASSERT
        USE_STDPERIPH_DRIVER
        HSE_VALUE=20000000
        #        DEBUG
        )

target_compile_options(${PROJECT_NAME} PUBLIC
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -fdata-sections
        -ffunction-sections
        -Wall
        )

target_link_options(${PROJECT_NAME} PUBLIC
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )