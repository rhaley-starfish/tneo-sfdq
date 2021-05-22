cmake_minimum_required(VERSION 3.19)
project(tneo_test C ASM)
set(CMAKE_C_STANDARD 11)
add_subdirectory(sfdq_firmware)
add_subdirectory(tneo)
#add_subdirectory(Drivers/STM32F4xx_HAL_Driver)
add_subdirectory(lua-5.4.3)

add_executable(tneo_test
#        src/main.c
        src/sfdq-main.c
        src/os-tasks.c
        buddymem/emm.c
        fft-c/libfft/fft.c
        src/task_consumer.c
#        src/task_producer.c
#        HAL_Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c
        src/system_stm32f4xx.c
        src/startup_stm32f446xx.s
        )
target_include_directories(${PROJECT_NAME}
        PUBLIC
        inc/
        buddymem/
        fft-c/libfft/
        fft-c/libcomplex/
        lua-5.4.3/src/
        tneo/src/
        tneo/src/core
        sfdq_firmware/inc
        sfdq_firmware/CMSIS
        sfdq_firmware/CMSIS/device
        sfdq_firmware/CMSIS/core
        sfdq_firmware/StdPeriph_Driver/inc
#        Drivers/BSP/STM32F4xx-Nucleo/
#        Drivers/CMSIS/Include
#        Drivers/CMSIS/Device/ST/STM32F4xx/Include
#        Drivers/STM32F4xx_HAL_Driver/Inc
        )

target_compile_definitions(${PROJECT_NAME}
        PUBLIC
        STM32F446xx
        STM32F446RETx
        STM32F4
        USE_FULL_ASSERT
        USE_STDPERIPH_DRIVER
        HSE_VALUE=20000000
        )

target_compile_options(${PROJECT_NAME} PUBLIC
        -Wno-unknown-pragmas # GCC doesn't like the pragmas used to turn off endless loop warnings in CLion.
        -mcpu=cortex-m4
        -mthumb
        -mfloat-abi=hard
        -mfpu=fpv4-sp-d16
        -fdata-sections
        -ffunction-sections
        -Wall
        -Os
        -g
        )


target_link_libraries(${PROJECT_NAME} PUBLIC
        tneo
        lua_static
        sfdq
        cmsis
        stm32StdPeriph
#        STM32F4xx_HAL_Driver
        )
# Only when cross-compiling, i.e. for hardware build
target_link_options(${PROJECT_NAME} PRIVATE
        -T../src/STM32F446RETx_FLASH.ld
        -specs=nosys.specs
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )



# Create gitInfo.h among other things as part of the cross-compile pre-build step.
add_custom_command(TARGET ${PROJECT_NAME}
        PRE_BUILD
        COMMAND ${CMAKE_PREBUILD_CMD}
        )

# Print compiled and linked executable size
add_custom_command(TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_SIZE_UTIL} ${PROJECT_NAME}
        )

set(MAP_FILE ${PROJECT_NAME}.map)
set(HEX_FILE ${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_NAME}.bin)
set(LST_FILE ${PROJECT_NAME}.lst)

# Create hex, bin, and lst files
add_custom_command(TARGET ${PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME} ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -O binary -S ${PROJECT_NAME} ${BIN_FILE}
        COMMAND ${CMAKE_OBJDUMP} -tD ${PROJECT_NAME} > ${LST_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}\nBuilding ${LST_FILE}"
        )

