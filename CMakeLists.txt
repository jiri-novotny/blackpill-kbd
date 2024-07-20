cmake_minimum_required(VERSION 3.0)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

project(blackpill-kbd C ASM)

message("-- Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")

include_directories(Core/Inc)
include_directories(lib/cmsis)
include_directories(lib/cmsis_device_f4/Include/)
include_directories(lib/stm32f4xx_hal_driver/Inc/)
include_directories(Middlewares/ST/STM32_USB_Device_Library/Core/Inc)
include_directories(Middlewares/ST/STM32_USB_Device_Library/Class/HID/Inc)
include_directories(USB_DEVICE/App)
include_directories(USB_DEVICE/Target)

add_definitions(-DSTM32F401xC)
add_definitions(-DUSE_HAL_DRIVER)

set(SOURCE
  ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c

  ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f4/Source/Templates/gcc/startup_stm32f401xc.s
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f4/Source/Templates/system_stm32f4xx.c

  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pcd.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pcd_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pwr.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pwr_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_dma.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_rcc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_tim.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash_ramfunc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_exti.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_dma_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_ll_usb.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_tim_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_rcc_ex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_cortex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_gpio.c

  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/usbd_hid.c

  ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usb_device.c
  ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c
)

# Needs to be set properly for your MCU
set(CPU_PARAMETERS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

# Set linker script
set(linker_script_SRC ${CMAKE_CURRENT_SOURCE_DIR}/STM32F401CCUX_FLASH.ld)

# Executable files
add_executable(${PROJECT_NAME} ${SOURCE})

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-strict-aliasing
    -pedantic
    -fstack-usage -fcyclomatic-complexity
    $<$<CONFIG:Debug>:-Og -g -ggdb3>
    $<$<CONFIG:MinSizeRel>:-Os>
    $<$<CONFIG:Release>:-O3>
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--start-group -lc -lm -Wl,--end-group
    -Wl,--print-memory-usage
)

# Execute post-build to print size, generate hex and bin
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND "echo"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
