cmake_minimum_required(VERSION 3.0)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

project(blackpill-kbd C ASM)

message("-- Build type: " ${CMAKE_BUILD_TYPE})

option (BUILD_BTL "Build bootloader" OFF)
option (BUILD_APP "Build target" ON)
option (BUILD_APP1 "Link as app 1" OFF)
option (BUILD_APP2 "Link as app 2" OFF)
option (BUILD_STANDALONE "Link standalone" ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fdiagnostics-color=always")

add_definitions(-DSTM32F401xC)
add_definitions(-DUSE_HAL_DRIVER)

include_directories(lib/cmsis)
include_directories(lib/cmsis_device_f4/Include/)
include_directories(lib/stm32f4xx_hal_driver/Inc/)

set(SOURCE
  ${CMAKE_CURRENT_SOURCE_DIR}/common/syscalls.c
  ${CMAKE_CURRENT_SOURCE_DIR}/common/sysmem.c

  ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f4/Source/Templates/gcc/startup_stm32f401xc.s
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmsis_device_f4/Source/Templates/system_stm32f4xx.c

  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_cortex.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_gpio.c
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_rcc.c
)

if(BUILD_BTL)
  # Set linker script
  set(linker_script_SRC ${CMAKE_CURRENT_SOURCE_DIR}/STM32F401CCUX_FLASH_boot.ld)

  include_directories(boot)

  set(SOURCE
    ${SOURCE}

    ${CMAKE_CURRENT_SOURCE_DIR}/boot/main.c
  )
elseif(BUILD_APP)
  # Set linker script
  if(BUILD_APP1)
    set(linker_script_SRC ${CMAKE_CURRENT_SOURCE_DIR}/STM32F401CCUX_FLASH_app1.ld)
  elseif(BUILD_APP2)
    set(linker_script_SRC ${CMAKE_CURRENT_SOURCE_DIR}/STM32F401CCUX_FLASH_app2.ld)
  elseif(BUILD_STANDALONE)
    set(linker_script_SRC ${CMAKE_CURRENT_SOURCE_DIR}/STM32F401CCUX_FLASH.ld)
  else()
    message(FATAL_ERROR "Please select target link!")
  endif()

  if(USB_CONSUMER)
    add_definitions(-DUSB_CONSUMER)
  endif()

  include_directories(app)
  include_directories(lib/stm32_mw_usb_device/Core/Inc/)
  include_directories(USB_DEVICE/App)
  include_directories(USB_DEVICE/Class/HID/Inc)
  include_directories(USB_DEVICE/Target)

  set(SOURCE
    ${SOURCE}

    ${CMAKE_CURRENT_SOURCE_DIR}/app/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/stm32f4xx_it.c

    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32_mw_usb_device/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32_mw_usb_device/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32_mw_usb_device/Core/Src/usbd_core.c

    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_ll_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/stm32f4xx_hal_driver/Src/stm32f4xx_hal_rcc_ex.c

    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Class/HID/Src/usbd_hid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c
  )
else()
  message(FATAL_ERROR "Please select target build!")
endif()

# Needs to be set properly for your MCU
set(CPU_PARAMETERS -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb)

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
