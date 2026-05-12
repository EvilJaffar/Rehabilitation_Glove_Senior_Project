# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# ============================================================
# CMakeLists.txt — Rehabilitation Glove Pico Firmware
# ============================================================
# Build:
#   mkdir build && cd build
#   cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
#   make
# Produces: Main_UI.uf2  and  Main_Motor.uf2
# ============================================================

cmake_minimum_required(VERSION 3.13)
if(DEFINED ENV{PICO_SDK_PATH} AND EXISTS "$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake")
    include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
else()
    message(FATAL_ERROR "Could not find pico_sdk_import.cmake. Set PICO_SDK_PATH or add pico_sdk_import.cmake to project root.")
endif()

project(rehab_glove C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

pico_sdk_init()

# Ensure picotool can trigger USB reboot reliably from running firmware.
function(enable_picotool_usb_reset target_name)
    target_compile_definitions(${target_name} PRIVATE
        PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
        PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL=1
        PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT=1
    )
endfunction()

# ── Controller Pico ──────────────────────────────────────────
# Uses: UART, ADC (pots + EMG), I2C (ADS1115 for 5 FSRs)
add_executable(Main_UI Main_UI.cpp)
target_link_libraries(Main_UI
    pico_stdlib
    hardware_uart
    hardware_adc
    hardware_i2c
    hardware_gpio
    hardware_sync
    pico_float
)
pico_enable_stdio_usb(Main_UI 1)
pico_enable_stdio_uart(Main_UI 0)
enable_picotool_usb_reset(Main_UI)
pico_add_extra_outputs(Main_UI)

# ── Motor Pico ───────────────────────────────────────────────
# Uses: UART, PWM (5 motors), GPIO IRQ (5 encoders)
add_executable(Main_Motor Main_Motor.cpp)
target_link_libraries(Main_Motor
    pico_stdlib
    hardware_uart
    hardware_pwm
    hardware_gpio
    hardware_irq
    hardware_sync
    pico_float
)
pico_enable_stdio_usb(Main_Motor 1)
pico_enable_stdio_uart(Main_Motor 0)
enable_picotool_usb_reset(Main_Motor)
pico_add_extra_outputs(Main_Motor)
