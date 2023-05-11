set(DRIVER_NAME simplefoc)
add_library(${DRIVER_NAME} INTERFACE)

target_sources(${DRIVER_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/BLDCMotor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/StepperMotor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common/base_classes/FOCMotor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common/base_classes/CurrentSense.cpp
  ${CMAKE_CURRENT_LIST_DIR}/drivers/BLDCDriver3PWM.cpp
  ${CMAKE_CURRENT_LIST_DIR}/drivers/StepperDriver4PWM.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common/lowpass_filter.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common/pid.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common/foc_utils.cpp
  ${CMAKE_CURRENT_LIST_DIR}/communication/Commander.cpp
  #${CMAKE_CURRENT_LIST_DIR}/drivers/hardware_specific/generic_mcu.cpp
  ${CMAKE_CURRENT_LIST_DIR}/drivers/hardware_specific/rp2040/rp2040_mcu.cpp

  #${CMAKE_CURRENT_LIST_DIR}/StepperMotor.cpp
)

add_compile_definitions(TARGET_RP2040)

target_include_directories(${DRIVER_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Pull in pico libraries that we need
target_link_libraries(${DRIVER_NAME} INTERFACE pico_stdlib hardware_spi hardware_i2c hardware_pwm)