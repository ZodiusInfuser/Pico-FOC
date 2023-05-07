set(DRIVER_NAME simplefoc)
add_library(${DRIVER_NAME} INTERFACE)

target_sources(${DRIVER_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/BLDCMotor.cpp
  ${CMAKE_CURRENT_LIST_DIR}/common/base_classes/FOCMotor.cpp
  #${CMAKE_CURRENT_LIST_DIR}/StepperMotor.cpp
)

target_include_directories(${DRIVER_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Pull in pico libraries that we need
target_link_libraries(${DRIVER_NAME} INTERFACE pico_stdlib hardware_spi hardware_i2c)