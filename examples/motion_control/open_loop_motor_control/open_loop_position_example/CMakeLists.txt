set(OUTPUT_NAME open_loop_position_example)

add_executable(
  ${OUTPUT_NAME}
  open_loop_position_example.cpp
)

#include(common/pimoroni_i2c)
#include(drivers/plasma/plasma)
#include(drivers/pwm/pwm)
#include(drivers/motor/motor)
#include(drivers/pwm/pwm_cluster)
#include(drivers/motor/motor_cluster)
#include(drivers/button/button)
#include(drivers/encoder/encoder)
#include(drivers/pid/pid)
#include(drivers/analog/analog)
#include(drivers/analogmux/analogmux)
#include(libraries/motor2040/motor2040)


# Pull in pico libraries that we need
target_link_libraries(${OUTPUT_NAME}
  pico_stdlib
  simplefoc
  #motor2040
  #button
  #pid
  #analog
  #analogmux
  #i2c_device
)

# enable usb output
pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${OUTPUT_NAME})