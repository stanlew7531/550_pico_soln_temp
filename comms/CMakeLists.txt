add_library(comms STATIC
  src/common.c
  src/protocol.c
  src/listener.c
  src/topic_data.c
  #src/messages.c
  src/messages_mb.c
)

target_link_libraries(comms
  pico_stdlib
  pico_stdio_usb
  pico_multicore
  hardware_pwm
  hardware_pio
)

target_include_directories(comms PUBLIC
  include
)