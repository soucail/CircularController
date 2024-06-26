set(controller_SRC
  CircularController.cpp
)

set(controller_HDR
  CircularController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DCircularController_EXPORTS")

