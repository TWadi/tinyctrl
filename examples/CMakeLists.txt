add_executable(motor_speed motor_speed.c)
target_link_libraries(motor_speed PRIVATE tinyctrl::tinyctrl)
tinyctrl_set_warnings(motor_speed)
