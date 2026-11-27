cmake_minimum_required(VERSION 3.8)
project(harurobo_sim)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_velocity.py DESTINATION lib/${PROJECT_NAME})
ament_package()
