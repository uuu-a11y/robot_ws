cmake_minimum_required(VERSION 2.8.3)
project(arm_controller)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  serial
  tf
  geometry_msgs
  std_srvs
)

add_message_files(FILES
  control.msg
)

add_service_files(FILES
  number.srv
  PickPlace.srv
  move.srv
  rotate.srv
  relative_pos.srv
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_swiftpro
  src/arm_swiftpro.cpp
)
add_dependencies(arm_swiftpro ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_swiftpro
  ${catkin_LIBRARIES}
)

add_executable(pick_ar
  src/pick_ar.cpp
)
add_dependencies(pick_ar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pick_ar
  ${catkin_LIBRARIES}
)
install(TARGETS arm_swiftpro
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS pick_ar
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach(dir launch srv msg)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
