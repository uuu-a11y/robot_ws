cmake_minimum_required(VERSION 2.8.3)

project(oryxbot_description)

find_package(catkin REQUIRED
	geometry_msgs
	roscpp
	std_msgs 
	std_srvs
	message_generation
	arm_controller
	tf2_ros
	tf
)
## Generate services in the 'srv' folder
add_service_files(
	FILES
	FK.srv
	IK.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	geometry_msgs std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swiftpro_control
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs std_srvs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(fk_swiftpro src/fk_swiftpro.cpp)
add_executable(ik_swiftpro src/ik_swiftpro.cpp)
add_executable(home_pose src/home_pose.cpp)
add_executable(simple_picker src/simple_picker.cpp)
add_executable(pose_swiftpro src/pose_swiftpro.cpp)
add_executable(pick_ar_gazebo src/pick_ar_gazebo.cpp)

add_dependencies(fk_swiftpro ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(ik_swiftpro ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(home_pose ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(simple_picker ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pose_swiftpro ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pick_ar_gazebo ${PROJECT_NAME}_generate_messages_cpp)
 
target_link_libraries(fk_swiftpro ${catkin_LIBRARIES})
target_link_libraries(ik_swiftpro ${catkin_LIBRARIES})
target_link_libraries(home_pose ${catkin_LIBRARIES}) 
target_link_libraries(simple_picker ${catkin_LIBRARIES}) 
target_link_libraries(pose_swiftpro ${catkin_LIBRARIES}) 
target_link_libraries(pick_ar_gazebo ${catkin_LIBRARIES}) 

## Add cmake target dependencies of the executable
## same as for the library above

#foreach(dir config launch meshes urdf)
#	install(DIRECTORY ${dir}/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)
