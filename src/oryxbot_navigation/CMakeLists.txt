cmake_minimum_required(VERSION 2.8.3)
project(oryxbot_navigation)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch param rviz maps)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)