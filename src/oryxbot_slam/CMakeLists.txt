cmake_minimum_required(VERSION 2.8.3)
project(oryxbot_slam)


catkin_package()

foreach(dir launch config rviz)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)