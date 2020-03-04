cmake_minimum_required(VERSION 2.8.3)
project(engagement_detector)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)