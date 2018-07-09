cmake_minimum_required(VERSION 3.0.0)
project(ros-peripherals)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs 
)

###########
## Build ##
###########


set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ros_peripherals_sonar sonar/ros/ros-peripherals-sonar.cpp)
target_compile_options(ros_peripherals_sonar PRIVATE -std=c++11)
target_link_libraries(ros_peripherals_sonar ${catkin_LIBRARIES}
                      ${roscpp_LIBRARIES} pthread)

#############
## Install ##
#############

install(TARGETS ros_peripherals_sonar
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
                SETUID SETGID
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hercules_sonar_skirt.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
