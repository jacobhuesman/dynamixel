cmake_minimum_required(VERSION 3.5)
project(dynamixel)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/DynamicServo.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES Dynamixel
    CATKIN_DEPENDS roscpp tf2 dynamic_reconfigure
)

find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

# Add gmock
link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

#############
# Libraries #
#############

add_library(Dynamixel src/dynamixel.cpp)
add_dependencies(Dynamixel ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Dynamixel ${PROJECT_NAME}_gencfg)
target_link_libraries(Dynamixel mraa)
target_link_libraries(Dynamixel ${catkin_LIBRARIES})

############
# Programs #
############
add_executable(comm_tool src/util/comm_tool.cpp)
target_link_libraries(comm_tool Dynamixel)
target_link_libraries(comm_tool ${Boost_LIBRARIES})

add_executable(mraa_test src/tests/mraa_test.cpp)
target_link_libraries(mraa_test mraa)

#########
# Tests #
#########
catkin_add_gtest(servo_tests src/tests/servo_tests.cpp)
if(TARGET servo_tests)
  target_link_libraries(servo_tests ${catkin_LIBRARIES})
  target_link_libraries(servo_tests Dynamixel)
endif()