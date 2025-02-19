cmake_minimum_required(VERSION 2.8.3)
project(unitree_controller)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    xpp_msgs
    xpp_states
    xpp_vis
    sensor_msgs
    unitree_legged_msgs
    towr_ros
    cmake_modules
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs
    DEPENDS Eigen
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

include_directories(${Eigen_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp
    src/storetrajectory.cpp
    src/inversekinematics.cpp
)

add_dependencies(${PROJECT_NAME} unitree_legged_msgs_gencpp)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${EXTRA_LIBS}
)

# add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
# target_link_libraries(unitreeFootContactPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
# target_link_libraries(unitreeDrawForcePlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(unitree_external_force src/external_force.cpp)
target_link_libraries(unitree_external_force ${catkin_LIBRARIES})

add_executable(unitree_servo src/servo.cpp)
target_link_libraries(unitree_servo ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_towr src/towrmove.cpp)
target_link_libraries(unitree_towr ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(unitree_move_kinetic src/move_publisher.cpp)
target_link_libraries(unitree_move_kinetic ${catkin_LIBRARIES})