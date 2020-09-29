# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
FIND_PACKAGE( Boost COMPONENTS system REQUIRED )

# Find packages
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros)

# Include and link Gazebo and SDF libraries
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Include and link yaml-cpp library
include_directories(${YAML_CPP_INCLUDE_DIR})
link_directories(${YAML_CPP_INCLUDE_DIR})

# Include custom messages
include_directories(${PROJECT_BINARY_DIR}/msgs)
link_directories(${PROJECT_BINARY_DIR}/msgs)
# Include custom utilities
include_directories(${PROJECT_BINARY_DIR}/utils)
link_directories(${PROJECT_BINARY_DIR}/utils)

include_directories(${catkin_INCLUDE_DIRS})

# Read grasps example client
add_executable (hand_signs
    hand_signs.cc)
target_link_libraries(hand_signs
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES})
add_dependencies(hand_signs
    grasp_msgs
    grasp_interface)

# Read grasps example client
add_executable (read_grasps
    read_grasps.cc)
target_link_libraries(read_grasps
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES})
add_dependencies(read_grasps
    grasp_msgs
    grasp_interface)

# Hand plugin example client
add_executable (hand_remote
    hand_remote.cc)
target_link_libraries(hand_remote
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES})
add_dependencies(hand_remote
    grasp_msgs
    grasp_interface)

# Hand plugin example client
add_executable (record_poses
    record_poses.cc)
target_link_libraries(record_poses
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES})
add_dependencies(record_poses
    grasp_msgs
    grasp_interface)

# Target plugin example client
add_executable (target_example target_example.cc)
target_link_libraries(target_example
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(target_example
    grasp_msgs)

# Contact world plugin example client
add_executable (contacts_example contacts_example.cc)
target_link_libraries(contacts_example
    grasp_msgs
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(contacts_example
    grasp_msgs)

# Hand plugin example client
add_executable (baseline
    baseline.cpp)
target_link_libraries(baseline
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(baseline
    grasp_msgs
    grasp_interface)

# Example of manipulation trajectories (regrasping) 
add_executable (manipulation
    manipulation.cpp)
target_link_libraries(manipulation
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(manipulation
    grasp_msgs
    grasp_interface)

# Example of manipulation trajectories (regrasping) 
add_executable (read_trajectory
    read_trajectory.cc)
target_link_libraries(read_trajectory
    grasp_msgs
    grasp_interface
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES}
    ${YAML_CPP_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(read_trajectory
    grasp_msgs
    grasp_interface)
