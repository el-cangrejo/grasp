#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <ignition/math.hh>

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <vector>
#include <string>
#include <chrono>
#include <thread>

geometry_msgs::Pose convert_ignition_to_geometry(const ignition::math::Pose3d p);

void inline waitMs(int delay);
