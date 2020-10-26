// xtensor - for reading npy arrays
#include "xtensor/xarray.hpp"
#include <xtensor/xio.hpp>
#include "xtensor/xnpy.hpp"

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo_msgs/SpawnModel.h>
#include "ros/ros.h"

// Interface
#include "Interface.hh"

// I/O streams
#include <iostream>

/// Topic for Gazebo factory utility
#define FACTORY_TOPIC       "~/factory"
#define REQUEST_TOPIC       "~/request"

/// TODO
void loadFromYml(
    const std::string & file_name,
    const std::string & object_name,
		std::vector<ignition::math::Pose3d> &target_pose,
		std::vector<ignition::math::Pose3d> &hand_pose, 
		std::vector<int> &grasp_idx);

/// TODO
void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot);

void spawnModelFromFilename(
    gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d & pose,
    const std::string & filename);

void removeModel(
    gazebo::transport::PublisherPtr pub,
    const std::string & name);

/// TODO
void inline waitMs(int delay);

std::vector<std::string> joints = {"rh_FFJ2",  // 1
																	 "rh_FFJ2",  // 2
																	 "rh_FFJ3",  // 3
																	 "rh_FFJ4",  // 4
																	 "rh_MFJ2",  // 5
																	 "rh_MFJ2",  // 6
																	 "rh_MFJ3",  // 7
																	 "rh_MFJ4",  // 8
																	 "rh_RFJ2",  // 9
																	 "rh_RFJ2",  // 10
																	 "rh_RFJ3",  // 11
																	 "rh_RFJ4",  // 12
																	 "rh_LFJ2",  // 13
																	 "rh_LFJ2",  // 14
																	 "rh_LFJ3",  // 15
																	 "rh_LFJ4",  // 16
																	 "rh_THJ2",  // 17
																	 "rh_THJ2",  // 18
																	 "rh_THJ3",  // 19
																	 "rh_THJ4",  // 20
																	 "rh_THJ5"}; // 22

void read_urdf_file (gazebo_msgs::SpawnModel &model);
