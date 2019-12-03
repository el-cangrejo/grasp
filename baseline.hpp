#ifndef _BASELINE_HH
#define _BASELINE_HH

//ROS
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include <ignition/math.hh>

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// xtensor - for reading npy arrays
#include "xtensor/xarray.hpp"
#include <xtensor/xio.hpp>
#include "xtensor/xnpy.hpp"

// Interface
#include "Interface.hh"

void read_urdf_file (gazebo_msgs::SpawnModel &model);

std::vector<std::string> joints = {"rh_FFJ2", //1
																	 "rh_FFJ2", //2
																	 "rh_FFJ3", //3
																	 "rh_FFJ4", //4
																	 "rh_MFJ2", //5
																	 "rh_MFJ2", //6
																	 "rh_MFJ3", //7
																	 "rh_MFJ4", //8
																	 "rh_RFJ2", //9
																	 "rh_RFJ2", //10
																	 "rh_RFJ3", //11
																	 "rh_RFJ4", //12
																	 "rh_LFJ2", //13
																	 "rh_LFJ2", //14
																	 "rh_LFJ3", //15
																	 "rh_LFJ4", //16
																	 "rh_THJ2", //17
																	 "rh_THJ2", //18
																	 "rh_THJ3", //19
																	 "rh_THJ4", //20
																	 "rh_THJ5"}; //21
