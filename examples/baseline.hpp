#include <algorithm>
#include <chrono>
#include <vector>

// ROS
#include "ros/ros.h"
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <ignition/math.hh>

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

// xtensor - for reading npy arrays
#include "xtensor/xarray.hpp"
#include "xtensor/xnpy.hpp"
#include <xtensor/xbuilder.hpp>
#include <xtensor/xio.hpp>

// Interface
#include "Interface.hh"

// Custom messages
#include "target_request.pb.h"
#include "target_response.pb.h"

/// Topic monitored by target plugin for incoming requests
#define TARGET_REQUEST_TOPIC "~/grasp/target"
/// Topic to which target plugin publishes replies
#define TARGET_RESPONSE_TOPIC "~/grasp/target/response"
/// Get pose request
#define TARGET_GET_POSE grasp::msgs::TargetRequest::GET_POSE

typedef const boost::shared_ptr<const grasp::msgs::TargetResponse>
    TargetResponsePtr;

void read_urdf_file(gazebo_msgs::SpawnModel &model);

void loadFromYml(const std::string &file_name, const std::string &object_name,
                 std::vector<ignition::math::Pose3d> &target_poses,
                 std::vector<ignition::math::Pose3d> &hand_poses,
                 std::vector<int> &grasp_indices);

void parseArgs(int argc, char **argv, std::string &cfg_dir, std::string &robot);

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
                                   "rh_THJ5"}; // 21

void inline waitMs(int delay);

template <typename T>
std::pair<bool, int> findInVector(const std::vector<T> &vecOfElements,
                                  const T &element) {
  std::pair<bool, int> result;

  // Find given element in vector
  auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);

  if (it != vecOfElements.end()) {
    result.second = distance(vecOfElements.begin(), it);
    result.first = true;
  } else {
    result.first = false;
    result.second = -1;
  }

  return result;
};

void spawnModelFromFilename(gazebo::transport::PublisherPtr pub,
                            ignition::math::Pose3d &pose,
                            const std::string &filename);

void removeModel(gazebo::transport::PublisherPtr pub, const std::string &name);

void setGravity(gazebo::transport::PublisherPtr pub_physics, float z_value);

void getTargetPose(gazebo::transport::PublisherPtr pub);

void onTargetResponse(TargetResponsePtr &_msg);
