// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Interface
#include "Interface.hh"

// I/O streams
#include <iostream>

// Custom messages
#include "target_request.pb.h"
#include "target_response.pb.h"

/// Command prompt
#define PROMPT "> "

/// Topic monitored by target plugin for incoming requests
#define TARGET_REQUEST_TOPIC   "~/grasp/target"
/// Topic to which target plugin publishes replies
#define TARGET_RESPONSE_TOPIC  "~/grasp/target/response"
//
/// Topic monitored by target plugin for incoming requests
#define HAND_REQUEST_TOPIC   "~/hand"
/// Topic to which target plugin publishes replies
#define HAND_RESPONSE_TOPIC  "~/hand/response"

/// Get pose request
#define TARGET_GET_POSE        grasp::msgs::TargetRequest::GET_POSE
/// Set pose request
#define TARGET_SET_POSE        grasp::msgs::TargetRequest::SET_POSE
/// Get updated resting pose request
#define TARGET_GET_REST_POSE   grasp::msgs::TargetRequest::GET_REST_POSE

/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::TargetResponse>
    TargetResponsePtr;

/// Shared pointer declaration for hand message type
typedef const boost::shared_ptr<const grasp::msgs::Hand>
    HandMsgPtr;

/// Shared pointer declaration for hand message type
typedef const boost::shared_ptr<const gazebo::msgs::Int>
    IntMsgPtr;

/// TODO
void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot);

/// \brief Requests updated target object resting pose
/// \param pub Publisher to target plugin topic
void getTargetPose(gazebo::transport::PublisherPtr pub);

/// TODO
void getHandPose(gazebo::transport::PublisherPtr pub);

/// TODO
void onTargetResponse(TargetResponsePtr & _msg);

/// TODO
void onHandResponse(HandMsgPtr & _msg);

/// TODO
void onGraspIdxResponse(IntMsgPtr & _msg);

/// TODO
void writeToYml(
    const std::string & file_name,
    const std::string & object_name,
    const ignition::math::Pose3d & target_pose,
    const ignition::math::Pose3d & hand_pose,
		const int grasp_idx);

/// TODO
void inline waitMs(int delay);

