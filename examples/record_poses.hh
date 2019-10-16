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

/// Get pose request
#define TARGET_GET_POSE        grasp::msgs::TargetRequest::GET_POSE
/// Set pose request
#define TARGET_SET_POSE        grasp::msgs::TargetRequest::SET_POSE
/// Get updated resting pose request
#define TARGET_GET_REST_POSE   grasp::msgs::TargetRequest::GET_REST_POSE

/// Shared pointer declaration for response message type
typedef const boost::shared_ptr<const grasp::msgs::TargetResponse>
    TargetResponsePtr;

/// \brief Requests updated target object resting pose
/// \param pub Publisher to target plugin topic
void getTargetPose(gazebo::transport::PublisherPtr pub);

/// TODO
void onTargetResponse(TargetResponsePtr & _msg);

void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot);
