#include "record_poses.hh"

/* 
 * Script to record the poses of the hand and the object during grasping.
 * To run first run:
 * roscore & rosrun gazebo_ros gazebo worlds/object.world --verbose -e dart
 * or
 * roscore & rosrun gazebo_ros gzserver worlds/object.world --verbose -e dart
 * to run headless. (Object is red box)
 * Then run the hand_remote example and then the record_poses and then the
 * read_grasps scripts.
 */

ignition::math::Pose3d g_target_pose{0, 0, 0, 0, 0, 0};
ignition::math::Pose3d g_hand_pose{0, 0, 0, 0, 0, 0};
int g_grasp_idx{0};

int main(int _argc, char **_argv) {
  // Command-line arguments
  std::string config_file, robot;
  parseArgs(_argc, _argv, config_file, robot);

  // Load gazebo as a client
  gazebo::client::setup(_argc, _argv);
	// Setup communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	std::map<std::string, gazebo::transport::PublisherPtr> pubs;
	std::map<std::string, gazebo::transport::SubscriberPtr> subs;
	setupCommunications(node, pubs, subs);

  // Interface
  Interface api("shadowhand");
  // Init interface with config file
  if (!api.init(config_file, robot)) {
    std::cout << "Exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Main loop
  std::string line{};
	std::vector<ignition::math::Pose3d> target_poses;
	std::vector<ignition::math::Pose3d> hand_poses;
	std::vector<int> grasp_indices;

  while (std::cout << PROMPT) {
    // Process command
    getline(std::cin, line);
    std::stringstream input_stream(line);
    std::string command = input_stream.str();
    // Change pose request
    if (command == "pose") {
      getHandPose(pubs["hand"]);
      std::cout << "Getting hand pose\n" << std::endl;
      waitMs(200);

      getTargetPose(pubs["target"]);
      std::cout << "Getting target pose\n" << std::endl;
      waitMs(500);

			target_poses.push_back(g_target_pose);
			hand_poses.push_back(g_hand_pose);
			grasp_indices.push_back(g_grasp_idx);
    } else if (command == "exit") {
			break;
    } else {
      std::cout << "Unknown command, try again..\n" << std::endl;
		}
  }

	std::string model_name = "red_large";
	std::string out_file = "./" + model_name + "_trial.rest.yml";
	writeToYml(out_file, model_name, target_poses, hand_poses, grasp_indices);

  // Shut down
  gazebo::client::shutdown();
  return 0;
}

void parseArgs(int argc, char **argv, std::string &cfg_dir,
               std::string &robot) {
  int opt;
  bool c, r;

  while ((opt = getopt(argc, argv, "c: r:")) != EOF) {
    switch (opt) {
    case 'c':
      c = true;
      cfg_dir = optarg;
      break;
    case 'r':
      r = true;
      robot = optarg;
      break;
    default:
      std::cout << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (!c || !r) {
    exit(EXIT_FAILURE);
  }

  std::cout << "Parameters:\n"
            << "   Robot configuration yaml '" << cfg_dir << "'\n"
            << "   Robot                    '" << robot << "'\n"
            << std::endl;
}

void getTargetPose(gazebo::transport::PublisherPtr pub) {
  grasp::msgs::TargetRequest msg;
  msg.set_type(TARGET_GET_POSE);
  pub->Publish(msg);
}

void getHandPose(gazebo::transport::PublisherPtr pub) {
  grasp::msgs::Hand msg;
  msg.set_type(grasp::msgs::Hand::GET);
  pub->Publish(msg);
}

void onTargetResponse(TargetResponsePtr &_msg) {
  if (_msg->has_pose()) {
    ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
    g_target_pose = pose;
    std::cout << "\nReceived target pose " << pose << std::endl;
  }
}

void onHandResponse(HandMsgPtr &_msg) {
  if (_msg->has_pose()) {
    ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
    g_hand_pose = pose;
    std::cout << "\nReceived hand pose " << pose << std::endl;
  }
}

void onGraspIdxResponse(IntMsgPtr &_msg) {
  g_grasp_idx = _msg->data();
  std::cout << "\nReceived grasp idx " << g_grasp_idx << std::endl;
}

void writeToYml(const std::string &file_name, const std::string &object_name,
                const std::vector<ignition::math::Pose3d> &target_poses,
                const std::vector<ignition::math::Pose3d> &hand_poses, 
								const std::vector<int> grasp_indices) {
  YAML::Node root;

	assert(hand_poses.size() == target_poses.size() && 
				 hand_poses.size() == grasp_indices.size());

	for (auto i{0}; i < target_poses.size(); ++i) {
		// Build YAML node
		std::string str_i = std::to_string(i);

		root[object_name]["pose"][str_i][0] = target_poses.at(i).Pos().X();
		root[object_name]["pose"][str_i][1] = target_poses.at(i).Pos().Y();
		root[object_name]["pose"][str_i][2] = target_poses.at(i).Pos().Z();
		root[object_name]["pose"][str_i][3] = target_poses.at(i).Rot().Roll();
		root[object_name]["pose"][str_i][4] = target_poses.at(i).Rot().Pitch();
		root[object_name]["pose"][str_i][5] = target_poses.at(i).Rot().Yaw();

		root["hand"]["pose"][str_i][0] = hand_poses.at(i).Pos().X();
		root["hand"]["pose"][str_i][1] = hand_poses.at(i).Pos().Y();
		root["hand"]["pose"][str_i][2] = hand_poses.at(i).Pos().Z();
		root["hand"]["pose"][str_i][3] = hand_poses.at(i).Rot().Roll();
		root["hand"]["pose"][str_i][4] = hand_poses.at(i).Rot().Pitch();
		root["hand"]["pose"][str_i][5] = hand_poses.at(i).Rot().Yaw();

		root["grasp index"][str_i][0] = grasp_indices.at(i);
	}

  std::ofstream fout(file_name);
  if (!fout) {
    errorPrintTrace("Could not write to " << file_name);
  } else {
    fout << root;
    debugPrintTrace("All rest poses written to " << file_name);
  }
}

void setupCommunications(
    gazebo::transport::NodePtr & node,
    std::map<std::string, gazebo::transport::PublisherPtr> & pubs,
    std::map<std::string, gazebo::transport::SubscriberPtr> & subs)
{
	// Create the communication node
	node->Init();

	// Instance publishers and subscribers
	pubs["target"] = node->Advertise<grasp::msgs::TargetRequest>(TARGET_REQUEST_TOPIC, 100);
	subs["target"] = node->Subscribe(TARGET_RESPONSE_TOPIC, onTargetResponse);
	pubs["hand"] =  node->Advertise<grasp::msgs::Hand>(HAND_REQUEST_TOPIC, 100);
	subs["hand"] =  node->Subscribe(HAND_RESPONSE_TOPIC, onHandResponse);
	subs["grasp_idx"] = node->Subscribe("~/", onGraspIdxResponse);

	// Wait for a subscriber to connect to this publisher
	std::cout << "Connecting" << std::endl;
	pubs["target"]->WaitForConnection();
	std::cout << "Target publisher connected" << std::endl;
	pubs["hand"]->WaitForConnection();
	std::cout << "Hand publisher connected" << std::endl;
}

void inline waitMs(int delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}
