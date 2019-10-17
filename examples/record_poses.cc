#include "record_poses.hh"

ignition::math::Pose3d g_target_pose      {0,0,0,0,0,0};
ignition::math::Pose3d g_hand_pose      {0,0,0,0,0,0};
int g_grasp_idx {0};

int main(int _argc, char **_argv)
{
    // Command-line arguments
    std::string config_file, robot;
    parseArgs(_argc, _argv, config_file, robot);

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);
    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the target plugin request topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<grasp::msgs::TargetRequest>(TARGET_REQUEST_TOPIC, 100);
    // Subscribe to the grasp target topic and link callback function
    gazebo::transport::SubscriberPtr sub =
        node->Subscribe(TARGET_RESPONSE_TOPIC, onTargetResponse);

    // Publish to the target plugin request topic
    gazebo::transport::PublisherPtr hand_pub =
        node->Advertise<grasp::msgs::Hand>(HAND_REQUEST_TOPIC, 100);
    // Subscribe to the grasp target topic and link callback function
    gazebo::transport::SubscriberPtr hand_sub =
        node->Subscribe(HAND_RESPONSE_TOPIC, onHandResponse);

    // Subscribe to the grasp target topic and link callback function
    gazebo::transport::SubscriberPtr grasp_idx_sub =
        node->Subscribe("~/", onGraspIdxResponse);

    // Wait for a subscriber to connect to this publisher
		std::cout << "Connecting" << std::endl;
    pub->WaitForConnection();
		std::cout << "Target publisher connected" << std::endl;
    hand_pub->WaitForConnection();
		std::cout << "Hand publisher connected" << std::endl;

    // Interface
    Interface api;
    // Init interface with config file
    if (!api.init(config_file, robot)) {
        std::cout << "Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Main loop
    std::string line = "";
    while (std::cout << PROMPT)
    {
        // Process command
        getline(std::cin, line);
        std::stringstream input_stream(line);
        std::string command = input_stream.str();
        // Change pose request
        if (command == "pose") {
					std::string model_name = "orange_box";

					getHandPose(hand_pub);
					waitMs(200);

					getTargetPose(pub);
					waitMs(200);

					std::string out_file = + "./trial.rest.yml"; 
					writeToYml(out_file, model_name, g_target_pose, g_hand_pose, g_grasp_idx);
				}
				else {
					std::cout << "Wrong command!\n" << std::endl;
				}

    }
    // Shut down
    gazebo::client::shutdown();
    return 0;
}

void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot)
{
    int opt;
    bool c, r;

    while ((opt = getopt(argc,argv,"c: r:")) != EOF)
    {
        switch (opt)
        {
            case 'c':
                c = true; cfg_dir = optarg;    break;
            case 'r':
                r = true; robot = optarg; break;
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!c || !r) {
        exit(EXIT_FAILURE);
    }

    std::cout << "Parameters:\n" <<
        "   Robot configuration yaml '" << cfg_dir << "'\n" <<
        "   Robot                    '" << robot << "'\n" << std::endl;
}

void getTargetPose(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::TargetRequest msg;
    msg.set_type(TARGET_GET_POSE);
    pub->Publish(msg);
}

void getHandPose(gazebo::transport::PublisherPtr pub)
{
    grasp::msgs::Hand msg;
    msg.set_type(grasp::msgs::Hand::GET);
    pub->Publish(msg);
}

void onTargetResponse(TargetResponsePtr & _msg)
{
    if (_msg->has_pose())
    {
        ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
				g_target_pose = pose;
        std::cout << "\nReceived target pose " << pose << std::endl;
    }
}

void onHandResponse(HandMsgPtr & _msg)
{
    if (_msg->has_pose())
    {
        ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
				g_hand_pose = pose;
        std::cout << "\nReceived hand pose " << pose << std::endl;
    }
}

void onGraspIdxResponse(IntMsgPtr & _msg) 
{
		g_grasp_idx = _msg->data();
		std::cout << "\nReceived grasp idx " << g_grasp_idx << std::endl;
}

void writeToYml(
    const std::string & file_name,
    const std::string & object_name,
    const ignition::math::Pose3d & target_pose,
    const ignition::math::Pose3d & hand_pose,
		const int grasp_idx)
{
    YAML::Node root;

		// Build YAML node
		root[object_name]["pose"][0] = target_pose.Pos().X();
		root[object_name]["pose"][1] = target_pose.Pos().Y();
		root[object_name]["pose"][2] = target_pose.Pos().Z();
		root[object_name]["pose"][3] = target_pose.Rot().Roll();
		root[object_name]["pose"][4] = target_pose.Rot().Pitch();
		root[object_name]["pose"][5] = target_pose.Rot().Yaw();

		root["hand"]["pose"][0] = hand_pose.Pos().X();
		root["hand"]["pose"][1] = hand_pose.Pos().Y();
		root["hand"]["pose"][2] = hand_pose.Pos().Z();
		root["hand"]["pose"][3] = hand_pose.Rot().Roll();
		root["hand"]["pose"][4] = hand_pose.Rot().Pitch();
		root["hand"]["pose"][5] = hand_pose.Rot().Yaw();

		root["grasp index"][0] = grasp_idx;

    std::ofstream fout(file_name);
    if (!fout) {
        errorPrintTrace("Could not write to " << file_name);
    } else {
        fout << root;
        debugPrintTrace("All rest poses written to " << file_name);
    }
}

void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}
