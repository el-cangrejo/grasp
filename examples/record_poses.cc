#include "record_poses.hh"

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

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();
		std::cout << "Target publisher connected" << std::endl;

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
        if (command == "hand") {
					std::cout << "Hand pose:\n";
					api.getPose();
				}
				else if (command == "target") {
					getTargetPose(pub);
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

void onTargetResponse(TargetResponsePtr & _msg)
{
    if (_msg->has_pose())
    {
        ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
        std::cout << "\nReceived target pose " << pose << std::endl;
    }
}
