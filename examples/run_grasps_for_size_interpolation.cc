#include "run_grasps_for_size_interpolation.hh"

extern std::vector<std::string> joints;

int main(int _argc, char **_argv) {
  // Command-line arguments
  std::string config_file, robot;
  parseArgs(_argc, _argv, config_file, robot);

	std::string object_name = "red_large";
	std::string model_filename = "model://red_box";
	std::string file_name = "object_size_extrapolation_trial.rest.yml";
	std::vector<ignition::math::Pose3d> target_poses;
	std::vector<ignition::math::Pose3d> hand_poses;
	std::vector<int> grasp_indices;

	auto grasp_file = "data/example_object_size_grasps.npy";
  auto data = xt::load_npy<double>(grasp_file);

	loadFromYml(file_name, object_name, target_poses, hand_poses, grasp_indices);
	//
	// ROS calls for spawning hand
	ros::init(_argc, _argv, "spawn_model");
	ros::NodeHandle ros_node;
	ros::ServiceClient gz_spawn_s = ros_node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel hand_model;
	read_urdf_file(hand_model);

	hand_model.request.model_name = "shadowhand";
	hand_model.request.reference_frame = "world";
	gz_spawn_s.call(hand_model); //Call the service

	gazebo::client::setup(_argc, _argv);

	// Setup communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	auto pub = node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
	pub->WaitForConnection();
	auto pub_delete = node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);
	pub_delete->WaitForConnection();

	// Interface for hand plugin
	Interface interface("shadowhand");
	interface.init(config_file,  robot);

	waitMs(1000);
	interface.openFingers();
	waitMs(1000);

  std::string line{};
	int idx{};
  while (std::cout << "> ") {
    // Process command
    getline(std::cin, line);
    try {
      idx = std::stoi(line);
    } catch (std::invalid_argument) {
      std::cout << "Give number \n";
    }

		interface.setPose(hand_poses.at(0));
		waitMs(1000);
		spawnModelFromFilename(pub, target_poses.at(0), model_filename);
		waitMs(1000);

		std::vector<double> angles;
    for (unsigned int i = 0; i < data.shape(1); i++) {
      /* if (joints.at(i).back() == '2' && joints.at(i) != "rh_THJ2") */
			/* /1* /2* if (joints.at(i).back() == '2') *2/ *1/ */ 
					/* angles.push_back(2 * data(idx, i) * 3.14 / 180); */
			/* else */
					angles.push_back(data(idx, i) * 3.14 / 180);
		}

		interface.setJoints(joints, angles);

		std::cout << "> ";
    getline(std::cin, line);
		removeModel(pub_delete, "red_box");
		waitMs(1000);
		interface.openFingers();
		waitMs(1000);
	}
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

void loadFromYml(
    const std::string & file_name,
    const std::string & object_name,
		std::vector<ignition::math::Pose3d> &target_poses,
		std::vector<ignition::math::Pose3d> &hand_poses, 
		std::vector<int> &grasp_indices)
{
    try
    {
        YAML::Node config = YAML::LoadFile(file_name);

				int num_poses = config[object_name]["pose"].size();
				double px,py,pz,rr,rp,ry;

				for (int i = 0; i < num_poses; i++)
				{
					std::string str_i{std::to_string(i)};
					YAML::Node target_node = config[object_name]["pose"][str_i];
					px = target_node[0].as<double>();
					py = target_node[1].as<double>();
					pz = target_node[2].as<double>();
					rr = target_node[3].as<double>();
					rp = target_node[4].as<double>();
					ry = target_node[5].as<double>();
					target_poses.emplace_back(px,py,pz,rr,rp,ry);

					YAML::Node hand_node = config["hand"]["pose"][str_i];
					px = hand_node[0].as<double>();
					py = hand_node[1].as<double>();
					pz = hand_node[2].as<double>();
					rr = hand_node[3].as<double>();
					rp = hand_node[4].as<double>();
					ry = hand_node[5].as<double>();
					hand_poses.emplace_back(px,py,pz,rr,rp,ry);

					YAML::Node grasp_idx_node = config["grasp index"];
					grasp_indices.push_back(grasp_idx_node[str_i][0].as<int>());
				}
    }
    catch (YAML::Exception& yamlException)
    {
        errorPrintTrace("Unable to parse " << file_name);
    }
}

void spawnModelFromFilename(
    gazebo::transport::PublisherPtr pub,
    ignition::math::Pose3d & pose,
    const std::string & filename)
{
    gazebo::msgs::Factory msg;
    msg.set_sdf_filename(filename);
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
		std::cout << pose << "\n";
    msg.set_allocated_pose(pose_msg);
    pub->Publish(msg);
}

void removeModel(
    gazebo::transport::PublisherPtr pub,
    const std::string & name)
{
    gazebo::msgs::Request *msg;
    msg = gazebo::msgs::CreateRequest("entity_delete", name);
    pub->Publish(*msg);
}

void inline waitMs(int delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

void read_urdf_file (gazebo_msgs::SpawnModel &model) {
  std::ifstream file("./models/shadowhand.urdf");
  std::string line;
 
   while(!file.eof()) // Parse the contents of the given urdf in a string
    {
      std::getline(file,line);
      model.request.model_xml+=line;
    }
  file.close();
}
