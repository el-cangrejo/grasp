#include "multiple_models.hh"

extern std::vector<std::string> joints;

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "spawn_model");
	ros::NodeHandle n;
	ros::ServiceClient gz_spawn_s = 
		n.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel model;
	read_urdf_file(model);
 
	std::string object_name = "orange_box";
	std::string model_filename = "model://orange_box";
	std::string file_name = "test_trial.rest.yml";
	ignition::math::Pose3d offset_pose{0.0, 0, 0, 0, 0, 0};
	const ignition::math::Pose3d offset_pose_{0.2, 0, 0, 0, 0, 0};
	std::vector<ignition::math::Pose3d> target_poses;
	std::vector<ignition::math::Pose3d> hand_poses;
	std::vector<int> grasp_indices;

	const auto grasp_file = "data/grasps.npy";
  const auto data = xt::load_npy<double>(grasp_file);

	loadFromYml(file_name, object_name, target_poses, hand_poses, grasp_indices);

  // Command-line arguments
  std::string config_file, robot;
  parseArgs(_argc, _argv, config_file, robot);
	gazebo::client::setup(_argc, _argv);
	// Setup communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	auto pub = node->Advertise<gazebo::msgs::Factory>(FACTORY_TOPIC);
	pub->WaitForConnection();
	auto pub_delete = node->Advertise<gazebo::msgs::Request>(REQUEST_TOPIC);
	pub_delete->WaitForConnection();

	for (auto j = 0; j < 10; ++j){
		int i = 2;
		auto target_pose = target_poses.at(i) + offset_pose;
		auto hand_pose = hand_poses.at(i) + offset_pose;
		
		std::cout << hand_poses.at(i) << std::endl;
		std::cout << hand_pose << std::endl;

		auto grasp_idx = grasp_indices.at(i);

		auto model_name{"hand_" + std::to_string(j)};
		model.request.model_name = model_name;
		model.request.reference_frame = "world";
		model.request.initial_pose = convert_ignition_to_geometry({0, 0, 0, 0, 0, 0});//
		gz_spawn_s.call(model); //Call the service
		waitMs(1000);

		// Interface for hand plugin
		Interface interface(std::move(model_name));
		interface.init(config_file,  robot);

		interface.openFingers();
		waitMs(1000);

		interface.setPose(hand_pose);
		waitMs(2000);

		spawnModelFromFilename(pub, target_pose, model_filename);
		waitMs(2000);

		std::vector<double> angles;
		angles.reserve(data.shape(1));
		for (unsigned int k = 0; k < data.shape(1); ++k) {
			/* if (joints.at(i).back() == '2' && joints.at(i) != "rh_THJ2") */
			if (joints.at(k).back() == '2')
				angles.push_back(2 * data(grasp_idx, k) * 3.14 / 180);
			else
				angles.push_back(data(grasp_idx, k) * 3.14 / 180);
		}
		interface.setJoints(joints, angles);

		offset_pose += offset_pose_;
	}
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

geometry_msgs::Pose convert_ignition_to_geometry(const ignition::math::Pose3d pose) {
	geometry_msgs::Pose new_pose;
	new_pose.position.x = pose.Pos().X();
	new_pose.position.y = pose.Pos().Y();
	new_pose.position.z = pose.Pos().Z();
	new_pose.orientation.x = pose.Rot().X();
	new_pose.orientation.y = pose.Rot().Y();
	new_pose.orientation.z = pose.Rot().Z();
	return new_pose;
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
void inline waitMs(int delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}
