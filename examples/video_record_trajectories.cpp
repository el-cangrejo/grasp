#include "manipulation.hpp"

/* 
 * Script to perform object manipulation trajectories.
 * To run first run:
 * roscore & rosrun gazebo_ros gazebo worlds/my_empty.world --verbose -e dart
 * or
 * roscore & rosrun gazebo_ros gzserver worlds/my_empty.world --verbose -e dart
 * to run headless.
 * Then run the hand_remote example and then the manipulation providing a
 * trajectory file and an indices file.
 */

ignition::math::Pose3d g_target_pose{0, 0, 0, 0, 0, 0};
int g_count_respones{0};

int main(int _argc, char **_argv)
{
	// Parse command line arguments
  std::string config_file, robot, trajectories_file, indices_file, n; 
  parseArgs(_argc, _argv, config_file, robot, trajectories_file, indices_file, n);

	// Read grasps, trajectories, poses
	const auto grasp_file = "data/grasps.npy";
  const auto grasp_data = xt::load_npy<double>(grasp_file);

  const auto trajectories_data = xt::load_npy<double>(trajectories_file);
  const auto indices_data = xt::load_npy<double>(indices_file);

	const std::string model_uri = "model://red_box";
	const std::string file_name = "data/recorded_grasps/baseline_trial.rest.yml";
	const std::string object_name = "baseline";
	std::string entity_name = "red_box";

	std::vector<ignition::math::Pose3d> target_poses;
	std::vector<ignition::math::Pose3d> hand_poses;
	std::vector<int> grasp_indices;
	loadFromYml(file_name, object_name, target_poses, hand_poses, grasp_indices);

	/* // ROS calls for spawning hand */
	ros::init(_argc, _argv, "spawn_model");
	ros::NodeHandle ros_node;
	ros::ServiceClient gz_spawn_s = ros_node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel hand_model;
	read_urdf_file(hand_model);

	hand_model.request.model_name = "shadowhand";
	hand_model.request.reference_frame = "world";
	gz_spawn_s.call(hand_model); //Call the service

	// Gazebo client setup
	gazebo::client::setup(_argc, _argv);
	/* // Setup communication */
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	std::cout << "Initialize communication\n";
	auto pub_factory = node->Advertise<gazebo::msgs::Factory>("~/factory");
	pub_factory->WaitForConnection();
	auto pub_physics = node->Advertise<gazebo::msgs::Physics>("~/physics");
	pub_physics->WaitForConnection();
	auto pub_delete = node->Advertise<gazebo::msgs::Request>("~/request");
	pub_delete->WaitForConnection();
	auto pub_target = node->Advertise<grasp::msgs::TargetRequest>(TARGET_REQUEST_TOPIC, 100);
	auto sub_target = node->Subscribe(TARGET_RESPONSE_TOPIC, onTargetResponse);

	// Initialize hand interface
	Interface api;
	std::cout << "Initialize interface\n";
	api.init(config_file,  robot);

	waitMs(1000);

	int trial_idx{};
	std::string line{};
	while (std::cout << "trajectory to execute: ") {
		
		getline(std::cin, line);
		try{
				trial_idx = std::stoi(line);
		} catch (std::invalid_argument) {
			std::cout << "Give number \n";
		}

		std::pair<bool, int> res_1 = findInVector<int>(grasp_indices, indices_data(trial_idx, 0));
		std::pair<bool, int> res_2 = findInVector<int>(grasp_indices, indices_data(trial_idx, 1));

		setGravity(pub_physics, -8.8);
		api.openFingers();
		waitMs(1000);

		api.setPose(hand_poses.at(res_1.second));
		waitMs(1000);
		
		setGravity(pub_physics, 0.0);

		spawnModelFromFilename(pub_factory, target_poses.at(res_1.second), model_uri);
		waitMs(1000);

		pub_target->WaitForConnection();
		std::vector<double> angles;
		for (unsigned int k = 0; k < grasp_data.shape(1); ++k) {
			angles.push_back(grasp_data(grasp_indices.at(res_1.second), k) * 3.14 / 180);
		}

		api.setJoints(joints, angles);
		waitMs(1000);

		setGravity(pub_physics, -9.8);
		std::vector<std::string> virtual_pz_joint{"virtual_pz_joint"};
		std::vector<double> value{hand_poses.at(res_1.second).Pos().Z() + 0.01};
		api.setJoints(virtual_pz_joint, value);
		waitMs(1000);

		for (auto traj_idx = 0; traj_idx < trajectories_data.shape(1); ++traj_idx) {
			std::vector<double> angles;
			for (unsigned int i = 0; i < trajectories_data.shape(2); i++) {
			angles.push_back(trajectories_data(trial_idx, traj_idx, i) * 3.14 / 180);
			}
			api.setJoints(joints, angles);
			waitMs(500);
		}

		waitMs(5000);

		removeModel(pub_delete, entity_name);
	}

	// Shut down
  gazebo::client::shutdown();
  return 0;
}

void setGravity(gazebo::transport::PublisherPtr pub_physics, float z_value) {
	gazebo::msgs::Vector3d* grvty = new gazebo::msgs::Vector3d();
	gazebo::msgs::Physics msg;
	grvty->set_x(0.0);
	grvty->set_y(0.0);
	grvty->set_z(z_value);
	msg.set_allocated_gravity(grvty);
	pub_physics->Publish(msg);
	waitMs(500);
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

void parseArgs(int argc, char **argv, std::string &cfg_dir, std::string &robot,
		std::string &trajectories_file, std::string &indices_file,std::string
		&trial_count) {
  int opt;
  bool c, r, t, i, n;

  while ((opt = getopt(argc, argv, "c: r: t: i: n:")) != EOF) {
    switch (opt) {
    case 'c':
      c = true;
      cfg_dir = optarg;
      break;
    case 'r':
      r = true;
      robot = optarg;
      break;
    case 't':
      t = true;
      trajectories_file = optarg;
      break;
    case 'i':
      i = true;
      indices_file = optarg;
      break;
    case 'n':
      n = true;
      trial_count = optarg;
      break;
    default:
      std::cout << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (!c || !r || !t || !i || !n) {
    exit(EXIT_FAILURE);
  }

  std::cout << "Parameters:\n"
            << "   Robot configuration yaml '" << cfg_dir << "'\n"
            << "   Robot                    '" << robot << "'\n"
            << "   Trajectory file          '" << trajectories_file << "'\n"
            << "   Indices file             '" << indices_file << "'\n"
            << "   Trial count              '" << trial_count << "'\n"
            << std::endl;
}

void inline waitMs(int delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
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

void getTargetPose(gazebo::transport::PublisherPtr pub) {
  grasp::msgs::TargetRequest msg;
  msg.set_type(TARGET_GET_POSE);
  pub->Publish(msg);
}

void onTargetResponse(TargetResponsePtr &_msg) {
  if (_msg->has_pose()) {
    ignition::math::Pose3d pose(gazebo::msgs::ConvertIgn(_msg->pose()));
    g_target_pose = pose;
		g_count_respones++;
    std::cout << "Received target pose " << pose << std::endl;
  }
}
