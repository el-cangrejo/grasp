#include "baseline.hpp"


int main(int _argc, char **_argv)
{
	// Read grasps, trajectories, poses
	const auto grasp_file = "data/grasps.npy";
  const auto grasp_data = xt::load_npy<double>(grasp_file);

	const auto trajectories_file = "data/trajectories.npy";
  const auto trajectories_data = xt::load_npy<double>(trajectories_file);

	const auto indices_file = "data/indices.npy";
  const auto indices_data = xt::load_npy<double>(indices_file);

	std::string file_name = "baseline_trial.rest.yml";
	std::vector<ignition::math::Pose3d> target_poses;
	std::vector<ignition::math::Pose3d> hand_poses;
	std::vector<int> grasp_indices;
	loadFromYml(file_name, object_name, target_poses, hand_poses, grasp_indices);

	// ROS calls for spawning hand
	ros::init(_argc, _argv, "spawn_model");
	ros::NodeHandle ros_node;
	ros::ServiceClient gz_spawn_s = ros_node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel hand_model;
	read_urdf_file(hand_model);

	hand_model.request.model_name = "hand";
	hand_model.request.reference_frame = "world";
	gz_spawn_s.call(hand_model); //Call the service

	// Initialize hand interface
	Interface interface("hand");
	interface.init(config_file,  robot);

	const int n_trajectories = trajectories_data.shape(0);

	for (auto traj_idx = 0; traj_idx < n_trajectories; ++traj_idx) {
	
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
