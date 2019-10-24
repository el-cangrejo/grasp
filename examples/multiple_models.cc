#include "multiple_models.hh"

int main(int _argc, char **_argv) {
	ros::init(_argc, _argv, "spawn_model");
	ros::NodeHandle n;
	ros::ServiceClient gz_spawn_s = 
		n.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
	gazebo_msgs::SpawnModel model;
  std::ifstream file("./models/shadowhand.urdf");
  std::string line;
 
   while(!file.eof()) // Parse the contents of the given urdf in a string
    {
      std::getline(file,line);
      model.request.model_xml+=line;
    }
  file.close();
 
	std::vector<ignition::math::Pose3d> poses{{0.0, 0, 0.01, 0, 0, 0},
																						{0.3, 0, 0.01, 0, 0, 0},
																						{0.6, 0, 0.01, 0, 0, 0},
																						{0.9, 0, 0.01, 0, 0, 0}};
	for (auto i = 0; i < poses.size(); ++i){
		model.request.model_name="my_model_" + std::to_string(i);
		model.request.reference_frame="world";
		model.request.initial_pose = convert_ignition_to_geometry(poses[i]);//
		gz_spawn_s.call(model); //Call the service
		waitMs(100);
	}
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

void inline waitMs(int delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}
