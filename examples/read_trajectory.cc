/*!
    \file examples/hand_remote.cc
    \brief Control manipulator example

    \author João Borrego : jsbruglie
*/

#include "read_grasps.hh"

int main(int _argc, char **_argv) {
  // Command-line arguments
  std::string config_file, grasp_file, labels_file, robot;
  parseArgs(_argc, _argv, config_file, grasp_file, labels_file, robot);

  // Load gazebo as a client
  gazebo::client::setup(_argc, _argv);

  // Create the communication node
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Interface
  Interface api("shadowhand");
  // Init interface with config file
  if (!api.init(config_file, robot)) {
    std::cout << "Exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }

  auto data = xt::load_npy<double>(grasp_file);

  std::cout << data << std::endl;
  std::cout << "Shape 0 : " << data.shape(0) << std::endl;
  std::cout << "Shape 1 : " << data.shape(1) << std::endl;

  std::vector<std::string> joints = {"rh_FFJ2",  // 1
                                     "rh_FFJ2",  // 2
                                     "rh_FFJ3",  // 3
                                     "rh_FFJ4",  // 4
                                     "rh_MFJ2",  // 5
                                     "rh_MFJ2",  // 6
                                     "rh_MFJ3",  // 7
                                     "rh_MFJ4",  // 8
                                     "rh_RFJ2",  // 9
                                     "rh_RFJ2",  // 10
                                     "rh_RFJ3",  // 11
                                     "rh_RFJ4",  // 12
                                     "rh_LFJ2",  // 13
                                     "rh_LFJ2",  // 14
                                     "rh_LFJ3",  // 15
                                     "rh_LFJ4",  // 16
                                     "rh_THJ2",  // 17
                                     "rh_THJ2",  // 18
                                     "rh_THJ3",  // 19
                                     "rh_THJ4",  // 20
                                     "rh_THJ5"}; // 21

  // Main loop
  std::string line = "";
	for (auto traj_idx = 0; traj_idx < data.shape(1); ++traj_idx) {
    std::vector<double> angles;
    for (unsigned int i = 0; i < data.shape(1); i++) {
					angles.push_back(data(traj_idx, i) * 3.14 / 180);
		}
		api.setJoints(joints, angles);
		waitMs(200);
  }

  // Shut down
  gazebo::client::shutdown();
  return 0;
}

//////////////////////////////////////////////////
const std::string getUsage(const char *argv_0) {
  return "usage:   " + std::string(argv_0) + " [options]\n" +
         "options: -c <robot configuration yaml>\n" + "         -r <robot>\n";
}

//////////////////////////////////////////////////
void parseArgs(int argc, char **argv, std::string &cfg_dir,
               std::string &grasp_file, std::string &labels_file,
               std::string &robot) {
  int opt;
  bool c, r, g, l;

  while ((opt = getopt(argc, argv, "c: g: l: r:")) != EOF) {
    switch (opt) {
    case 'c':
      c = true;
      cfg_dir = optarg;
      break;
    case 'g':
      g = true;
      grasp_file = optarg;
      break;
    case 'l':
      l = true;
      labels_file = optarg;
      break;
    case 'r':
      r = true;
      robot = optarg;
      break;
    case '?':
      std::cerr << getUsage(argv[0]);
    default:
      std::cout << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  if (!c || !r || !g || !l) {
    std::cerr << getUsage(argv[0]);
    exit(EXIT_FAILURE);
  }

  std::cout << "Parameters:\n"
            << "   Robot configuration yaml '" << cfg_dir << "'\n"
            << "   Grasp Dataset '" << grasp_file << "'\n"
            << "   Grasp Labels  '" << labels_file << "'\n"
            << "   Robot                    '" << robot << "'\n"
            << std::endl;
}

void inline waitMs(int delay) {
  std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

