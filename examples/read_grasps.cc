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

  // Publish to the target plugin request topic
  gazebo::transport::PublisherPtr pub =
      node->Advertise<gazebo::msgs::Int>("~/", 100);

  pub->WaitForConnection();

  // Interface
  Interface api("shadowhand");
  // Init interface with config file
  if (!api.init(config_file, robot)) {
    std::cout << "Exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }

  auto data = xt::load_npy<double>(grasp_file);
  auto labels = xt::load_npy<double>(labels_file);

  std::cout << data << std::endl;
  std::cout << "Shape 0 : " << data.shape(0) << std::endl;
  std::cout << "Shape 1 : " << data.shape(1) << std::endl;

  std::vector<std::string> obj_configs = {"BIG GREEN BALL",
                                          "MEDIUM BLUE BALL",
                                          "SMALL WHITE BALL",
                                          "BIG RED CYLINDER TOP",
                                          "BIG RED CYLINDER SIDE",
                                          "MEDIUM BLUE CYLINDER TOP",
                                          "MEDIUM BLUE CYLINDER SIDE",
                                          "SMALL RED CYLINDER TOP",
                                          "SMALL RED CYLINDER SIDE",
                                          "PEN",
                                          "SMALL PURPLE CUBE",
                                          "BLUE BOX LARGE SIDE",
                                          "BLUE BOX SMALL SIDE",
                                          "ORANGE BOX LARGE SIDE",
                                          "ORANGE BOX SMALL SIDE",
                                          "RED BOX LARGE SIDE",
                                          "RED BOX SMALL SIDE",
                                          "RED BOX MEDIUM SIDE",
                                          "YELLOW BOX SMALL SIDE",
                                          "YELLOW BOX LARGE SIDE"};

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

  std::vector<std::string> grasp_types = {
      "TRIPOD",         "PALMAR PINCH",       "LATERAL",
      "WRITING TRIPOD", "PARALLEL EXTENSION", "ADDUCTION GRIP",
      "TIP PINCH",      "LATERAL TRIPOD"};

  int idx = 0;
  // Main loop
  std::string line = "";
  while (std::cout << PROMPT) {
    // Process command
    getline(std::cin, line);
    try {
      idx = std::stoi(line);
    } catch (std::invalid_argument) {
      std::cout << "Give number \n";
    }

    gazebo::msgs::Int msg;
    msg.set_data(idx);
    pub->Publish(msg);

    std::vector<double> angles;
    for (unsigned int i = 0; i < data.shape(1); i++) {
      if (joints.at(i).back() == '2' && joints.at(i) != "rh_THJ2")
			/* /1* if (joints.at(i).back() == '2') *1/ */ 
					angles.push_back(2 * data(idx, i) * 3.14 / 180);
			else
					angles.push_back(data(idx, i) * 3.14 / 180);
		}

		api.setJoints(joints, angles);

    /* std::cout << "Shape 0 : " << angles.size() << std::endl; */
    std::cout << "Object Configuration -> "
              << obj_configs.at(labels(idx, 0) - 1) << std::endl;
    std::cout << "Grasp Type -> " << grasp_types.at(labels(idx, 1) - 1)
              << std::endl;
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

