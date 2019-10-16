/*!
    \file examples/hand_remote.cc
    \brief Control manipulator example

    \author João Borrego : jsbruglie
*/

#include "hand_signs.hh"

int main(int _argc, char **_argv)
{
    // Command-line arguments
    std::string config_file, robot;
    parseArgs(_argc, _argv, config_file, robot);

    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Interface
    Interface api;
    // Init interface with config file
    if (!api.init(config_file, robot)) {
        std::cout << "Exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

		/* std::vector<std::string> joints = {"rh_FFJ2", //1 */
		/* 																	 "rh_FFJ2", //2 */
		/* 																	 "rh_FFJ3", //3 */
		/* 																	 "rh_FFJ4", //4 */
		/* 																	 "rh_MFJ2", //5 */
		/* 																	 "rh_MFJ2", //6 */
		/* 																	 "rh_MFJ3", //7 */
		/* 																	 "rh_MFJ4", //8 */
		/* 																	 "rh_RFJ2", //9 */
		/* 																	 "rh_RFJ2", //10 */
		/* 																	 "rh_RFJ3", //11 */
		/* 																	 "rh_RFJ4", //12 */
		/* 																	 "rh_LFJ2", //13 */
		/* 																	 "rh_LFJ2", //14 */
		/* 																	 "rh_LFJ3", //15 */
		/* 																	 "rh_LFJ4", //16 */
		/* 																	 "rh_THJ2", //17 */
		/* 																	 "rh_THJ2", //18 */
		/* 																	 "rh_THJ3", //19 */
		/* 																	 "rh_THJ4", //20 */
		/* 																	 "rh_THJ5"}; //21 */

		std::vector<std::string> joints = {"rh_FFJ4", //1
																			 "rh_LFJ4", //21
																			 "rh_MFJ3", //21
																			 "rh_RFJ3", //21
																			 "rh_MFJ2", //21
																			 "rh_RFJ2", //21
																			 "rh_THJ2", //21
																			 "rh_THJ3", //21
																			 "rh_THJ4", //21
																			 "rh_THJ5"}; //21

		std::vector<double> angles = {-0.2, -0.2, 2.1, 2.1, 0.9, 0.9, 0.9, 1.9, 1.9, 0.5};

		/* std::vector<std::string> joints = {"rh_FFJ4", //1 */
		/* 																	 "rh_LFJ4", //21 */
		/* 																	 "rh_MFJ4", //21 */
		/* 																	 "rh_RFJ4", //21 */
		/* 																	 "rh_THJ5"}; //21 */

		/* std::vector<double> angles = {-0.5, -0.5, 0.1, 0.1, 0.5}; */

		api.setJoints(joints, angles);
    // Main loop
    std::string line = "";
    while (std::cout << PROMPT)
    {
        // Process command
        getline(std::cin, line);
        std::stringstream input_stream(line);
        std::string command = input_stream.str();
				
				if (command == "exit") 
				{
					return 0;
				}
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

//////////////////////////////////////////////////
const std::string getUsage(const char* argv_0)
{
    return \
        "usage:   " + std::string(argv_0) + " [options]\n" +
        "options: -c <robot configuration yaml>\n"  +
        "         -r <robot>\n";
}

//////////////////////////////////////////////////
void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & robot)
{
    int opt;
    bool c, r, g;

    while ((opt = getopt(argc,argv,"c: r:")) != EOF)
    {
        switch (opt)
        {
            case 'c':
                c = true; cfg_dir = optarg;    break;
            case 'r':
                r = true; robot = optarg; break;
            case '?':
                std::cerr << getUsage(argv[0]);
            default:
                std::cout << std::endl;
                exit(EXIT_FAILURE);
        }
    }

    if (!c || !r) {
        std::cerr << getUsage(argv[0]);
        exit(EXIT_FAILURE);
    }

    std::cout << "Parameters:\n" <<
        "   Robot configuration yaml '" << cfg_dir << "'\n" <<
        "   Robot                    '" << robot << "'\n" << std::endl;
}

