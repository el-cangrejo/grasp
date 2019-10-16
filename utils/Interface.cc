/*!
    \file utils/Interface.cc
    \brief Grasp interface

    \author João Borrego : jsbruglie
*/

#include "Interface.hh"

//////////////////////////////////////////////////
Interface::Interface()
{
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    pub = node->Advertise<grasp::msgs::Hand>(HAND_PLUGIN_TOPIC);
    pub->WaitForConnection();

    std::vector<double> values;

    // Initial configuration

    joints = {
        "virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
        "virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint"
    };
    values = {
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
    };

    setJoints(joints, values);
}

//////////////////////////////////////////////////
bool Interface::init(
    const std::string & config_file,
    const std::string & robot)
{
    std::string grasp_name, joint;
    double pre_value, post_value, force_value; 

    try
    {
        YAML::Node config = YAML::LoadFile(config_file);
        // Read robot name
        robot_name = config[robot]["name"].as<std::string>();
        // Read grasp configurations
        YAML::Node grasp_shapes = config[robot]["grasp_configurations"];

        for (auto const & grasp : grasp_shapes)
        {
            grasp_name = grasp.first.as<std::string>();
            grasps.emplace_back(grasp_name);
            YAML::Node joint_value_pairs =
                config[robot]["grasp_configurations"][grasp_name];            
            for (auto const & pair : joint_value_pairs)
            {
                joint = pair.first.as<std::string>();
                pre_value = pair.second["pre"].as<double>();
                post_value = pair.second["post"].as<double>();
                if (pair.second["force"])
                {
                    force_value = pair.second["force"].as<double>();
                    grasps.back().forces.emplace_back(joint, force_value);
                }
                grasps.back().pre.emplace_back(joint, pre_value);
                grasps.back().post.emplace_back(joint, post_value);
            }
            debugPrintTrace("Grasp configuration " << grasps.back().name << 
                " : " << grasps.back().pre.size() << " joints.");
        }
        // Read transformation matrix from pose
        std::vector<double> p = config[robot]["pose"].as<std::vector<double>>();
        ignition::math::Pose3d pose(p.at(0),p.at(1),p.at(2),p.at(3),p.at(4),p.at(5));
        t_base_gripper = ignition::math::Matrix4d(pose);
    }
    catch (YAML::Exception& yamlException)
    {
        errorPrintTrace("Unable to parse " << config_file);
        return false;
    }
    return true;
}

/////////////////////////////////////////////////
ignition::math::Matrix4d Interface::getTransformBaseGripper()
{
    return t_base_gripper;
}

/////////////////////////////////////////////////
std::string Interface::getRobotName()
{
    return robot_name;
}

/////////////////////////////////////////////////
void Interface::setPose(ignition::math::Pose3d pose,
    double timeout)
{
    grasp::msgs::Hand msg;
    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    msg.set_allocated_pose(pose_msg);
    if (timeout > 0) {
        msg.set_timeout(timeout);
    }
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void Interface::reset()
{
    grasp::msgs::Hand msg;
    msg.set_reset(true);
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void Interface::openFingers(double timeout, bool set_position)
{
    grasp::msgs::Hand msg;
    // TODO allow grasp to be chosen
    GraspShape grasp = grasps.back();
    for (auto const & pair : grasp.pre)
    {
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(POSITION);
        target->set_joint(pair.first);
        target->set_value(pair.second);
    }
    if (timeout > 0)    { msg.set_timeout(timeout); }
    if (set_position)   { msg.set_force_target(true); }
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void Interface::closeFingers(double timeout, bool apply_force)
{
    grasp::msgs::Hand msg;
    // TODO allow grasp to be chosen
    GraspShape grasp = grasps.back();
    
    if (apply_force)
    {
        if (grasp.forces.empty())
        {
            debugPrintTrace("No force values for closing fingers specified in config."); 
        }
        for (auto const & pair : grasp.forces)
        {
            grasp::msgs::Target *target = msg.add_pid_targets();
            target->set_joint(pair.first);
            target->set_type(FORCE);
            target->set_value(pair.second);   
        }
    }
    else
    {
        for (auto const & pair : grasp.post)
        {
            grasp::msgs::Target *target = msg.add_pid_targets();
            target->set_joint(pair.first);
            target->set_type(POSITION);
            target->set_value(pair.second);   
        }
    }

    if (timeout > 0) { msg.set_timeout(timeout); }
    pub->Publish(msg);
}

/////////////////////////////////////////////////
void Interface::moveUp(double timeout)
{
    grasp::msgs::Hand msg;
    std::string virtual_joint = "virtual_pz_joint";
    double value = {0.7};
		state[virtual_joint] = value;
		grasp::msgs::Target *target = msg.add_pid_targets();
		target->set_type(POSITION);
		target->set_joint(virtual_joint);
		target->set_value(state[virtual_joint]);
    if (timeout > 0) { msg.set_timeout(timeout); }
    pub->Publish(msg);
}

void Interface::getPose()
{
	std::vector<std::string> virtual_joints = {
        "virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
        "virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint"
	};

	for (auto& joint : virtual_joints)
		std::cout << joint << ": " << state[joint] << std::endl;
}

/////////////////////////////////////////////////
void Interface::disturbHand()
{
    const char* virtual_joint_x = "virtual_px_joint";
    const char* virtual_joint_y = "virtual_py_joint";
    const char* virtual_joint_z = "virtual_pz_joint";
		int delay = 1000;

	// Disturb along x
		moveJoint(virtual_joint_x,  0.055);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		moveJoint(virtual_joint_x,  -0.110);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		moveJoint(virtual_joint_x,  0.055);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
	// Disturb along y
		moveJoint(virtual_joint_y,  0.055);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		moveJoint(virtual_joint_y,  -0.110);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		moveJoint(virtual_joint_y,  0.055);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
	// Disturb along z
		moveJoint(virtual_joint_z,  0.055);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		moveJoint(virtual_joint_z,  -0.110);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
		moveJoint(virtual_joint_z,  0.055);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}

/////////////////////////////////////////////////
void Interface::raiseHand(double timeout)
{
    grasp::msgs::Hand msg;
    std::vector<std::string> virtual_joints;
    std::vector<double> values;

    // TODO - Read virtual joints from config file
    virtual_joints = {
        "virtual_px_joint","virtual_py_joint", "virtual_pz_joint",
        "virtual_rr_joint","virtual_rp_joint", "virtual_ry_joint"
    };
    values = {0,0,0.6,0,0,0};

    for (unsigned int i = 0; i < virtual_joints.size(); i++)
    {
        grasp::msgs::Target *target = msg.add_pid_targets();
        target->set_type(POSITION);
        target->set_joint(virtual_joints.at(i));
        target->set_value(values.at(i));
    }
    if (timeout > 0) { msg.set_timeout(timeout); }
    pub->Publish(msg);
}

//////////////////////////////////////////////////
void Interface::loop()
{
    bool loop = true;
    char key = -1;

    while (loop)
    {
        // Keyboard input
        if (kbhit()) {
            key = getchar();
            if (key == KEY_EXIT) {
                loop = false;
            } else {
                processKeypress(key);
            }
        }
    }

    return;
}

//////////////////////////////////////////////////
int Interface::processKeypress(char key)
{
    switch (key)
    {
        // Translate base
        case 'w':
            moveJoint("virtual_px_joint",  0.015); break;
        case 's':
            moveJoint("virtual_px_joint", -0.015); break;
        case 'a':
            moveJoint("virtual_py_joint",  0.015); break;
        case 'd':
            moveJoint("virtual_py_joint", -0.015); break;
        case 'e':
            moveJoint("virtual_pz_joint",  0.015); break;
        case 'q':
            moveJoint("virtual_pz_joint", -0.015); break;
        // Rotate base
        case 'i':
            moveJoint("virtual_rr_joint",  0.015); break;
        case 'k':
            moveJoint("virtual_rr_joint", -0.015); break;
        case 'j':
            moveJoint("virtual_rp_joint",  0.015); break;
        case 'l':
            moveJoint("virtual_rp_joint", -0.015); break;
        case 'u':
            moveJoint("virtual_ry_joint",  0.015); break;
        case 'o':
            moveJoint("virtual_ry_joint", -0.015); break;
        // Move fingers
        case 'r':
            openFingers(); break;
        case 'f':
            closeFingers(); break;
        case 'y':
						moveUp(); break;
        case 't':
						disturbHand(); break;
        default:
            break;
    }
    return 0;
}

//////////////////////////////////////////////////
void Interface::moveJoint(const char *joint, double value)
{
    state[joint] += value;
    grasp::msgs::Hand msg;
    grasp::msgs::Target *target = msg.add_pid_targets();
    target->set_type(POSITION);
    target->set_joint(joint);
    target->set_value(state[joint]);
    pub->Publish(msg);
}

//////////////////////////////////////////////////
void Interface::setJoints(
    std::vector<std::string> & joints,
    std::vector<double> & values)
{
    if (joints.size() == values.size())
    {
        grasp::msgs::Hand msg;
        for (unsigned int i = 0; i < joints.size(); i++)
        {
            const char *joint = joints.at(i).c_str();
            state[joint] = values.at(i);
            grasp::msgs::Target *target = msg.add_pid_targets();
            target->set_type(POSITION);
            target->set_joint(joint);
            target->set_value(values.at(i));
        }
        pub->Publish(msg);
    }
}
