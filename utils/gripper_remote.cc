/*!
    \file utils/gripper_remote.cc
    \brief TODO

    TODO

    \author João Borrego : jsbruglie
*/

#include "gripper_remote.hh"

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the object spawner topic
    gazebo::transport::PublisherPtr pub =
        node->Advertise<grasp::msgs::Gripper>(GRIPPER_PLUGIN_TOPIC);

    // Wait for a subscriber to connect to this publisher
    pub->WaitForConnection();

    // Main loop
    std::string line = "";

    while (std::cout << PROMPT)
    {
        // Create a custom message
        grasp::msgs::Gripper msg;
        
        // Process command
        getline(std::cin, line);
        std::stringstream input_stream(line);
        std::string command = input_stream.str();
        // Change pose request
        if (command == "pose")
        {
            ignition::math::Pose3d pose(0,0,0.1,0,1.57,0);
            gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
            gazebo::msgs::Set(pose_msg, pose);
            msg.set_allocated_pose(pose_msg);
        }
        // Change velocity request
        else if (command == "velocity")
        {
            ignition::math::Vector3d velocity(0,0,0.8);
            gazebo::msgs::Vector3d *velocity_msg = new gazebo::msgs::Vector3d();
            gazebo::msgs::Set(velocity_msg, velocity);
            msg.set_allocated_velocity(velocity_msg);
        }
        // Open gripper request
        else if (command == "open")
        {
            msg.set_open(true);
        }
        // Close gripper request
        else if (command == "close")
        {
            msg.set_open(false);
        }

        // Send the message
        pub->Publish(msg);
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}