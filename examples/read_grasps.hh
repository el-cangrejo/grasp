/*!
    \file examples/hand_remote.hh
    \brief Control manipulator example

    Use keyboard to control robotic manipulator spawned in simulation.

    \author João Borrego : jsbruglie
*/

#ifndef _HAND_REMOTE_HH
#define _HAND_REMOTE_HH

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// xtensor - for reading npy arrays
#include "xtensor/xarray.hpp"
#include <xtensor/xio.hpp>
#include "xtensor/xnpy.hpp"

#include <stdexcept>

// Interface
#include "Interface.hh"

/// Command prompt
#define PROMPT "> "

// Functions

/// \brief Returns usage string.
/// \param argv_0 Executable name
/// \return Usage string.
const std::string getUsage(const char* argv_0);

/// \brief Parses command-line arguments
/// \param argc Argument count
/// \param argv Argument values
/// \param cfg_dir Robot configuration file
/// \param robot Robot name 
void parseArgs(
    int argc,
    char** argv,
    std::string & cfg_dir,
    std::string & grasp_file,
    std::string & robot);

#endif
