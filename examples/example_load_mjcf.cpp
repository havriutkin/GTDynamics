/* ----------------------------------------------------------------------------
 * Example: Loading a robot from MuJoCo XML file
 * This example demonstrates how to create a Robot from a MJCF file
 * -------------------------------------------------------------------------- */

#include <gtdynamics/universal_robot/mjcf.h>
#include <gtdynamics/universal_robot/sdf.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <iostream>

using namespace gtdynamics;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_mjcf_file.xml>" << std::endl;
    return 1;
  }
  
  std::string mjcf_path = argv[1];
  
  try {
    // Load robot from MuJoCo XML file
    std::cout << "Loading robot from: " << mjcf_path << std::endl;
    Robot robot = CreateRobotFromMJCF(mjcf_path);
    
    // Print robot information
    std::cout << "\nRobot loaded successfully!" << std::endl;
    std::cout << "Number of links: " << robot.numLinks() << std::endl;
    std::cout << "Number of joints: " << robot.numJoints() << std::endl;
    
    // List all links
    std::cout << "\nLinks:" << std::endl;
    for (const auto& link : robot.links()) {
      std::cout << "  - " << link->name() 
                << " (mass: " << link->mass() << ")" << std::endl;
    }
    
    // List all joints
    std::cout << "\nJoints:" << std::endl;
    for (const auto& joint : robot.joints()) {
      std::cout << "  - " << joint->name() 
                << " (type: ";
      switch (joint->type()) {
        case Joint::Type::Revolute: std::cout << "revolute"; break;
        case Joint::Type::Prismatic: std::cout << "prismatic"; break;
        case Joint::Type::Fixed: std::cout << "fixed"; break;
        case Joint::Type::Helical: std::cout << "helical"; break;
        default: std::cout << "unknown"; break;
      }
      std::cout << ")" << std::endl;
    }
    
    // Alternative: Use auto-detect function
    std::cout << "\n\nTesting auto-detect function..." << std::endl;
    Robot robot2 = CreateRobotFromFileAutoDetect(mjcf_path);
    std::cout << "Auto-detect successful! Loaded " << robot2.numLinks() 
              << " links and " << robot2.numJoints() << " joints." << std::endl;
    
    return 0;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
