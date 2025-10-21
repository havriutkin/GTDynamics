/* ----------------------------------------------------------------------------
 * Standalone test for MuJoCo XML parsing
 * This demonstrates the parsing functionality without requiring GTSAM/SDFormat
 * -------------------------------------------------------------------------- */

#include <tinyxml2.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <stdexcept>

// Helper function to parse a space-separated string into a vector of doubles
static std::vector<double> parseDoubles(const std::string& str) {
  std::vector<double> result;
  std::istringstream iss(str);
  double val;
  while (iss >> val) {
    result.push_back(val);
  }
  return result;
}

// Simple structure to represent a parsed body
struct ParsedBody {
  std::string name;
  std::string parent_name;
  double pos[3];
  double quat[4];  // w x y z
  double mass;
  double com_pos[3];
  double com_quat[4];
  double inertia[3];
  bool has_joint;
  std::string joint_name;
  std::string joint_type;
  double joint_axis[3];
  double joint_range[2];
  double joint_damping;
};

// Parse position from XML element
static void parsePosition(const tinyxml2::XMLElement* elem, double pos[3], const char* attr = "pos") {
  pos[0] = pos[1] = pos[2] = 0.0;
  const char* posStr = elem->Attribute(attr);
  if (!posStr) return;
  
  auto vals = parseDoubles(posStr);
  if (vals.size() == 3) {
    pos[0] = vals[0];
    pos[1] = vals[1];
    pos[2] = vals[2];
  }
}

// Parse quaternion from XML element (w x y z)
static void parseQuaternion(const tinyxml2::XMLElement* elem, double quat[4], const char* attr = "quat") {
  quat[0] = 1.0; quat[1] = quat[2] = quat[3] = 0.0; // identity
  const char* quatStr = elem->Attribute(attr);
  if (!quatStr) return;
  
  auto vals = parseDoubles(quatStr);
  if (vals.size() == 4) {
    quat[0] = vals[0];
    quat[1] = vals[1];
    quat[2] = vals[2];
    quat[3] = vals[3];
  }
}

// Parse body recursively
static void parseBody(const tinyxml2::XMLElement* bodyElem, 
                     const std::string& parentName,
                     std::vector<ParsedBody>& bodies) {
  ParsedBody body;
  body.has_joint = false;
  body.mass = 1.0;
  body.joint_damping = 0.0;
  
  // Get body name
  const char* nameAttr = bodyElem->Attribute("name");
  if (!nameAttr) {
    throw std::runtime_error("Body element must have a name attribute");
  }
  body.name = nameAttr;
  body.parent_name = parentName;
  
  // Get body position and orientation
  parsePosition(bodyElem, body.pos, "pos");
  parseQuaternion(bodyElem, body.quat, "quat");
  
  // Parse inertial properties
  const tinyxml2::XMLElement* inertialElem = bodyElem->FirstChildElement("inertial");
  if (inertialElem) {
    inertialElem->QueryDoubleAttribute("mass", &body.mass);
    parsePosition(inertialElem, body.com_pos, "pos");
    parseQuaternion(inertialElem, body.com_quat, "quat");
    
    // Parse diagonal inertia
    const char* inertiaStr = inertialElem->Attribute("diaginertia");
    if (inertiaStr) {
      auto vals = parseDoubles(inertiaStr);
      if (vals.size() == 3) {
        body.inertia[0] = vals[0];
        body.inertia[1] = vals[1];
        body.inertia[2] = vals[2];
      }
    }
  }
  
  // Check for joint element
  const tinyxml2::XMLElement* jointElem = bodyElem->FirstChildElement("joint");
  if (jointElem) {
    body.has_joint = true;
    
    const char* jointNameAttr = jointElem->Attribute("name");
    body.joint_name = jointNameAttr ? jointNameAttr : (body.name + "_joint");
    
    const char* typeAttr = jointElem->Attribute("type");
    body.joint_type = typeAttr ? typeAttr : "hinge";
    
    // Parse joint axis
    body.joint_axis[0] = 0; body.joint_axis[1] = 0; body.joint_axis[2] = 1;
    const char* axisStr = jointElem->Attribute("axis");
    if (axisStr) {
      auto vals = parseDoubles(axisStr);
      if (vals.size() == 3) {
        body.joint_axis[0] = vals[0];
        body.joint_axis[1] = vals[1];
        body.joint_axis[2] = vals[2];
      }
    }
    
    // Parse joint range
    body.joint_range[0] = -INFINITY;
    body.joint_range[1] = INFINITY;
    const char* rangeStr = jointElem->Attribute("range");
    if (rangeStr) {
      auto vals = parseDoubles(rangeStr);
      if (vals.size() == 2) {
        body.joint_range[0] = vals[0];
        body.joint_range[1] = vals[1];
      }
    }
    
    jointElem->QueryDoubleAttribute("damping", &body.joint_damping);
  }
  
  bodies.push_back(body);
  
  // Recursively parse child bodies
  for (const tinyxml2::XMLElement* childElem = bodyElem->FirstChildElement("body");
       childElem != nullptr;
       childElem = childElem->NextSiblingElement("body")) {
    parseBody(childElem, body.name, bodies);
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <mjcf_file.xml>" << std::endl;
    return 1;
  }
  
  std::string file_path = argv[1];
  
  try {
    // Load XML file
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
      std::cerr << "Failed to load MJCF file: " << file_path << std::endl;
      return 1;
    }
    
    // Get root mujoco element
    const tinyxml2::XMLElement* mujocoElem = doc.FirstChildElement("mujoco");
    if (!mujocoElem) {
      std::cerr << "MJCF file must have a <mujoco> root element" << std::endl;
      return 1;
    }
    
    // Get model name
    const char* modelAttr = mujocoElem->Attribute("model");
    std::string modelName = modelAttr ? modelAttr : "unknown";
    std::cout << "Model name: " << modelName << std::endl;
    
    // Get worldbody element
    const tinyxml2::XMLElement* worldbodyElem = mujocoElem->FirstChildElement("worldbody");
    if (!worldbodyElem) {
      std::cerr << "MJCF file must have a <worldbody> element" << std::endl;
      return 1;
    }
    
    // Parse all bodies
    std::vector<ParsedBody> bodies;
    
    const tinyxml2::XMLElement* rootBodyElem = worldbodyElem->FirstChildElement("body");
    if (!rootBodyElem) {
      std::cerr << "MJCF worldbody must contain at least one body" << std::endl;
      return 1;
    }
    
    parseBody(rootBodyElem, "", bodies);
    
    // Print results
    std::cout << "\nParsed " << bodies.size() << " bodies:" << std::endl;
    std::cout << "========================================" << std::endl;
    
    for (const auto& body : bodies) {
      std::cout << "\nBody: " << body.name << std::endl;
      std::cout << "  Parent: " << (body.parent_name.empty() ? "(root)" : body.parent_name) << std::endl;
      std::cout << "  Position: [" << body.pos[0] << ", " << body.pos[1] << ", " << body.pos[2] << "]" << std::endl;
      std::cout << "  Quaternion: [" << body.quat[0] << ", " << body.quat[1] << ", " << body.quat[2] << ", " << body.quat[3] << "]" << std::endl;
      std::cout << "  Mass: " << body.mass << std::endl;
      std::cout << "  COM Position: [" << body.com_pos[0] << ", " << body.com_pos[1] << ", " << body.com_pos[2] << "]" << std::endl;
      std::cout << "  Inertia (diag): [" << body.inertia[0] << ", " << body.inertia[1] << ", " << body.inertia[2] << "]" << std::endl;
      
      if (body.has_joint) {
        std::cout << "  Joint: " << body.joint_name << std::endl;
        std::cout << "    Type: " << body.joint_type << std::endl;
        std::cout << "    Axis: [" << body.joint_axis[0] << ", " << body.joint_axis[1] << ", " << body.joint_axis[2] << "]" << std::endl;
        std::cout << "    Range: [" << body.joint_range[0] << ", " << body.joint_range[1] << "]" << std::endl;
        std::cout << "    Damping: " << body.joint_damping << std::endl;
      }
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Successfully parsed MJCF file!" << std::endl;
    
    return 0;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
