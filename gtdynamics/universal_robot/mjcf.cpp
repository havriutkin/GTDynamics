/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file mjcf.cpp
 * @brief Implementation of MuJoCo XML (MJCF) related utilities.
 * @author GTDynamics Contributors
 */

#include <gtdynamics/universal_robot/mjcf.h>
#include <gtdynamics/universal_robot/FixedJoint.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/universal_robot/PrismaticJoint.h>
#include <gtdynamics/universal_robot/Link.h>

#include <tinyxml2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <fstream>
#include <map>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <cmath>

namespace gtdynamics {

using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Point3;
using gtsam::Vector3;
using gtsam::Vector6;
using gtsam::Matrix3;

/**
 * Helper function to parse a space-separated string into a vector of doubles
 */
static std::vector<double> parseDoubles(const std::string& str) {
  std::vector<double> result;
  std::istringstream iss(str);
  double val;
  while (iss >> val) {
    result.push_back(val);
  }
  return result;
}

/**
 * Helper function to parse position attribute (3 values: x y z)
 */
static Point3 parsePosition(const tinyxml2::XMLElement* elem, const char* attr = "pos") {
  const char* posStr = elem->Attribute(attr);
  if (!posStr) return Point3(0, 0, 0);
  
  auto vals = parseDoubles(posStr);
  if (vals.size() != 3) {
    throw std::runtime_error("Position attribute must have 3 values");
  }
  return Point3(vals[0], vals[1], vals[2]);
}

/**
 * Helper function to parse quaternion attribute (4 values: w x y z)
 * MuJoCo uses wxyz format
 */
static Rot3 parseQuaternion(const tinyxml2::XMLElement* elem, const char* attr = "quat") {
  const char* quatStr = elem->Attribute(attr);
  if (!quatStr) return Rot3();  // Identity rotation
  
  auto vals = parseDoubles(quatStr);
  if (vals.size() != 4) {
    throw std::runtime_error("Quaternion attribute must have 4 values");
  }
  // MuJoCo uses w x y z format
  return Rot3(gtsam::Quaternion(vals[0], vals[1], vals[2], vals[3]));
}

/**
 * Helper function to parse axis attribute (3 values: x y z)
 */
static Vector3 parseAxis(const tinyxml2::XMLElement* elem, const char* attr = "axis") {
  const char* axisStr = elem->Attribute(attr);
  if (!axisStr) return Vector3(0, 0, 1);  // Default Z axis
  
  auto vals = parseDoubles(axisStr);
  if (vals.size() != 3) {
    throw std::runtime_error("Axis attribute must have 3 values");
  }
  return Vector3(vals[0], vals[1], vals[2]);
}

/**
 * Helper function to parse diagonal inertia (3 values: Ixx Iyy Izz)
 */
static Matrix3 parseDiagInertia(const tinyxml2::XMLElement* elem, const char* attr = "diaginertia") {
  const char* inertiaStr = elem->Attribute(attr);
  if (!inertiaStr) {
    // Return small default inertia
    Matrix3 I;
    I << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    return I;
  }
  
  auto vals = parseDoubles(inertiaStr);
  if (vals.size() != 3) {
    throw std::runtime_error("Diagonal inertia must have 3 values");
  }
  
  Matrix3 I;
  I << vals[0], 0, 0, 0, vals[1], 0, 0, 0, vals[2];
  return I;
}

/**
 * Parse inertial properties from a MuJoCo body element
 */
struct InertialData {
  double mass;
  Point3 com_pos;
  Rot3 com_rot;
  Matrix3 inertia;
  
  InertialData() : mass(0.0), com_pos(0, 0, 0), com_rot(), inertia(Matrix3::Zero()) {}
};

static InertialData parseInertial(const tinyxml2::XMLElement* bodyElem) {
  InertialData data;
  
  const tinyxml2::XMLElement* inertialElem = bodyElem->FirstChildElement("inertial");
  if (!inertialElem) {
    // Default inertial properties
    data.mass = 1.0;
    data.inertia << 0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001;
    return data;
  }
  
  // Parse mass
  inertialElem->QueryDoubleAttribute("mass", &data.mass);
  
  // Parse COM position and orientation
  data.com_pos = parsePosition(inertialElem, "pos");
  data.com_rot = parseQuaternion(inertialElem, "quat");
  
  // Parse inertia
  data.inertia = parseDiagInertia(inertialElem, "diaginertia");
  
  return data;
}

/**
 * Structure to hold body (link) information during parsing
 */
struct BodyData {
  std::string name;
  Point3 pos;
  Rot3 rot;
  InertialData inertial;
  std::string parent_name;
  bool has_joint;
  std::string joint_name;
  std::string joint_type;
  Vector3 joint_axis;
  double joint_lower_limit;
  double joint_upper_limit;
  double joint_damping;
  
  BodyData() : pos(0, 0, 0), rot(), has_joint(false), 
               joint_lower_limit(-std::numeric_limits<double>::infinity()),
               joint_upper_limit(std::numeric_limits<double>::infinity()),
               joint_damping(0.0) {}
};

/**
 * Recursively parse body elements from MJCF
 */
static void parseBody(const tinyxml2::XMLElement* bodyElem, 
                     const std::string& parentName,
                     std::vector<BodyData>& bodies,
                     const Point3& parentPos = Point3(0, 0, 0),
                     const Rot3& parentRot = Rot3()) {
  BodyData body;
  
  // Get body name
  const char* nameAttr = bodyElem->Attribute("name");
  if (!nameAttr) {
    throw std::runtime_error("Body element must have a name attribute");
  }
  body.name = nameAttr;
  body.parent_name = parentName;
  
  // Get body position and orientation relative to parent
  Point3 localPos = parsePosition(bodyElem, "pos");
  Rot3 localRot = parseQuaternion(bodyElem, "quat");
  
  // Compute absolute position and rotation
  body.pos = parentPos + parentRot * localPos;
  body.rot = parentRot * localRot;
  
  // Parse inertial properties
  body.inertial = parseInertial(bodyElem);
  
  // Check for freejoint element (indicates base link with 6 DOF)
  const tinyxml2::XMLElement* freeJointElem = bodyElem->FirstChildElement("freejoint");
  if (freeJointElem) {
    // This is a floating base - no joint to parent
    body.has_joint = false;
  } else {
    // Check for regular joint element
    const tinyxml2::XMLElement* jointElem = bodyElem->FirstChildElement("joint");
    if (jointElem) {
      body.has_joint = true;
      
      // Get joint name
      const char* jointNameAttr = jointElem->Attribute("name");
      if (jointNameAttr) {
        body.joint_name = jointNameAttr;
      } else {
        body.joint_name = body.name + "_joint";
      }
      
      // Determine joint type (default is hinge/revolute)
      const char* typeAttr = jointElem->Attribute("type");
      if (typeAttr) {
        body.joint_type = typeAttr;
      } else {
        body.joint_type = "hinge";  // Default in MuJoCo
      }
      
      // Parse joint axis
      body.joint_axis = parseAxis(jointElem, "axis");
      
      // Parse joint range
      const char* rangeStr = jointElem->Attribute("range");
      if (rangeStr) {
        auto vals = parseDoubles(rangeStr);
        if (vals.size() == 2) {
          body.joint_lower_limit = vals[0];
          body.joint_upper_limit = vals[1];
        }
      }
      
      // Parse damping
      jointElem->QueryDoubleAttribute("damping", &body.joint_damping);
    }
  }
  
  // Add this body to the list
  bodies.push_back(body);
  
  // Recursively parse child bodies
  for (const tinyxml2::XMLElement* childElem = bodyElem->FirstChildElement("body");
       childElem != nullptr;
       childElem = childElem->NextSiblingElement("body")) {
    parseBody(childElem, body.name, bodies, body.pos, body.rot);
  }
}

Robot CreateRobotFromMJCF(const std::string &file_path,
                          const std::string &model_name) {
  // Load XML file
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(file_path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load MJCF file: " + file_path);
  }
  
  // Get root mujoco element
  const tinyxml2::XMLElement* mujocoElem = doc.FirstChildElement("mujoco");
  if (!mujocoElem) {
    throw std::runtime_error("MJCF file must have a <mujoco> root element");
  }
  
  // Get worldbody element
  const tinyxml2::XMLElement* worldbodyElem = mujocoElem->FirstChildElement("worldbody");
  if (!worldbodyElem) {
    throw std::runtime_error("MJCF file must have a <worldbody> element");
  }
  
  // Parse all bodies
  std::vector<BodyData> bodies;
  
  // Find the root body (first body in worldbody)
  const tinyxml2::XMLElement* rootBodyElem = worldbodyElem->FirstChildElement("body");
  if (!rootBodyElem) {
    throw std::runtime_error("MJCF worldbody must contain at least one body");
  }
  
  // Parse root body and all its children
  parseBody(rootBodyElem, "", bodies);
  
  // Create links
  LinkMap links;
  uint8_t link_id = 0;
  
  for (const auto& bodyData : bodies) {
    // Calculate bMcom: transform from base frame to COM frame
    Pose3 bMlink(bodyData.rot, bodyData.pos);
    Pose3 lMcom(bodyData.inertial.com_rot, bodyData.inertial.com_pos);
    Pose3 bMcom = bMlink * lMcom;
    
    auto link = std::make_shared<Link>(
        link_id++,
        bodyData.name,
        bodyData.inertial.mass,
        bodyData.inertial.inertia,
        bMcom,
        bMlink
    );
    
    links[bodyData.name] = link;
  }
  
  // Create joints
  JointMap joints;
  uint8_t joint_id = 0;
  
  for (const auto& bodyData : bodies) {
    if (!bodyData.has_joint) continue;
    
    // Skip if this is the root body (no parent)
    if (bodyData.parent_name.empty()) continue;
    
    // Get parent and child links
    auto parent_link = links[bodyData.parent_name];
    auto child_link = links[bodyData.name];
    
    // Joint frame is at the child link's position
    Pose3 bMj(bodyData.rot, bodyData.pos);
    
    // Create joint parameters
    JointParams params;
    params.scalar_limits.value_lower_limit = bodyData.joint_lower_limit;
    params.scalar_limits.value_upper_limit = bodyData.joint_upper_limit;
    params.damping_coefficient = bodyData.joint_damping;
    params.effort_type = JointEffortType::Actuated;
    
    JointSharedPtr joint;
    
    // Create appropriate joint type
    if (bodyData.joint_type == "hinge" || bodyData.joint_type == "revolute") {
      joint = std::make_shared<RevoluteJoint>(
          joint_id++,
          bodyData.joint_name,
          bMj,
          parent_link,
          child_link,
          bodyData.joint_axis,
          params
      );
    } else if (bodyData.joint_type == "slide" || bodyData.joint_type == "prismatic") {
      joint = std::make_shared<PrismaticJoint>(
          joint_id++,
          bodyData.joint_name,
          bMj,
          parent_link,
          child_link,
          bodyData.joint_axis,
          params
      );
    } else if (bodyData.joint_type == "free" || bodyData.joint_type == "freejoint") {
      // For free joints, we don't create a joint object
      // The link is effectively the base link with 6 DOF
      continue;
    } else {
      // For unsupported joint types, create a fixed joint
      joint = std::make_shared<FixedJoint>(
          joint_id++,
          bodyData.joint_name,
          bMj,
          parent_link,
          child_link
      );
    }
    
    if (joint) {
      joints[bodyData.joint_name] = joint;
      parent_link->addJoint(joint);
      child_link->addJoint(joint);
    }
  }
  
  return Robot(links, joints);
}

}  // namespace gtdynamics
