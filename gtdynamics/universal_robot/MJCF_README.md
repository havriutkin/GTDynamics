# MuJoCo XML (MJCF) Support in GTDynamics

This directory contains the implementation for loading robot models from MuJoCo XML (MJCF) files.

## Overview

GTDynamics now supports creating `Robot` objects from MuJoCo XML files in addition to URDF and SDF formats. This allows users to leverage the extensive library of MuJoCo robot models available in the robotics community.

## Features

- Parse MuJoCo XML files (.xml format)
- Extract robot structure (bodies/links and joints)
- Support for inertial properties with quaternion orientations
- Support for diagonal inertia tensors
- Support for revolute (hinge) and prismatic (slide) joints
- Automatic file format detection (MJCF vs SDF/URDF)

## Usage

### Direct MJCF Loading

```cpp
#include <gtdynamics/universal_robot/mjcf.h>

// Load a robot from a MuJoCo XML file
Robot robot = CreateRobotFromMJCF("path/to/robot.xml");

// Use the robot as normal
std::cout << "Loaded " << robot.numLinks() << " links" << std::endl;
std::cout << "Loaded " << robot.numJoints() << " joints" << std::endl;
```

### Auto-detection

```cpp
#include <gtdynamics/universal_robot/sdf.h>

// Automatically detect format and load (supports URDF, SDF, or MJCF)
Robot robot = CreateRobotFromFileAutoDetect("path/to/robot.xml");
```

## MuJoCo XML Format Support

The parser supports the following MJCF features:

### Body (Link) Elements
- `name`: Body name
- `pos`: Position relative to parent (x y z)
- `quat`: Orientation quaternion (w x y z format)

### Inertial Properties
- `mass`: Link mass
- `pos`: Center of mass position
- `quat`: Center of mass orientation
- `diaginertia`: Diagonal inertia tensor (Ixx Iyy Izz)

### Joint Elements
- `name`: Joint name
- `type`: Joint type (`hinge`, `slide`, `free`, etc.)
- `axis`: Joint axis (x y z)
- `range`: Joint limits (lower upper)
- `damping`: Joint damping coefficient

## Current Limitations

1. **Default Classes**: The parser does not currently support MuJoCo's `<default>` class system for inheriting properties. All joint properties should be explicitly specified in each joint element.

2. **Full Inertia Matrices**: Only diagonal inertia matrices are currently supported via the `diaginertia` attribute. Full 3x3 inertia matrices specified with individual `ixx`, `ixy`, etc. attributes are not yet supported.

3. **Free Joints**: Free joints (6-DOF) are recognized but not converted to joint objects in the GTDynamics representation. The link is treated as a base link.

4. **Advanced Features**: Advanced MuJoCo features like tendons, actuators, sensors, and contact parameters are not parsed.

## Example Models

Test the implementation with the provided robot models:

- `models/h1_description/mjcf/h1.xml` - Humanoid robot with explicit joint axes
- `models/go2_description/mjcf/go2.xml` - Quadruped robot
- `models/test_mjcf/simple_pendulum.xml` - Simple test model

## Testing

A standalone test program is provided to verify MJCF parsing without requiring the full GTDynamics build:

```bash
cd tests
g++ -std=c++17 -o standalone_mjcf_test standalone_mjcf_test.cpp -ltinyxml2
./standalone_mjcf_test ../models/h1_description/mjcf/h1.xml
```

## Implementation Details

The MJCF parser is implemented in:
- `gtdynamics/universal_robot/mjcf.h` - Public API
- `gtdynamics/universal_robot/mjcf.cpp` - Implementation using TinyXML2

The parser converts MuJoCo bodies to GTDynamics Links and MuJoCo joints to GTDynamics Joints, maintaining the kinematic tree structure.

## Dependencies

- TinyXML2 library for XML parsing
- Standard GTDynamics dependencies (GTSAM, etc.)

Install TinyXML2:
```bash
# Ubuntu/Debian
sudo apt-get install libtinyxml2-dev

# macOS
brew install tinyxml2
```
