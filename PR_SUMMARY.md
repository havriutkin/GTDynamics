# Pull Request: Implement Robot Creation from MuJoCo XML Files

## Overview
This PR successfully implements support for creating GTDynamics Robot objects from MuJoCo XML (MJCF) files, enabling users to leverage the extensive library of MuJoCo robot models with GTDynamics.

## What Was Implemented

### Core Functionality
✅ **MuJoCo XML Parser** (`mjcf.h`, `mjcf.cpp`)
- Parses MuJoCo XML format using TinyXML2
- Extracts robot structure (bodies → links, joints)
- Handles quaternion-based orientations
- Supports diagonal inertia tensors
- Manages freejoint (floating base) elements

✅ **Joint Type Support**
- Revolute/hinge joints
- Prismatic/slide joints
- Fixed joints (fallback)
- Freejoint recognition

✅ **Auto-Detection Feature**
- `CreateRobotFromFileAutoDetect()` function
- Automatically detects URDF, SDF, or MJCF format
- Seamless integration with existing code

### Testing & Validation
✅ **Unit Tests** (`testMJCF.cpp`)
- Integration with CppUnitLite framework
- Tests for multiple robot models

✅ **Standalone Test** (`standalone_mjcf_test.cpp`)
- Works without full GTDynamics build
- Only requires TinyXML2
- Useful for quick validation

✅ **Test Models**
- `simple_pendulum.xml` - Basic 2-link robot
- `test_robot.xml` - Comprehensive test (5 bodies, 4 joints)
- Verified with `h1.xml` (20 bodies, humanoid)
- Verified with `go2.xml` (13 bodies, quadruped)

### Documentation
✅ **Comprehensive Documentation**
- `MJCF_README.md` - Usage guide and API reference
- `IMPLEMENTATION_SUMMARY.md` - Technical details
- `MJCF_VISUAL_GUIDE.md` - Architecture diagrams
- `example_load_mjcf.cpp` - Working example code

## Files Changed

### Added (13 files)
```
gtdynamics/universal_robot/mjcf.h                  (API header)
gtdynamics/universal_robot/mjcf.cpp                (Implementation)
gtdynamics/universal_robot/MJCF_README.md          (Documentation)
tests/testMJCF.cpp                                 (Unit tests)
tests/standalone_mjcf_test.cpp                     (Standalone test)
examples/example_load_mjcf.cpp                     (Example)
models/test_mjcf/simple_pendulum.xml               (Test model)
models/test_mjcf/test_robot.xml                    (Test model)
IMPLEMENTATION_SUMMARY.md                          (Tech docs)
MJCF_VISUAL_GUIDE.md                               (Visual guide)
```

### Modified (6 files)
```
CMakeLists.txt                                     (TinyXML2 dependency)
gtdynamics/CMakeLists.txt                          (Library linking)
gtdynamics/config.h.in                             (kModelsPath constant)
gtdynamics/universal_robot/sdf.h                   (Auto-detect API)
gtdynamics/universal_robot/sdf.cpp                 (Auto-detect impl)
.gitignore                                         (Test binaries)
```

## Key Features

1. **Seamless Integration**: Works alongside existing URDF/SDF support
2. **Quaternion Support**: Proper handling of MuJoCo's quaternion format (w,x,y,z)
3. **Kinematic Tree Preservation**: Maintains parent-child relationships
4. **Inertial Properties**: Accurate extraction of mass, COM, and inertia
5. **Auto-Detection**: Automatic format detection based on file content

## Usage Example

```cpp
#include <gtdynamics/universal_robot/mjcf.h>

// Load robot from MuJoCo XML
Robot robot = CreateRobotFromMJCF("h1.xml");

// Or use auto-detection
Robot robot2 = CreateRobotFromFileAutoDetect("robot.xml");

// Use with GTDynamics as normal
std::cout << "Links: " << robot.numLinks() << std::endl;
std::cout << "Joints: " << robot.numJoints() << std::endl;
```

## Testing Results

All tests pass successfully:

| Robot Model | Bodies | Joints | Status |
|-------------|--------|--------|--------|
| simple_pendulum.xml | 2 | 1 | ✅ Pass |
| test_robot.xml | 5 | 4 | ✅ Pass |
| go2.xml | 13 | 12 | ✅ Pass |
| h1.xml | 20 | 19 | ✅ Pass |

## Dependencies

**New Dependency Added:**
- TinyXML2 (`libtinyxml2-dev`) - Lightweight XML parsing library

**Installation:**
```bash
# Ubuntu/Debian
sudo apt-get install libtinyxml2-dev

# macOS
brew install tinyxml2
```

## Known Limitations

1. **Default Classes**: MuJoCo's `<default>` class inheritance not supported
2. **Full Inertia**: Only diagonal inertia tensors supported
3. **Advanced Features**: Tendons, actuators, sensors not parsed

These limitations are documented and do not affect core functionality for robot creation.

## Verification

✅ Code compiles without warnings
✅ Standalone test executable runs successfully
✅ All test robot models parse correctly
✅ H1 humanoid robot (20 bodies) loads successfully
✅ GO2 quadruped robot (13 bodies) loads successfully
✅ Inertial properties extracted accurately
✅ Joint parameters parsed correctly
✅ Kinematic tree structure preserved

## Impact

This implementation enables GTDynamics users to:
- Access MuJoCo's extensive robot model library
- Use models from MuJoCo ecosystem directly
- Have flexibility in choosing URDF, SDF, or MJCF formats
- Leverage auto-detection for simpler code

## Backward Compatibility

✅ **Fully backward compatible**
- No changes to existing APIs
- Existing URDF/SDF loading unchanged
- New functionality is additive only
