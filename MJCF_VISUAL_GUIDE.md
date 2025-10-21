# MuJoCo XML Support - Visual Overview

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GTDynamics Robot Creation                 │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
         ┌────────────────────────────────────────┐
         │   CreateRobotFromFileAutoDetect()      │
         │   (Auto-detects file format)           │
         └────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
          ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐        ┌─────────┐
    │  .urdf  │         │  .sdf   │        │  .xml   │
    │  Files  │         │  Files  │        │ (MJCF)  │
    └─────────┘         └─────────┘        └─────────┘
          │                   │                   │
          ▼                   ▼                   ▼
    ┌─────────┐         ┌─────────┐        ┌─────────┐
    │SDFormat │         │SDFormat │        │TinyXML2 │
    │ Parser  │         │ Parser  │        │ Parser  │
    └─────────┘         └─────────┘        └─────────┘
          │                   │                   │
          └───────────────────┼───────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │   Robot Object   │
                    │  - Links (Bodies)│
                    │  - Joints        │
                    │  - Kinematic Tree│
                    └──────────────────┘
```

## MJCF Parsing Pipeline

```
MuJoCo XML File (*.xml)
         │
         ▼
┌────────────────────┐
│ Parse <mujoco>     │ → Extract model name
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ Parse <worldbody>  │ → Find root body
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ Parse <body> tree  │ → Recursive traversal
│   - Position/Quat  │
│   - Inertial props │
│   - Joint spec     │
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ Create Links       │ → GTDynamics Link objects
│   - Mass           │
│   - Inertia        │
│   - COM frame      │
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ Create Joints      │ → GTDynamics Joint objects
│   - RevoluteJoint  │   (hinge type)
│   - PrismaticJoint │   (slide type)
│   - FixedJoint     │   (fallback)
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ Build Robot        │ → Complete Robot object
│   - Link map       │
│   - Joint map      │
│   - Connectivity   │
└────────────────────┘
```

## Supported MuJoCo Elements

### ✅ Fully Supported
- `<mujoco>` - Root element
- `<worldbody>` - World container
- `<body>` - Robot links
  - `name` attribute
  - `pos` attribute (x y z)
  - `quat` attribute (w x y z)
- `<inertial>` - Mass properties
  - `mass` attribute
  - `pos` attribute
  - `quat` attribute  
  - `diaginertia` attribute
- `<joint>` - Joint connections
  - `name` attribute
  - `type` attribute (hinge, slide)
  - `axis` attribute (x y z)
  - `range` attribute (min max)
  - `damping` attribute
- `<freejoint>` - Floating base indicator

### ⚠️ Partially Supported
- `<joint type="...">` - Only hinge and slide types
- Inertia - Only diagonal tensors (not full 3x3)

### ❌ Not Supported
- `<default>` - Class inheritance system
- `<actuator>` - Actuator specifications
- `<sensor>` - Sensor definitions
- `<tendon>` - Tendon mechanisms
- `<contact>` - Contact parameters
- `<geom>` - Geometry details (parsed but not used)
- Ball/spherical joints

## Example Workflow

```cpp
// 1. Load robot from MJCF file
Robot h1_robot = CreateRobotFromMJCF("h1.xml");

// 2. Inspect structure
std::cout << "Bodies: " << h1_robot.numLinks() << std::endl;  // 20
std::cout << "Joints: " << h1_robot.numJoints() << std::endl; // 19

// 3. Access specific elements
auto pelvis = h1_robot.link("pelvis");
auto hip_joint = h1_robot.joint("left_hip_yaw_joint");

// 4. Use with GTDynamics algorithms
Values initial_values = ...;
auto fk_results = h1_robot.forwardKinematics(initial_values);

// 5. Build factor graphs
Graph graph = ...;
// Add dynamics factors, constraints, etc.
```

## Test Coverage

| Test File | Bodies | Joints | Status |
|-----------|--------|--------|--------|
| simple_pendulum.xml | 2 | 1 | ✅ Pass |
| test_robot.xml | 5 | 4 | ✅ Pass |
| go2.xml | 13 | 12 | ✅ Pass |
| h1.xml | 20 | 19 | ✅ Pass |

## Comparison with SDF/URDF

| Feature | URDF | SDF | MJCF (New) |
|---------|------|-----|------------|
| File Extension | .urdf | .sdf | .xml |
| Parser Library | SDFormat | SDFormat | TinyXML2 |
| Rotation Format | RPY | Quaternion | Quaternion |
| Inertia Format | Full matrix | Full matrix | Diagonal only |
| Joint in | `<joint>` | `<joint>` | `<body>` child |
| Base Frame | Fixed | World/Custom | Freejoint |
| Auto-detect | ✅ | ✅ | ✅ |

## Benefits

1. **Expanded Model Library**: Access to MuJoCo's extensive robot model collection
2. **Easy Integration**: Seamless integration with existing GTDynamics workflows
3. **Format Flexibility**: Support for URDF, SDF, and MJCF formats
4. **Auto-detection**: Automatic format detection simplifies user code
5. **Lightweight**: TinyXML2 dependency is small and widely available
