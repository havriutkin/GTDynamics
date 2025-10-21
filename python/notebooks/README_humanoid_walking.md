# Humanoid Walking with MuJoCo Example

This directory contains a Jupyter notebook demonstrating a walking humanoid robot using GTDynamics and MuJoCo.

![Humanoid Walking Animation](https://img.shields.io/badge/Status-Ready-brightgreen)
![Python](https://img.shields.io/badge/Python-3.8+-blue)
![MuJoCo](https://img.shields.io/badge/MuJoCo-2.3+-orange)

## Overview

This example demonstrates full kinodynamic trajectory optimization for a bipedal humanoid robot:
- Multi-phase contact dynamics with alternating foot support
- Trajectory optimization with GTDynamics factor graphs
- 3D visualization using MuJoCo physics simulator
- Matplotlib plots for trajectory analysis

## File: `humanoid_walking_mujoco.ipynb`

This notebook demonstrates:
- Loading a humanoid biped robot model
- Setting up multi-phase trajectory optimization for walking
- Solving for optimal walking trajectories using GTDynamics
- Visualizing results with matplotlib and MuJoCo

## Prerequisites

### Required Dependencies
- GTDynamics (with Python bindings)
- GTSAM (with Python bindings)
- NumPy
- Matplotlib

### Optional Dependencies
- MuJoCo (`pip install mujoco`) - for 3D visualization
- Jupyter Notebook or JupyterLab

## Installation

1. Install GTDynamics with Python support (see main README)

2. Install MuJoCo for visualization:
```bash
pip install mujoco
```

3. Install Jupyter:
```bash
pip install jupyter
```

## Running the Notebook

1. Navigate to the notebooks directory:
```bash
cd python/notebooks
```

2. Start Jupyter:
```bash
jupyter notebook
```

3. Open `humanoid_walking_mujoco.ipynb` in the Jupyter interface

4. Run all cells to see the walking humanoid in action!

## What the Notebook Does

1. **Loads the Robot Model**: Uses the biped URDF model (a simple humanoid with two legs)

2. **Defines Walking Gait**: Creates a multi-phase walking pattern with:
   - Double support phases (both feet on ground)
   - Single support phases (one foot swinging)
   - Alternating left and right foot contacts

3. **Optimizes Trajectory**: Solves a constrained optimization problem to find joint trajectories that:
   - Satisfy dynamics equations
   - Maintain contact constraints
   - Keep the body upright
   - Move forward with specified step size

4. **Visualizes Results**:
   - Plots body position over time
   - Plots joint angles over time
   - (Optional) Shows 3D animation in MuJoCo viewer

## Customization

You can modify the example by:
- Changing the step size (forward distance per step)
- Adjusting the number of walk cycles
- Modifying body height or pose targets
- Using different robot models (NAO, Atlas)
- Changing phase durations
- Adding terrain or obstacles

## Troubleshooting

### MuJoCo Visualization Issues

If the MuJoCo visualization doesn't work:
- The notebook will still run and show matplotlib plots
- Some URDF models may not be fully compatible with MuJoCo's URDF parser
- Try installing a newer version of MuJoCo: `pip install --upgrade mujoco`

### Optimization Convergence

If the optimization doesn't converge:
- Try adjusting the noise model parameters (sigma values)
- Reduce the step size
- Decrease the number of walk cycles
- Adjust the phase durations

### Missing Dependencies

If you get import errors:
- Ensure GTDynamics Python bindings are installed (see main README)
- Make sure GTSAM Python bindings are available
- Install missing packages: `pip install numpy matplotlib jupyter`

## Related Examples

- `quadruped.ipynb` - Quadruped walking example
- `spider.ipynb` - Spider robot walking
- `inverse_kinematics.ipynb` - Inverse kinematics example
- `pybullet_sim.ipynb` - PyBullet simulation example

## References

- [GTDynamics Documentation](https://github.com/borglab/GTDynamics)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [GTSAM Documentation](https://gtsam.org/)
