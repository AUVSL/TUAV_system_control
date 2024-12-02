# Tethered Quadcopter Simulation

This repository contains MATLAB code for simulating the nonlinear control of a quadcopter tethered to a ground-based winder. The system uses a backstepping control technique and simulates the UAV moving through predefined waypoints while incorporating the dynamics of a tether.

## Features

- **Quadcopter Simulation**: Models the dynamics of a quadcopter including its position, orientation, and motion.
- **Tether Dynamics**: Simulates the behavior of the tether using catenary and spring-damper models to capture realistic tether motion and wiggling effects.
- **Nonlinear Control**: Implements a backstepping control method to ensure stability and waypoint tracking.
- **Customizable Parameters**: Provides options to modify quadcopter, tether, and winder characteristics.
- **3D Visualization**: Animates the quadcopter's trajectory along with tether dynamics.

![Quadcopter Animation](./Animation/quadcopter_animation.gif)

---

## Getting Started

### Prerequisites

- MATLAB (R2020b or later is recommended)
- MATLAB's ODE solver (e.g., `ode45`)

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/username/tethered-quadcopter.git
   cd tethered-quadcopter
2. Open MATLAB and set the cloned directory as your working folder.

## Usage

### Running the Simulation
1. Open the main.m script.
2. Modify the parameters in the main.m file as needed:
  - Quadcopter parameters: Mass, moments of inertia, etc.
  - Tether parameters: Density, cross-sectional area, initial tension, etc.
  - Winder parameters: Moment of inertia, radius, etc.
  - Controller gains: Adjust the gains (k1, k2, ..., k14) for tuning.
  - Waypoints: Define the UAV's desired trajectory.
3. Run the script by typing:
```bash
  main
```
4. Watch the 3D animation of the quadcopter and tether in real-time.


## Files Overview
- main.m: Main script that initializes parameters, solves the system's dynamics using ode45, and visualizes the simulation.
- draw_drone.m: Helper function to visualize the quadcopter in 3D with its propellers and body axes.
- get_rotation.m: Calculates the rotation matrix for transforming the quadcopter's orientation.
- dynamics.m: Defines the system's ordinary differential equations (ODEs) for the quadcopter, tether, and winder.
- cableDynamics.m: Simulates the internal dynamics of the tether using a spring-damper model.

## Output
### Visualization
  - Quadcopter Trajectory: Displays the UAV's movement and tether dynamics in a 3D plot.
  - Waypoints: Shows predefined positions that the quadcopter tracks during the simulation.
  - Trail: Illustrates the path followed by the UAV.

### Data
Simulation results for position, orientation, tether length, and other state variables are available in the workspace for post-processing.

## Customization
  - Modify the control gains (k1 to k14) in the main.m script for different stability and tracking behaviors.
  - Change the tether parameters (e.g., length, density) to experiment with various configurations.
  - Adjust the number of masses (n) in the tether model for finer resolution or faster simulation.

## Limitations
- The simulation assumes ideal environmental conditions with simplified wind disturbances.
- Cable dynamics are approximated using a spring-damper model and may not fully capture real-world tether behavior.

## Acknowledgments
If you find this useful, consider citing our paper:






