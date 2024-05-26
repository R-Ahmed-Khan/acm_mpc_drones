# acm_mpc_drones
# MPC Drone Control Project

## Overview
This project implements a Model Predictive Control (MPC) strategy for stabilizing and navigating a quadrotor drone. The controller is designed to manage the drone's position, orientation, and velocity in three-dimensional space. The codebase includes functions to simulate the drone's dynamics, generate optimal control inputs, and visualize the results.

**Team Members:**
- Roohan Ahmed Khan
- Malaika Zafar
- Amber Batool


**Presentation:** [Final Presentation Link](https://your-presentation-link.com)

## Problem Statement
The primary objective of this project is to develop a control system for a quadrotor drone that can achieve precise point stabilization and trajectory tracking. Quadrotors are widely used in various applications such as aerial photography, surveillance, and delivery services. However, their nonlinear dynamics and the presence of external disturbances make control a challenging task.

### Importance
Developing a robust control system for quadrotors is crucial as it enhances their performance and reliability in real-world applications. Efficient and precise control can lead to significant advancements in automation and autonomous systems.

## Results
The implemented MPC controller successfully stabilizes the drone at the desired point and follows specified trajectories. The results include:
- Accurate position tracking in 3D space.
- Stability in orientation and velocity.
- Visualization of the drone's state over time.

### Visual Aids
The following plots illustrate the performance of the MPC controller:

1. **Positions of Drone over Time**
   ![Positions of Drone](images/position_plot.png)

2. **Orientation of Drone over Time**
   ![Orientation of Drone](images/orientation_plot.png)

3. **Velocities of Drone over Time**
   ![Velocities of Drone](images/velocity_plot.png)

4. **Desired Orientations of Drone over Time**
   ![Desired Orientations of Drone](images/desired_orientation_plot.png)

## Running the Project
Follow these steps to set up and run the project on your local machine.

### Prerequisites
- Python 3.x
- CasADi
- NumPy
- Matplotlib

### Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/mpc-drone-control.git
    cd mpc-drone-control
    ```

2. Install the required packages:
    ```bash
    pip install -r requirements.txt
    ```

### Running the Code
1. **Single Point Stabilization:**
    ```bash
    python mpc_drone_single_point.py
    ```

2. **Trajectory Tracking:**
    ```bash
    python mpc_drone_trajectory.py
    ```

3. **Obstacle Avoidance:**
    ```bash
    python mpc_drone_obstacle_avoid.py
    ```

### Visualizing Results
The results can be visualized using Matplotlib. The scripts will automatically generate plots showing the drone's positions, orientations, velocities, and desired orientations over time.

## Directory Structure
- `mpc_drone_single_point.py`: Script for single point stabilization.
- `mpc_drone_trajectory.py`: Script for trajectory tracking.
- `mpc_drone_obstacle_avoid.py`: Script for obstacle avoidance.
- `plotting.py`: Helper functions for plotting results.
- `trajectory.py`: Helper functions for trajectory generation.
- `images/`: Directory containing generated plots for visualization.


