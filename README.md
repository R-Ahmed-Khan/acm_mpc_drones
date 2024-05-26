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

## Appendix
### Quadrotor Dynamics
The quadrotor's dynamics are described by the following state-space equations, where the state vector includes positions, orientations, and their respective velocities, and the control inputs are the propeller speeds.

#### States

- \( x, y, z \):  Positions in the inertial frame.
- \( \phi, \theta, \psi \): Roll, pitch, and yaw angles.
- \( \dot{x}, \dot{y}, \dot{z} \): Velocities in the inertial frame.
- \( \dot{\phi}, \dot{\theta}, \dot{\psi} \): Angular velocities.

#### Inputs
- \( u_1, u_2, u_3, u_4 \): Propeller speeds.

#### Equations of Motion
1. **Position Dynamics:**
2. 
   $\[
   \begin{align*}
   \dot{x} &= v_x \\
   \dot{y} &= v_y \\
   \dot{z} &= v_z \\
   \end{align*}
   \]$

3. **Orientation Dynamics:**
   \[
   \begin{align*}
   \dot{\phi} &= \omega_x \\
   \dot{\theta} &= \omega_y \\
   \dot{\psi} &= \omega_z \\
   \end{align*}
   \]

4. **Velocity Dynamics:**
$$
   \[
   \begin{align*}
   \dot{v_x} &= \frac{1}{m}\left(\cos(\phi) \sin(\theta) \cos(\psi) + \sin(\phi) \sin(\psi)\right) f_1 - \frac{K_1 v_x}{m} \\
   \dot{v_y} &= \frac{1}{m}\left(\cos(\phi) \sin(\theta) \sin(\psi) - \sin(\phi) \cos(\psi)\right) f_1 - \frac{K_2 v_y}{m} \\
   \dot{v_z} &= -\frac{1}{m}\left(\cos(\psi) \cos(\theta)\right) f_1 + g - \frac{K_3 v_z}{m} \\
   \end{align*}
   \]
$$
6. **Angular Velocity Dynamics:**
   \[
   \begin{align*}
   \dot{\omega_x} &= \frac{I_y - I_z}{I_x} \omega_y \omega_z + \frac{l}{I_x} f_2 - \frac{K_4 l}{I_x} \omega_x + \frac{J_r}{I_x} \omega_y (u_1 - u_2 + u_3 - u_4) \\
   \dot{\omega_y} &= \frac{I_z - I_x}{I_y} \omega_x \omega_z + \frac{l}{I_y} f_3 - \frac{K_5 l}{I_y} \omega_y + \frac{J_r}{I_y} \omega_z (u_1 - u_2 + u_3 - u_4) \\
   \dot{\omega_z} &= \frac{I_x - I_y}{I_z} \omega_x \omega_y + \frac{1}{I_z} f_4 - \frac{K_6}{I_z} \omega_z \\
   \end{align*}
   \]

#### Forces and Torques
- \( f_1 = b(u_1^2 + u_2^2 + u_3^2 + u_4^2) \): Thrust
- \( f_2 = b(-u_2^2 + u_4^2) \): Roll moment
- \( f_3 = b(u_1^2 - u_3^2) \): Pitch moment
- \( f_4 = d(-u_1^2 + u_2^2 - u_3^2 + u_4^2) \): Yaw moment

#### Parameters
- \( m = 1.1 \, \text{kg} \): Mass of the quadrotor
- \( g = 9.81 \, \text{m/s}^2 \): Gravity
- \( I_x = 8.1 \times 10^{-3} \, \text{kg.m}^2 \): Moment of inertia about x-axis
- \( I_y = 8.1 \times 10^{-3} \, \text{kg.m}^2 \): Moment of inertia about y-axis
- \( I_z = 14.2 \times 10^{-3} \, \text{kg.m}^2 \): Moment of inertia about z-axis
- \( l = 0.17 \, \text{m} \): Arm length
- \( b = 0.5 \, \text{N.s}^2 \): Thrust coefficient
- \( d = 0.7 \): Drag coefficient
- \( J_r = 1/2 M_R R_R^2 \): Rotor inertia
- \( K_1, K_2, K_3, K_4, K_5, K_6 = 0.2 \): Drag coefficients

### State and Input Constraints
#### State Constraints
- Position: Unbounded
- Orientation: \( -\pi/2 \leq \phi, \theta, \psi \leq \pi/2 \)
- Velocities: \( -5 \, \text{m/s} \leq v_x, v_y, v_z \leq 5 \, \text{m/s} \)
- Angular Velocities: \( -0.05 \, \text{rad/s} \leq \omega_x, \omega_y, \omega_z \leq 0.05 \, \text{rad/s} \)

#### Input Constraints
- Propeller Speeds: \( 0 \leq u_1, u_2, u_3, u_4 \leq 1200 \)

### Cost Function and Weights
The cost function is designed to minimize the error between the current state and the desired state while penalizing large control inputs. It consists of a quadratic term for state deviation and another for control effort.

#### State Cost Weights
\[
Q = \text{diag}(1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1, 1, 1, 1, 1, 1)
\]

#### Control Cost Weights
\[
R = \text{diag}(0.01, 0.01, 0.01, 0.01)
\]

#### Terminal Cost Weights
\[
Q_{\text{terminal}} = \text{diag}(5, 5, 5, 3, 3, 3, 1, 1, 1, 1, 1, 1)
\]


## Directory Structure
- `mpc_drone_single_point.py`: Script for single point stabilization.
- `mpc_drone_trajectory.py`: Script for trajectory tracking.
- `mpc_drone_obstacle_avoid.py`: Script for obstacle avoidance.
- `plotting.py`: Helper functions for plotting results.
- `trajectory.py`: Helper functions for trajectory generation.
- `images/`: Directory containing generated plots for visualization.


