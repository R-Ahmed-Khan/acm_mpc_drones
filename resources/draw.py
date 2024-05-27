import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from resources.trajectory import trajectory

class Draw_MPC_point_stabilization_3D(object):
    def __init__(self, robot_states: list, init_state: np.array, target_state: np.array, rob_diam=0.3,
                 export_fig=False):
        self.robot_states = robot_states
        self.init_state = init_state
        self.target_state = target_state
        self.rob_radius = rob_diam / 2.0
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')  # Creating a 3D subplot
        self.ax.set_xlim([-0.8, 3])
        self.ax.set_ylim([-1, 3.0])
        self.ax.set_zlim([-0.2, 2])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('MPC for Point Stabilization')
        # init for plot
        self.animation_init()
        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)
        plt.grid('--')
        if export_fig:
            self.ani.save('./Drone Trajectory 3D.mp4', writer='ffmpeg', fps=10)
        plt.show()


    def animation_init(self):
        # plot target state
        self.target_circle = self.ax.plot([self.target_state[0]], [self.target_state[1]], [self.target_state[2]],
                                          marker='o', markersize=10, color='b')
        self.robot_body = self.ax.plot([self.init_state[0]], [self.init_state[1]], [self.init_state[2]],
                                       marker='o', markersize=10, color='r')
        return self.target_circle,  self.robot_body

    def animation_loop(self, indx):
        position = self.robot_states[indx][:3]
        orientation = self.robot_states[indx][3:6]
        self.robot_body[0].set_data(position[0], position[1])
        self.robot_body[0].set_3d_properties(position[2])
        return self.robot_body[0]
    


class Draw_MPC_Obstacle_2D(object):
    def __init__(self, robot_states: list, init_state: np.array, target_state: np.array, obstacle1: np.array, obstacle2: np.array,
                 rob_diam=0.3, export_fig=False):
        self.robot_states = robot_states
        self.init_state = init_state
        self.target_state = target_state
        self.rob_radius = rob_diam / 2.0
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(-0.8, 3), ylim=(-0.8, 3.))
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
        self.ax.set_title('Obstacle avoidance using MPC Controller')
        if obstacle1 is not None:
            self.obstacle1 = obstacle1
        else:
            print('no obstacle1 given, break')

        if obstacle2 is not None:
            self.obstacle2 = obstacle2
        else:
            print('no obstacle2 given, break')
        self.fig.set_size_inches(7, 6.5)
        # init for plot
        self.animation_init()
        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)
        plt.grid('--')
        if export_fig:
            self.ani.save('./ObstacleAvoidance.mp4', writer='ffmpeg', fps=10)
        plt.show()

    def animation_init(self):
        # plot target state
        self.target_circle = plt.Circle(self.target_state[:2], self.rob_radius, color='b', fill=False)
        self.ax.add_artist(self.target_circle)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.obstacle1_circle = plt.Circle(self.obstacle1[:2], self.obstacle1[2], color='g', fill=True)
        self.ax.add_artist(self.obstacle1_circle)

        self.obstacle2_circle = plt.Circle(self.obstacle2[:2], self.obstacle2[2], color='g', fill=True)
        self.ax.add_artist(self.obstacle2_circle)

        return self.target_circle, self.robot_body, self.obstacle1_circle, self.obstacle2_circle
    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        self.robot_body.center = position
        return self.robot_body
    



class Draw_MPC_trajectory_3D(object):
    def __init__(self, robot_states: list, init_state: np.array, target_state: np.array, rob_diam=0.1,
                 export_fig=False):
        self.robot_states = robot_states
        self.init_state = init_state
        self.target_state = target_state
        self.rob_radius = rob_diam / 100
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')  # Creating a 3D subplot
        self.ax.set_xlim([-0.2, 1])
        self.ax.set_ylim([-2.0, 0.2])
        self.ax.set_zlim([-0.2, 1])
        self.ax.set_xlabel('Drone X-Position (m)',fontsize = 11)
        self.ax.set_ylabel('Drone Y-Position (m)',fontsize = 11)
        self.ax.set_zlabel('Drone Z-Position (m)',fontsize = 11)
        self.ax.set_title('MPC Controller on Drone Trajectory(N=7)')
        # init for plot
        self.animation_init()

        # Create the legend once after initializing the plot
        self.ax.legend()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, frames=len(self.robot_states),
                                           init_func=self.animation_init, interval=100, repeat=False)

        plt.grid('--')
        if export_fig:
            self.ani.save('./Trajectory_Drone_Large.mp4', writer='ffmpeg', fps=10)
        plt.show()

    def animation_init(self):
        start_point = (0, 0, 0)  # Initial point
        mid_point = (0, -1, 0.3)  # Mid point
        end_point = (1, -2, 0.3)  # Final point
        n_wp = 40
        x1, y1, z1 = trajectory(n_wp, start_point, mid_point, end_point, plotting=False)

        self.robot_body, = self.ax.plot([self.init_state[0]], [self.init_state[1]], [self.init_state[2]],
                                        marker='o', markersize=8, color='r', label='Robot States')

        # plot final position
        self.final_position, = self.ax.plot([self.robot_states[-1][0]], [self.robot_states[-1][1]], [self.robot_states[-1][2]],
                                            marker='o', markersize=8, color='k', label='Final Position')

        # plot trajectory curve
        trajectory_points = np.array(self.robot_states)[:, :3]
        self.trajectory_curve, = self.ax.plot(trajectory_points[:, 0], trajectory_points[:, 1], trajectory_points[:, 2], color='b', label='Trajectory')

        # plot obstacles as gate pillars
        z = np.linspace(0, 0.2, 100)  # Define the height of the pillars
        self.pillar1, = self.ax.plot(np.full_like(z, 0.2), np.full_like(z, -1.5), z, color='g', label='Gate')
        self.pillar2, = self.ax.plot(np.full_like(z, 0.2), np.full_like(z, -0.9), z, color='g')
        self.pillar3, = self.ax.plot(np.full_like(z, 0.7), np.full_like(z, -1.9), z, color='g')
        self.pillar4, = self.ax.plot(np.full_like(z, 0.7), np.full_like(z, -1.3), z, color='g')

        return self.robot_body, self.final_position, self.pillar1, self.pillar2, self.pillar3, self.pillar4, self.trajectory_curve

    def animation_loop(self, indx):
        position = self.robot_states[indx][:3]
        self.robot_body.set_data(position[0], position[1])
        self.robot_body.set_3d_properties(position[2])

        # Update the final position if it's the last frame
        if indx == len(self.robot_states) - 1:
            self.final_position.set_data(position[0], position[1])
            self.final_position.set_3d_properties(position[2])

        return self.robot_body, self.final_position, self.pillar1, self.pillar2, self.pillar3, self.pillar4, self.trajectory_curve


