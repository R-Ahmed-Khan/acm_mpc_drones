import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def trajectory(n_wp, start_point, mid_point, end_point, plotting):
    # Extracting x, y, and z coordinates of start, mid, and end points
    x_start, y_start, z_start = start_point
    x_mid, y_mid, z_mid = mid_point
    x_end, y_end, z_end = end_point
    
    # Generating points along the curve using parametric equations of a curve
    t = np.linspace(0, 1, int(n_wp))  
    x_curve = (1 - t)**2 * x_start + 4.5 * (1 - t) * t * x_mid + t**2 * x_end
    y_curve = (1 - t)**2 * y_start + 4.5 * (1 - t) * t * y_mid + t**2 * y_end
    z_curve = (1 - t)**2 * z_start + 4.5 * (1 - t) * t * z_mid + t**2 * z_end

    if plotting == True:
        # # Plotting the curve 
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x_curve, y_curve, z_curve, label='Curve', color='blue')
        
        # Marking the start, mid, and end points
        ax.scatter(x_start, y_start, z_start, color='red', label='Start Point')
        ax.scatter(x_mid, y_mid, z_mid, color='green', label='Mid Point')
        ax.scatter(x_end, y_end, z_end, color='orange', label='End Point')
        
        # Adding labels and legend
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title('Plot of Curve from Start to End Point')
        ax.legend()
        ax.axis('equal')
        # Displaying the plot
        plt.grid(True)
        plt.show()  
    return x_curve, y_curve, z_curve
    #966 726 486



# start_point = (0, 0, 0)  # Initial point
# mid_point = (0, -1, 0.3)    # Mid point
# end_point = (1, -2, 0.3)   # Final point
# n_wp = 2
# x, y, z = trajectory(n_wp, start_point, mid_point, end_point, plotting=True)
