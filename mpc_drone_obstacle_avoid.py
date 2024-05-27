import casadi as ca
import numpy as np
import time
from resources.draw import *
from matplotlib import pyplot as plt

def shift_movement(T, t0, x0, u, f):
    st = x0
    con = (u[:,0])
    f_value = f(st,con)
    state_next_ = st + T*f_value
    t_ = t0 + T
    u_next_ = ca.vertcat(u[1:, :], u[-1, :])
    return t_, state_next_, u_next_

if __name__ == '__main__':

    T = 0.1                                 # sampling time [s]                        
    N = 10                                  # prediction horizon (Final With Terminal Cost)                        
    M_R = 0.065                             # kg, mass of rotor
    R_R = 0.31                              # m, radius of rotor
    Jr = 1/2 * M_R * R_R**2                 # inertia of rotor about COM
    K1 = K2 = K3 = K4 = K5 = K6 = 0.2       # drag coefficient
    m = 1.1                                 # mass of the quadrotor [kg]
    g = 9.81                                # gravity [m/s^2]
    Ix = 8.1*10**(-3)                       # moment of inertia about Bx axis [kg.m^2]
    Iy =  8.1*10**(-3)                      # moment of inertia about By axis [kg.m^2]
    Iz = 14.2*10**(-3)                      # moment of inertia about Bz axis [kg.m^2]
    l = 0.17                                # quadrotor arm length [m]
    b = 0.5                                 # thrust/lift coefficient [N.s^2]
    d = 0.7                                 # scalling factor

    # initalize CasADi variables
    x = ca.SX.sym('x');                     y = ca.SX.sym('y');                 z = ca.SX.sym('z')
    phi = ca.SX.sym('phi');                 phi_dot = ca.SX.sym('phi_dot');     theta = ca.SX.sym('theta')
    vx = ca.SX.sym('vx');                   vy = ca.SX.sym('vy');               vz = ca.SX.sym('vz')
    theta_dot = ca.SX.sym('theta_dot');     psi = ca.SX.sym('psi');             psi_dot = ca.SX.sym('psi_dot')
    
    # initalize system states (linear and angular positions and velocities)
    states = ca.vertcat(x, y, z, phi, theta, psi,  vx, vy, vz, phi_dot, theta_dot, psi_dot)
    n_states = states.size()[0]

    # initalize system inputs (propeller speed) in CasADi 
    u1 = ca.SX.sym('u1');       u2 = ca.SX.sym('u2');       u3 = ca.SX.sym('u3');       u4 = ca.SX.sym('u4')
    controls = ca.vertcat(u1, u2, u3, u4)
    n_controls = controls.size()[0]

    ## forces and torques
    f1 = b*(u1**2 + u2**2 + u3**2 + u4**2)    #Thrust
    f2 = b*(-u2**2 + u4**2)                   #L_Moment
    f3 = b*(u1**2 - u3**2)                    #M_Moment
    f4 = d*(-u1**2 + u2**2 - u3**2 + u4**2)   #N_Moment
    
    # rhs
    x_dot = states[6]
    y_dot = states[7]
    z_dot = states[8]
    phi_dot = states[9]
    theta_dot = states[10]
    psi_dot = states[11]
    vx_d = (1/m)*(ca.cos(states[3]) * ca.sin(states[4]) * ca.cos(states[5]) + ca.sin(states[3]) * ca.sin(states[5])) * f1 - (K1 * states[6]) / m
    vy_d = (1/m)*(ca.cos(states[3]) * ca.sin(states[4]) * ca.sin(states[5]) - ca.sin(states[3]) * ca.cos(states[5])) * f1 - (K2 * states[7]) / m
    vz_d = -(1/m)*(ca.cos(states[5]) * ca.cos(states[4])) * f1 + g - (K3 * states[8]) / m
    p_d = (states[10] * states[11] * (Iy - Iz) / Ix) + (f2 * l / Ix) - (K4 * l / Ix) * states[9]+ (Jr / Ix) * states[10] * (u1 - u2 + u3 - u4)
    q_d =  (states[9] * states[11] * (Iz - Ix) / Iy) + (f3 * l / Iy)  - (K5 * l / Iy) * states[10] + (Jr / Iy) * states[11] * (u1 - u2 + u3 - u4)
    r_d =  (states[9] * states[10] * (Ix - Iy) / Iz) + f4/Iz - (K6 / Iz) * states[11]

    rhs = ca.vertcat(x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot, vx_d, vy_d, vz_d, p_d, q_d, r_d)

    # create CasADi function for states and controls
    f = ca.Function('f', [states, controls], [rhs], ['input_state', 'control_input'], ['rhs'])

    U = ca.SX.sym('U', n_controls, N) 
    P = ca.SX.sym('P', n_states+n_states)
    X = ca.SX.sym('X', n_states,N+1)

    X[:,0] = P[0:12] 

    # define the relationship within the horizon
    for i in range(N):
        st = X[:, i]
        con = U[:, i]
        f_value = f(st, con)
        st_next = st + (T*f_value)
        X[:, i+1] = st_next

    # define weights for states (Q) and inputs (R)
    Q = np.diag([1, 3, 1, 1, 2, 1, 1, 0.5, 1, 1, 0.5, 1])
    R = np.diag([0.01,0.01,0.01,0.01])

    # weights for terminal state
    Q_terminal = np.diag([5, 5, 3, 3, 3, 2, 1, 5, 1, 1, 0.5, 1]) 
    
    # define quadratic cost function
    obj = 0 
    for i in range(N-1):
        st = X[:,i]
        con = U[:,i]
        obj = obj + ca.mtimes([(st-P[12:24]).T, Q, (st-P[12:24])]) + ca.mtimes([con.T, R, con])

    # add terminal cost
    st_terminal = X[:, N]  # terminal state
    obj += ca.mtimes([(st_terminal - P[12:24]).T, Q_terminal, (st_terminal - P[12:24])]) 

    #Defining Obstacles parameters
    obs1_x = 0.54
    obs1_y = 0.8
    obs1_diam = 0.15
    obs1_radius = obs1_diam / 2.0

    obs2_x = 0.8
    obs2_y = 0.54
    obs2_diam = 0.15
    obs2_radius = obs2_diam / 2.0

    ## initialize state-constraints vector
    g = [] 
    for i in range(N+1):
        g.append(X[0, i])
        g.append(X[1, i])
        g.append(X[2, i])
        g.append(X[3, i])
        g.append(X[4, i])
        g.append(X[5, i])
        g.append(X[6, i])
        g.append(X[7, i])
        g.append(X[8, i])
        g.append(X[9, i])
        g.append(X[10, i])
        g.append(X[11, i])
        obstacle1_distance = ca.sqrt((X[0,i] - obs1_x)**2 + (X[1,i] - obs1_y)**2)  # Euclidean distance between quadrotor and obstacle
        g.append(obstacle1_distance - obs1_radius)  # Ensure distance is greater than obstacle radius
        obstacle2_distance = ca.sqrt((X[0,i] - obs2_x)**2 + (X[1,i] - obs2_y)**2)  # Euclidean distance between quadrotor and obstacle
        g.append(obstacle2_distance - obs2_radius)  # Ensure distance is greater than obstacle radius
        

    # setting-up Solver Settings
    OPT_variables = ca.reshape(U, -1,1)
    nlp_prob = {'f': obj, 'x': OPT_variables,'g':ca.vertcat(*g), 'p':P }
    opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts_setting)

    # create empty lists for storing constraints for N
    lbg = []; ubg = []
    lbx = []; ubx = []
    
    # fill-in state and input Constraints
    for _ in range(N+1):
        lbg.append(float("-inf")); ubg.append(float("+inf"))
        lbg.append(float("-inf")); ubg.append(float("+inf"))
        lbg.append(float("-inf")); ubg.append(float("+inf"))
        lbg.append(-np.pi/2); ubg.append(np.pi/2)
        lbg.append(-np.pi/2); ubg.append(np.pi/2)
        lbg.append(-np.pi/2); ubg.append(np.pi/2)
        lbg.append(-5); ubg.append(5)
        lbg.append(-5); ubg.append(5)
        lbg.append(-5); ubg.append(5)
        lbg.append(-0.05); ubg.append(0.05)
        lbg.append(-0.05); ubg.append(0.05)
        lbg.append(-0.05); ubg.append(0.05)
        lbg.append(obs1_diam+0.1); ubg.append(float("+inf")) 
        lbg.append(obs2_diam+0.1); ubg.append(float("+inf")) 

    # Convert lists to numpy arrays
    lbg = np.array(lbg)
    ubg = np.array(ubg)

    # define minimum and propeller speeds
    u1_min = u2_min = u3_min = u4_min = 0    
    u1_max = u2_max = u3_max = u4_max = 1200
    for _ in range(N):
        lbx.append(u1_min)
        ubx.append(u1_max)  
        lbx.append(u2_min)
        ubx.append(u2_max)
        lbx.append(u3_min)
        ubx.append(u3_max)
        lbx.append(u4_min)
        ubx.append(u4_max)

    # initial conditions
    t0 = 0.0
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1, 1)# initial state
    xs = np.array([1.3, 1.3, 1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).reshape(-1, 1) # final state
    u0 = 0 * np.ones((N,4)) 

    # initialize variable to store state history
    u_c = []
    t_c = [] 
    xx = []     
    sim_time = 40.0

   ## start MPC
    mpciter = 0
    start_time = time.time()
    index_t = []
    while(np.linalg.norm(x0-xs)>1e-2 and mpciter-sim_time/T<0.0 ):
        c_p = np.concatenate((x0, xs))
        init_control = ca.reshape(u0, -1, 1)
        t_ = time.time()
        res = solver(x0=init_control, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=c_p)
        index_t.append(time.time()- t_)
        u_sol = ca.reshape(res['x'], n_controls, N) 
        u_c.append(u_sol[:, 0])
        t_c.append(t0)
        t0, x0, u0 = shift_movement(T, t0, x0, u_sol, f)
        x0 = ca.reshape(x0, -1, 1)
        xx.append(x0.full())
        x = np.array(xx)
        mpciter = mpciter + 1

    t_v = np.array(index_t)

    # storing states seperately for plotting
    X, Y, Z, phi, theta, psi, vx, vy, vz, phi_d, theta_d, psi_d = x[:, 0], x[:, 1], x[:, 2], x[:, 3], x[:, 4], x[:, 5], x[:, 6], x[:, 7], x[:, 8], x[:, 9], x[:, 10], x[:, 11]
    tc = np.array(t_c)

    # plotting 

    fig, axs = plt.subplots(2, 2, figsize=(15,15))
    ## Plot 1: Positions of Drone over Time
    axs[0,0].plot(t_c, X, label='X data')
    axs[0,0].plot(t_c, Y, label='Y data')
    axs[0,0].set_xlabel('Time (sec)')
    axs[0,0].set_ylabel('Position (m)')
    axs[0,0].set_title('Positions of Drone over Time')
    axs[0,0].legend()

    ## Plot 2: Orientation of Drone over Time
    axs[0,1].plot(t_c, [roll * 57.3 for roll in phi], label='phi')
    axs[0,1].plot(t_c, [pitch * 57.3 for pitch in theta], label='theta')
    axs[0,1].plot(t_c, [yaw * 57.3 for yaw in psi], label='psi')
    axs[0,1].set_xlabel('Time (sec)')
    axs[0,1].set_ylabel('Orientation (deg)')
    axs[0,1].set_title('Orientation of Drone over Time')
    axs[0,1].legend()

    ## Plot 3: Velocities of Drone over Time
    axs[1,0].plot(t_c, vx, label='vx')
    axs[1,0].plot(t_c, vy, label='vy')
    axs[1,0].set_xlabel('Time (s)')
    axs[1,0].set_ylabel('Velocity (m/s)')
    axs[1,0].set_title('Velocities of Drone over Time')
    axs[1,0].legend()

    ## Plot 4: Desired Orientations of Drone over Time
    axs[1,1].plot(t_c, [roll_d * 57.3 for roll_d in phi_d], label='phi_d')
    axs[1,1].plot(t_c, [pitch_d * 57.3 for pitch_d in theta_d], label='theta_d')
    axs[1,1].plot(t_c, [yaw_d * 57.3 for yaw_d in psi_d], label='psi_d')
    axs[1,1].set_xlabel('Time (sec)')
    axs[1,1].set_ylabel('Desired Orientation (deg/sec)')
    axs[1,1].set_title('Desired Orientations of Drone over Time')
    axs[1,1].legend()

    ## Adjust layout
    # plt.tight_layout(pad = 4)
    fig.subplots_adjust(hspace=0.35, wspace=0.27)  # Increase the vertical space

    fig.savefig('./results/drone_states_obs_avoid.png')  # Save as PNG file

    ## Show the plot
    plt.show()

    ## Plot Animation
    draw_result = Draw_MPC_Obstacle_2D(rob_diam=0.3, init_state=x0.full(), target_state=xs, robot_states=xx, obstacle1=np.array([obs1_x, obs1_y, obs1_diam/2.]),obstacle2=np.array([obs2_x, obs2_y, obs2_diam/2.]), export_fig=True)


