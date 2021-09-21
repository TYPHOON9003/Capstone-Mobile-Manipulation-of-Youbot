import numpy as np
from MobileManipulation import MobileManipulator

def main():

    bot_para = [0.47,0.3,0.0475,0.0963]     # [l,w,r,z]
    config = [0.1, -0.2, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0]  # youbot initial configuration

    maxspeed = 0.5  # max velicity for wheel and joints
    dt = 0.01  #timestep

    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])

    # The end-effector frame {e} relative to the arm base frame {0}, when the arm is at its home configuration:
    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]])
    Blist = np.array([[0, 0, 1, 0, 0.0330, 0],
                      [0, -1, 0, -0.5076, 0, 0],
                      [0, -1, 0, -0.3526, 0, 0],
                      [0, -1, 0, -0.2176, 0, 0],
                      [0, 0, 1, 0, 0, 0]]).T



    # initial positions
    Tse_init = np.array([[0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [-1, 0, 0, 0.5],
                            [0, 0, 0, 1]])

    # The cube's initial configuration:
    Tsc_init = np.array([[1, 0, 0, -1],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.025],
                            [0, 0, 0, 1]])

    # The cube's desired final configuration:
    Tsc_final = np.array([[0, 1, 0, 1],
                         [-1, 0, 0, -1],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    # The end-effector's configuration relative to the cube when it is grasping the cube
    # (the two frames located in the same coordinates, rotated about the y axis):
    Tce_grasp = np.array([[-0.7071, 0 , 0.7071, 0.02],[0,1,0,0],[-0.7071, 0, -0.7071, -0.02],[0,0,0,1]])

    # The end-effector's standoff configuration above the cube, before and after grasping, relative
    # to the cube (the {e} frame located 0.1m above the {c} frame, rotated about the y axis):
    Tce_standoff = np.array([[-0.7071, 0 , 0.7071, 0],[0,1,0,0],[-0.7071, 0, -0.7071, 0.06],[0,0,0,1]])


    # Tregectory
    traj_type = "Cartesian"                 # "Screw" or "Cartesian"
    method = 5    # The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling and
	              #  5 indicates quintic (fifth-order polynomial) time scaling
    k = 5         # The number of trajectory reference configurations per 0.01 seconds


    #PID controller
    Kp =10
    Ki = 0.05
   # joint_limits = [[-3,3], [-1, 1.2], [-1.6, 3], [-0.8, 3], [-3,3]]
    joint_limits = [[-1,1],[-1.9,1.2],[-1.9,1.5],[-2.89,2.89],[-2.89,2.89]]


if __name__ == "__main__":
    main()
    suceess = MobileManipulator(bot_para, config, joint_limits, Blist, M0e, Tb0, Tse_init, Tsc_init, Tsc_final,Tce_grasp, Tce_standoff,maxspeed, dt, Kp, Ki, k, traj_type, method)
    print("simulation succes:", success)


