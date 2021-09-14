from feedback import FeedbackControl
from trejectory import TrajectoryGenerator,trajlist_to_array
from nextstep2 import NextStep

import numpy as np
import pandas as pd
from math import sin,cos
import modern_robotics as mr
import matplotlib.pyplot as plt

def main():
    "bor parameters"
    l = 0.47 / 2
    r = 0.0475
    w = 0.3 / 2
    Hpsu = np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)], [1, 1, 1, 1], [-1, 1, -1, 1]])

    # Initialization of configuration matrices:

    # The initial configuration of the robot
    # (chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state)
    # The end-effector has at least 30 degrees of orientation error and 0.2m of position error:
    initial_config = np.array([0.1, -0.2, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

    # The initial configuration of the end-effector in the reference trajectory:
    Tse_init = np.array([[0, 0, 1, 0],
                            [0, 1, 0, 0],
                            [-1, 0, 0, 0.5],
                            [0, 0, 0, 1]])

    # The cube's initial configuration:
    Tsc_init = np.array([[1, 0, 0, 1],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.025],
                            [0, 0, 0, 1]])

    # The cube's desired final configuration:
    Tsc_final = np.array([[0, 1, 0, 0],
                         [-1, 0, 0, -1],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])

    # The end-effector's configuration relative to the cube when it is grasping the cube
    # (the two frames located in the same coordinates, rotated about the y axis):
    Tce_grasp = np.array([[-1 / np.sqrt(2), 0, 1 / np.sqrt(2), 0],
                          [0, 1, 0, 0],
                          [-1 / np.sqrt(2), 0, -1 / np.sqrt(2), 0],
                          [0, 0, 0, 1]])

    # The end-effector's standoff configuration above the cube, before and after grasping, relative
    # to the cube (the {e} frame located 0.1m above the {c} frame, rotated about the y axis):
    Tce_standoff = np.array([[-1 / np.sqrt(2), 0, 1 / np.sqrt(2), 0],
                             [0, 1, 0, 0],
                             [-1 / np.sqrt(2), 0, -1 / np.sqrt(2), 0.1],
                             [0, 0, 0, 1]])

    # The fixed offset from the chassis frame {b} to the base frame of the arm {0}:
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])

    # The end-effector frame {e} relative to the arm base frame {0}, when the arm is at its home configuration:
    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0, 1]])

    # The screw axes B for the 5 joints expressed in the end-effector frame {e}, when the arm is at its home configuration:
    B = np.array([[0, 0, 1, 0, 0.0330, 0],
                      [0, -1, 0, -0.5076, 0, 0],
                      [0, -1, 0, -0.3526, 0, 0],
                      [0, -1, 0, -0.2176, 0, 0],
                      [0, 0, 1, 0, 0, 0]]).T
    # Definitons of constant and simulation parameterizing terms
    dt = 0.01
    maxspeed=10

    # Terms for feeback control to calculate velocity
    kp_gain = 40
    ki_gain = 0.1
    Kp = np.identity(6) * kp_gain  # np.zeros((4,4))
    Ki = np.identity(6) * ki_gain
    e_int = np.zeros(6)
    err_integral = np.zeros(6)

    F = (r / 4) * Hpsu
    # Make the function F6 such that Vb6=F6.u
    F6 = np.array([[0] * 4, [0] * 4])
    F6 = np.append(F6, F, axis=0)
    F6 = np.append(F6, np.array([[0] * 4]), axis=0)

    "initial parameters"
    """ Initial parameters """
    k = 1
    time_per_seg = 3
    config = [0.1, -0.2, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0]
    Final_config = []
    Final_config.append(config)
    traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff,time_per_seg, k,traj_type='Cartesian',method=5)
    df1 = pd.DataFrame(traj)
    df1.to_csv('traj.csv', index=False, header=False)

    timelist = []
    time = 0
    errlist = []

    for i in range (len(traj)-1):
        Xd,gripperstate = trajlist_to_array(traj[i])
        Xd_next,gripperstate = trajlist_to_array(traj[i+1])
        config[len(config)-1] = gripperstate                     # change the gripperstate
        # calculate actual position
        phi,x,y = config[0:3]
        thetalist = np.array([config[3:8]]).T

        Tsb = np.array([[cos(phi), -sin(phi), 0, x],
                        [sin(phi), cos(phi), 0, y],
                        [0, 0, 1, 0.0963],
                        [0, 0, 0, 1]])

        T0e = mr.FKinBody(M0e, B, thetalist)
        X = np.dot(np.dot(Tsb,Tb0),T0e)           #Tse current

        # run feedback
        Ve,Xerr,err_intg = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt,e_int)
        err_integral += err_intg
        #jacobian matrixes

        J_base = np.dot(mr.Adjoint(np.dot(mr.TransInv(T0e),mr.TransInv(Tb0))),F6)
        J_arm = mr.JacobianBody(B,thetalist)
        J_e = np.hstack((J_base, J_arm))

        # velocities
        Velocity_control = np.dot(np.linalg.pinv(J_e,1e-6),Ve)

        # nextstep
        new_config = NextStep(config, Velocity_control, dt, maxspeed,r,Hpsu)
        config = new_config
        Final_config.append(config)

        time = time + dt
        timelist.append(round(time,3))
        errlist.append(Xerr.tolist())


    df = pd.DataFrame(Final_config)
    df.to_csv('config.csv',index=False,header=False)

    dfXerr = pd.DataFrame(Xerr)
    dfXerr.to_csv('Xerr.csv', index=False, header=False)

    dferrintegral = pd.DataFrame(err_integral)
    dferrintegral.to_csv("err_integral",header=False,index=False)

    print(len(errlist))
    print(len(timelist))
    err1 = []
    err2 = []
    err3 = []
    err4 = []
    err5 = []
    err6 = []

    for err in errlist:
        err1.append(err[0])
        err2.append(err[1])
        err3.append(err[2])
        err4.append(err[3])
        err5.append(err[4])
        err6.append(err[5])

    plt.figure()
    plt.plot(timelist,err1,label='Xerr[0]')
    plt.plot(timelist, err2,label='Xerr[1]')
    plt.plot(timelist, err3,label='Xerr[2]')
    plt.plot(timelist, err4,label='Xerr[3]')
    plt.plot(timelist, err5,label='Xerr[4]')
    plt.plot(timelist, err6,label='Xerr[5]')
    plt.title(f'Xerr, kp={kp_gain}, ki={ki_gain}')
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.legend(loc="best")
    plt.grid()
    plt.savefig(f'Xerr,kp={kp_gain},ki={ki_gain}.png')
    plt.show()

    plt.show()


if __name__ == '__main__':
	main()