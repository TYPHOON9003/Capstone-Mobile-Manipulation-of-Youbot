import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from feedback import FeedbackControl
from trejectory import TrajectoryGenerator,trajlist_to_array
from nextstep import NextStep

def MobileManipulator(bot_para,config,Blist,M0e,Tb0,Tse_init, Tsc_init, Tsc_final, Tce_grasp,
                      Tce_standoff,maxspeed,max_joint_velocity,joint_limits,dt,Kp_gain,Ki_gain,k,traj_type,method,outputname):


    Kp = np.identity(6) * Kp_gain
    Ki = np.identity(6) * Ki_gain
    e_int = np.zeros(6)

    l,w,r,z = bot_para
    l = l/2
    w = w/2
    Hpsu = np.array([[-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)], [1, 1, 1, 1], [-1, 1, -1, 1]])
    F = (r / 4) * Hpsu

    # Make the function F6 such that Vb6=F6.u
    F6 = np.array([[0] * 4, [0] * 4])
    F6 = np.append(F6, F, axis=0)
    F6 = np.append(F6, np.array([[0] * 4]), axis=0)

    Final_config = []
    Final_config.append(config)

    print("generating trajectory")
    traj = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff,maxspeed, k,traj_type,method)
    df_traj = pd.DataFrame(traj)
    name = 'trajectory_' + outputname +'.csv'
    df_traj.to_csv(name, index=False, header=False)

    timelist = []
    time = 0
    errlist = []
    count = 0
    success = True
    print('running simulation')
    while success==True:
        if count < len(traj)-1:
            Xd,gripperstate = trajlist_to_array(traj[count])
            Xd_next,gripperstate = trajlist_to_array(traj[count+1])
            config[len(config)-1] = gripperstate                     # change the gripperstate

            # run feedback
            Ve,Xerr,J_e = FeedbackControl(config, Xd, Xd_next, Kp, Ki, dt,e_int,Blist,M0e,Tb0,F6,max_joint_velocity,joint_limits)

            final_velocity = np.dot(np.linalg.pinv(J_e,1e-3),Ve)
            # nextstep
            new_config = NextStep(config, final_velocity, dt, maxspeed,r,Hpsu)
            config = new_config
            Final_config.append(config)
            time = time + dt
            timelist.append(round(time,3))
            errlist.append(Xerr.tolist())
        else :
            success = False
        count = count +1



    df = pd.DataFrame(Final_config)
    name = 'config_' + outputname+'.csv'
    df.to_csv(name,index=False,header=False)

    dfXerr = pd.DataFrame(errlist)
    name= 'Xerr_' + outputname +'.csv'
    dfXerr.to_csv(name, index=False, header=False)


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
    plt.title(f'Xerr, kp={Kp_gain}, ki={Ki_gain}')
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.legend(loc="best")
    plt.grid()
    name = 'Xerr_' + outputname
    plt.savefig(f'{name},kp={Kp_gain},ki={Ki_gain}.png')
    plt.show()

    return success