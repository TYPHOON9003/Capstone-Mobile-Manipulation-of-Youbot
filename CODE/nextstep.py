import numpy as np
from math import sin, cos


def NextStep(current_config, controlspeed, dt, maxspeed,r,Hpsu):
    """
    currentConfig: Chassis phi, Chassis x, Chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, Gripper
    control: Arm joint speed(thetadot), Wheel speed(wdot)
    dt: timestamp
    maxSpeed: Maximum angular speed of the arm joints and the wheels
    """
    phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,gripper = current_config
    w1dot, w2dot, w3dot, w4dot,j1dot,j2dot,j3dot,j4dot,j5dot = controlspeed
    curr_chasis_config = [phi,x,y]
    curr_arm_config = [j1,j2,j3,j4,j5]
    curr_wheels_config = [w1,w2,w3,w4]


    for i in range(len(controlspeed)):

        if controlspeed[i] > maxspeed:
            controlspeed[i] = maxspeed
        elif controlspeed[i] < -maxspeed:
            controlspeed[i] = -maxspeed

    #eulerstep
    wheelspeed = [w1dot, w2dot, w3dot, w4dot]
    armspeed = [j1dot,j2dot,j3dot,j4dot,j5dot]

    new_joint_config = [joint+dt*jointvelocity for joint,jointvelocity in zip(curr_arm_config,armspeed)]
    new_wheel_config = [wheel + dt*wheelvelocity for wheel,wheelvelocity in zip(curr_wheels_config,wheelspeed)]


    Vb = (r / 4) * np.dot(Hpsu, np.array(wheelspeed).T)

    del_phi = Vb[0]   #omega_z
    vx =Vb[1]
    vy = Vb[2]

    if del_phi==0:
        del_qb = np.array([[0,vx,vy]]).T

    else :
        del_qb = np.array([[del_phi],
                           [(vx*sin(del_phi)+vy*(cos(del_phi)-1))/del_phi],
                           [(vy*sin(del_phi)+vx*(1-cos(del_phi)))/del_phi]])

    Transformation_mat = np.array([[1,0,0],
                      [0,cos(curr_chasis_config[0]),-sin(curr_chasis_config[0])],
                      [0,sin(curr_chasis_config[0]),cos(curr_chasis_config[0])]])


    new_chasis_config = np.array([curr_chasis_config]).T + np.dot(Transformation_mat,del_qb)

    new_config = new_chasis_config.T.tolist()[0] + new_joint_config + new_wheel_config
    new_config.append(gripper)

    return new_config



