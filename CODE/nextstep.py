import numpy as np
from math import sin, cos


def NextStep(current_config, controlspeed, dt, maxspeed,r,Hpsu):
    """
    calculates the next step of simulation control from current control
    configuration and current velocities.
    bounds as maxspeed is given to have more control over robot motion

    currentConfig: Chassis phi, Chassis x, Chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, Gripper
    control: Arm joint speed(thetadot), Wheel speed(wdot)
    dt: timestamp
    maxSpeed: Maximum angular speed of the arm joints and the wheels
    """

    phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4, grip = current_config
    dw1, dw2, dw3, dw4,dj1, dj2, dj3, dj4, dj5  = controlspeed

    jointanglesspeed = [dj1,dj2,dj3,dj4,dj5]
    wheelspeed = [dw1,dw2,dw3,dw4]

    for i in range(3):
        i = i + 1
        if wheelspeed[i] > maxspeed:
            wheelspeed[i] = maxspeed
        elif wheelspeed[i] < -maxspeed:
            wheelspeed[i] = -maxspeed

    #eulerstep
    joint_config = [j1, j2, j3, j4, j5]
    wheel_config = [w1, w2, w3, w4]

    new_joint_config = [joint+dt*jointvelocity for joint,jointvelocity in zip(joint_config,jointanglesspeed)]
    new_wheel_config = [wheel*whellvelocity for wheel,whellvelocity in zip(wheel_config,wheelspeed)]

    wheel_dtheta = np.array([dw1, dw2, dw3, dw4])
    new_wheel_theta = dt * wheel_dtheta
    Vb = (r / 4) * np.dot(Hpsu, new_wheel_theta)

    del_phi = Vb[0]   #omega_z
    vx =Vb[1]
    vy = Vb[2]
    Transformation_mat = np.array([[1,0,0],
                      [0,cos(del_phi),-sin(del_phi)],
                      [0,sin(del_phi),cos(del_phi)]])

    if del_phi==0:
        del_qb = np.array([[0,vx,vy]]).T

    else :
        del_qb = np.array([[del_phi,(vx*sin(del_phi)+vy*(cos(del_phi)-1))/del_phi,(vy*sin(del_phi)+vx*(1-cos(del_phi)))/del_phi]]).T

    del_q = Transformation_mat.dot(del_qb)


    new_state_chasis = np.array([[phi,x,y]]).T + del_q

    new_chasis_phi =float(new_state_chasis[0])
    new_chasis_x = float(new_state_chasis[1])
    new_chasis_y = float(new_state_chasis[2])

    new_config = [new_chasis_phi,new_chasis_x,new_chasis_y] + new_joint_config + new_wheel_config
    new_config.append(grip)

    return new_config



