import modern_robotics as mr
import numpy as np
from math import sin,cos,fabs,copysign

def jacobean(config,Blist,M0e,Tb0,F6):
	thetalist = np.array([config[3:8]]).T
	T0e = mr.FKinBody(M0e, Blist, thetalist)
	J_base = np.dot(mr.Adjoint(np.dot(mr.TransInv(T0e), mr.TransInv(Tb0))), F6)
	J_arm = mr.JacobianBody(Blist, thetalist)
	J_e = np.hstack((J_base,J_arm))
	return J_e

def FeedbackControl(config, Xd, Xd_next, Kp, Ki, dt,e_int,Blist,M0e,Tb0,F6,max_joint_velocity,joint_limits):
	# calculate actual position Tse
	phi, x, y = config[0:3]
	thetalist = np.array([config[3:8]]).T

	Tsb = np.array([[cos(phi), -sin(phi), 0, x],
					[sin(phi), cos(phi), 0, y],
					[0, 0, 1, 0.0963],
					[0, 0, 0, 1]])

	T0e = mr.FKinBody(M0e, Blist, thetalist)
	X = np.dot(np.dot(Tsb, Tb0), T0e)

	# find desired twist
	Vd = mr.se3ToVec((1/dt) * mr.MatrixLog6(np.matmul(mr.TransInv(Xd),Xd_next)))
	Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
	e_int = e_int + Xerr * dt
	V = np.matmul(mr.Adjoint(np.dot(mr.TransInv(X),Xd)), Vd) + np.dot(Kp, Xerr) + np.dot(Ki,e_int)

	# Joint limit and velocity checks
	J_e = jacobean(config,Blist,M0e,Tb0,F6)
	velocity = np.dot(np.linalg.pinv(J_e,1e-3),V)
	velocity = joint_velocity_check(velocity,max_joint_velocity)
	checklist = joint_limit_check(config,velocity,dt,joint_limits)
	for i in range (len(checklist)-1):
		column = i+3
		J_e[:,column] = checklist[i] * J_e[:,column]
	return V, Xerr,J_e #twist in end effector frame,Xerr,jecobean matrix

def joint_velocity_check(velocity,max_joint_velocity):
	w1,w2,w3,w4,j1,j2,j3,j4,j5 = velocity          # corresponding speeds
	wheelspeed = [w1,w2,w3,w4]
	jointspeeds= [j1,j2,j3,j4,j5]
	newjointspeed = []
	for joint in jointspeeds:
		if fabs(joint)>max_joint_velocity :
			speed = copysign(max_joint_velocity,joint)
			newjointspeed.append(speed)
		else :
			speed= joint
			newjointspeed.append(speed)
	newspeeds = wheelspeed + newjointspeed
	newspeeds = [round(speed,4) for speed in newspeeds]
	return newspeeds


def joint_limit_check(config,velocity,dt,joint_limits):
	phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4, grip = config
	dw1, dw2, dw3, dw4, dj1, dj2, dj3, dj4, dj5 = velocity

	jointspeed = [dj1, dj2, dj3, dj4, dj5]
	joint_config_old = [j1, j2, j3, j4, j5]
	joint_config_old = [round(joint,4) for joint in joint_config_old]
	joint_config = [ round(joint + dt*speed,4) for joint,speed in zip(joint_config_old,jointspeed)]
	vel = []
	for i in range(len(joint_config)):
		joint = joint_config[i]
		jointlimit = joint_limits[i]
		if joint > jointlimit[0] and joint < jointlimit[1]:
			value = 1
			vel.append(value)
		else:
			value = 0
			vel.append(value)
	return vel



