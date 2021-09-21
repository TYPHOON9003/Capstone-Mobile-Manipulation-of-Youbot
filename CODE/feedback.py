import modern_robotics as mr
import numpy as np
from math import sin,cos

def jacobean(config,Blist,M0e,Tb0,F6):
	thetalist = np.array([config[3:8]]).T
	T0e = mr.FKinBody(M0e, Blist, thetalist)
	J_base = np.dot(mr.Adjoint(np.dot(mr.TransInv(T0e), mr.TransInv(Tb0))), F6)
	J_arm = mr.JacobianBody(Blist, thetalist)
	return J_arm,J_base

def FeedbackControl(config, Xd, Xd_next, Kp, Ki, dt,e_int,Blist,M0e,Tb0,F6,jointlimits):
	# calculate actual position tSE
	phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4, grip = config
	joint_config = [j1, j2, j3, j4, j5]
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

	#Jacobeans
	J_arm,J_base = jacobean(config,Blist,M0e,Tb0,F6)
	J_e = np.hstack((J_base,J_arm))
	je_pinv = np.linalg.pinv(J_e, rcond=1e-3)

	Velocity_control = np.dot(je_pinv, V)
	dw1, dw2, dw3, dw4, dj1, dj2, dj3, dj4, dj5 = Velocity_control

	# check the joint limits
	jointanglesspeed = [dj1, dj2, dj3, dj4, dj5]
	new_joint_config = [joint + dt*jointspeed for joint,jointspeed in zip(joint_config,jointanglesspeed)]
	jacobean_control = joint_limit_check(new_joint_config,jointlimits)

	# make the new jecobean by making jacobeana of corresponding joint 0
	for i in range (len(jacobean_control)) :
		J_arm[:,i] = jacobean_control[i] * J_arm[:,i]

	J_e = np.hstack((J_base,J_arm))

	return V, Xerr,J_e #twist in end effector frame,Xerr,jecobean matrix


def joint_limit_check(joint_config,joint_limits):
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

