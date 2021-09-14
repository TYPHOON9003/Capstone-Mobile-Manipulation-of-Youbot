import modern_robotics as mr
import numpy as np

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt,e_int):

	#find desired twist

	Vd = mr.se3ToVec((1/dt) * mr.MatrixLog6(np.matmul(mr.TransInv(Xd),Xd_next)))
	Xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X),Xd)))
	e_int = e_int + Xerr * dt
	V = np.matmul(mr.Adjoint(np.dot(mr.TransInv(X),Xd)), Vd) + np.dot(Kp, Xerr) + np.dot(Ki,e_int)

	return V, Xerr,e_int #twist in end effector frame, #errot , # error integral
