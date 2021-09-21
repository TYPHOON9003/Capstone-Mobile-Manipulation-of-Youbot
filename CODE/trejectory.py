import numpy as np
from math import sin, cos
import modern_robotics as mr

def Trajectory_segement(T_initial,T_final,time,k,traj_type,method,gripperstate):
	Tt = time
	N = Tt*k/0.01
	if (traj_type == "Screw"):
		traj = mr.ScrewTrajectory(T_initial, T_final, Tt, N, method)
	elif (traj_type == "Cartesian"):
		traj = mr.CartesianTrajectory(T_initial, T_final, Tt, N, method)

	trajlist = []
	for element in traj:
		trajlist.append([element[0][0],element[0][1],element[0][2],element[1][0],element[1][1],element[1][2],\
			element[2][0],element[2][1],element[2][2],element[0][3],element[1][3],element[2][3],gripperstate])
	return trajlist

def trajlist_to_array(traj):

	array= np.array([[traj[0],traj[1],traj[2],traj[9]],
					 [traj[3],traj[4],traj[5],traj[10]],
					 [traj[6],traj[7],traj[8],traj[11]],
					 [0,0,0,1]])
	gripperstate =traj[len(traj)-1]
	return array,gripperstate

def gripper_operation(previouslist,gripperstate):
	lastentry = previouslist[-1]
	lastentry.pop()
	lastentry.append(gripperstate)
	gripper= []
	for i in range (64) :
		gripper.append(lastentry)

	return gripper

def get_time(T_start,T_end,maxspeed):
	x1, y1 = mr.TransToRp(T_start)
	x2, y2 = mr.TransToRp(T_end)
	dist = np.linalg.norm(y2 - y1)
	time = dist/maxspeed
	return time

def TrajectoryGenerator(Tse_init,Tsc_init,Tsc_final,Tce_grasp,Tce_standoff,maxspeed,k,traj_type,method):
	"""
	Tse_init : The initial configuration of the end-effector in the reference trajectory == Tse_initial.
	Tsc_init : The cube's initial configuration == Tsc_initial
	Tsc_final: The cube's desired final configuration == Tsc_final
	Tce_grasp: The end-effector's configuration relative to the cube when it is grasping the cube == Tce_grasp
	Tce_standoff : The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
	maxspeed : Maximum speed of wheels
	traj_type : "Screw" or "Cartesian"
	method : The time-scaling method, where 3 indicates cubic (third-order polynomial) time scaling and
	         5 indicates quintic (fifth-order polynomial) time scaling
	k: The number of trajectory reference configurations per 0.01 seconds

	"""
	grip_open = 0
	grip_close = 1
	total_trejectory = []

	# state-1 initial end-effector to the standoff position
	start = Tse_init
	end = np.dot(Tsc_init,Tce_standoff)
	time = get_time(start,end,maxspeed)
	traj_initial_standoff = Trajectory_segement(start,end,time,k,traj_type,method,grip_open)
	total_trejectory.extend(traj_initial_standoff)

	# state-2 standoff position to cube grasp position
	start = end
	end = np.dot(Tsc_init,Tce_grasp)
	traj_init_grasp = Trajectory_segement(start,end,1,k,traj_type,method,grip_open)
	total_trejectory.extend(traj_init_grasp)

	# state-3 at grasp position close the gripper
	gripper_close = gripper_operation(traj_init_grasp,grip_close)
	total_trejectory.extend(gripper_close)

	# state-4 cube grasp position to standoff position
	start = end
	end = np.dot(Tsc_init, Tce_standoff)
	traj_grasp_standoff = Trajectory_segement(start, end, 1, k, traj_type,method, grip_close)
	total_trejectory.extend(traj_grasp_standoff)

	# state-5 standoff position to final cube standoff position
	start = end
	end = np.dot(Tsc_final, Tce_standoff)
	time = get_time(start, end, maxspeed)
	traj_grasp_final_standoff = Trajectory_segement(start, end, time, k, traj_type,method, grip_close)
	total_trejectory.extend(traj_grasp_final_standoff)

	# state-6 final standoff to final grasp position
	start = end
	end = np.dot(Tsc_final, Tce_grasp)
	traj_final_grasp_release = Trajectory_segement(start, end, 1, k, traj_type,method, grip_close)
	total_trejectory.extend(traj_final_grasp_release)

	# state-7 gripper release
	gripper_open = gripper_operation(traj_final_grasp_release, grip_open)
	total_trejectory.extend(gripper_open)

	# state-8 final standoff position after gripper release
	start = end
	end = np.dot(Tsc_final, Tce_standoff)
	traj_standoff_grasp = Trajectory_segement(start, end, 1, k, traj_type,method, grip_open)
	total_trejectory.extend(traj_standoff_grasp)

	return total_trejectory
