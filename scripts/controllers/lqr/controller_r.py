import rospy
import numpy as np
import scipy.linalg as la
from copy import deepcopy
from scipy.optimize import minimize
import scripts.pt_scripts.utils as utils 


class Controller:
	def __init__(self, ros_interface, ref_traj, robot, plotting, dt = 0.2):
		
		# input data (robot & reference trajectory)
		self.robot = robot
		self.ref_traj = ref_traj
		self.goal_x = ref_traj.x[-1]
		self.goal_y = ref_traj.y[-1]
		self.robot = robot
		self.rosi = ros_interface
		
		# plotting
		self.plotting = plotting

		# controller
		self.from_beggins = False
		self.horizon = 15

		# # settings
		self.dt = dt
		self.dist_thresh = 0.6
		self.goal_dist_thresh = 1.0
		# set the next target if velocity is less than these values:
		self.stop_v_thresh = 0.06 
		self.stop_w_thresh = np.deg2rad(4)
		self.robot.set_dt(dt)

		# LQR settings
		self.Q = np.eye(5)*0.1
		self.R = np.eye(2)*0.1
		self.R[0,0] = 0.9

		self.solver_max_iter = 150
		
		# recorded trajectory
		self.rec_traj_x = []
		self.rec_traj_y = []
		self.rec_traj_yaw = []
		self.rec_l = 0
		self.rec_t = 0
		self.rec_w = 0
		self.prev_w = 0
		self.start_time = rospy.get_time()


	def control(self):
		goal_dist = utils.distance(self.goal_x, self.goal_y, self.rosi.r_pose[0], self.rosi.r_pose[1])
		
		# starting index
		if self.from_beggins:
			current_idx = 0
		else:
			current_idx, dists = self.find_nearest_ind(self.rosi.r_pose)

		prev_min_dist = 0
		prev_yaw_curr_ref = 0

		while goal_dist>self.goal_dist_thresh:
						
			# update distance and target ind
			_, dists = self.find_nearest_ind(self.rosi.r_pose)
			# min_dist = dists[current_idx]
			min_dist = min(dists[current_idx:current_idx+self.horizon])

			while min_dist<self.dist_thresh and current_idx!= len(dists)-1:
				current_idx+= 1
				# min_dist = dists[current_idx]
				min_dist = min(dists[current_idx:current_idx+self.horizon])

			self.lookahead_point = self.ref_traj.xy_poses[current_idx]
			side = self.eval_side(current_idx)
			min_dist = side*min_dist

			# update current and reference yaw difference
			dx = self.rosi.r_pose[0] - self.ref_traj.x[current_idx]
			dy = self.rosi.r_pose[1] - self.ref_traj.y[current_idx]
			theta = np.arctan2(-dy, -dx)
			yaw_curr_ref =  self.rosi.r_pose[2] - theta
			yaw_curr_ref = np.arctan2(np.sin(yaw_curr_ref), np.cos(yaw_curr_ref))

			# yaw_curr_ref = self.rosi.r_pose[2] - self.ref_traj.yaw[current_idx]
			# yaw_curr_ref = np.arctan2(np.sin(yaw_curr_ref), np.cos(yaw_curr_ref))

			# update current and reference velocity difference
			current_v = self.rosi.r_vel
			v_curr_ref = current_v - self.ref_traj.v[current_idx]

			# A, B, State vector
			# state vector
			# x = [min_dist, min_dist_d, yaw_curr_ref, yaw_curr_ref_d, delta_v]
			# A = [1.0, dt, 0.0, 0.0, 0.0
			#      0.0, 0.0, v, 0.0, 0.0]
			#      0.0, 0.0, 1.0, dt, 0.0]
			#      0.0, 0.0, 0.0, 0.0, 0.0]
			#      0.0, 0.0, 0.0, 0.0, 1.0]
			# B = [0.0, 0.0
			#     0.0, 0.0
			#     0.0, 0.0
			#     v/L, 0.0
			#     0.0, dt]
			A = np.zeros((5, 5))
			A[0, 0] = 1.0
			A[0, 1] = self.dt
			A[1, 2] = current_v
			A[2, 2] = 1.0
			A[2, 3] = self.dt
			A[4, 4] = 1.0
			B = np.zeros((5, 2))
			B[3, 0] = self.dt
			B[4, 1] = self.dt

			# update K
			K, _, _ = self.DLQR(A, B)

			# state vector
			x = np.zeros((5, 1))
			x[0, 0] = min_dist
			x[1, 0] = (min_dist - prev_min_dist) / self.dt
			x[2, 0] = yaw_curr_ref
			x[3, 0] = (yaw_curr_ref - prev_yaw_curr_ref) / self.dt
			x[4, 0] = v_curr_ref

			# update prev_min_dist and yaw_curr_ref
			prev_min_dist = min_dist
			prev_yaw_curr_ref = yaw_curr_ref

			# input vector
			u = -np.dot(K, x)
			cmd_w = u[0, 0]
			cmd_acc = u[1, 0]
			# feed forward
			# cmd_w = cmd_w - yaw_curr_ref
			print("cmd_w", round(cmd_w, 2), "cmd_acc", round(cmd_acc, 2))

			# update robot
			cmd_w = min(max(cmd_w, self.robot.w_min), self.robot.w_max) 
			self.robot.update_lqr(cmd_acc, cmd_w)
			self.rosi.update(self.robot.v, self.robot.w, self.lookahead_point)
			goal_dist = self.update()
			self.robot.update_pose(self.rosi.r_pose)
			self.robot.update_velocity(self.rosi.r_vel, cmd_w)

		self.rec_t = rospy.get_time() - self.start_time
		self.rosi.stop()


	def DLQR(self, A, B):
		"""Solve the discrete time lqr controller.
		x[k+1] = A x[k] + B u[k]
		cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
		# ref Bertsekas, p.151
		"""

		# first, try to solve the ricatti equation
		X = self.solve_dare(A, B)

		# compute the LQR gain
		temp1 = np.dot(B.T, X)
		temp2 = np.dot(temp1, B)
		temp3 = temp2 + self.R
		temp4 = la.inv(temp3)
		K = np.dot(temp4, np.dot(temp1, A))
		# K = la.inv(B.T @ X @ B + self.R) @ (B.T @ X @ A)

		eig_result = la.eig(A - np.dot(B, K))

		return K, X, eig_result[0]	

		
	# solve a discrete time_Algebraic Riccati equation (DARE)
	def solve_dare(self, A, B):		
		x = self.Q
		x_next = self.Q
		eps = 0.01

		for i in range(self.solver_max_iter):
			temp1 = np.dot(np.dot(A.T, x), A)
			temp2 = np.dot(np.dot(np.dot(np.dot(A.T, x), B), la.inv(self.R + np.dot(np.dot(B.T, x), B))), np.dot(B.T, x))
			x_next = temp1 - temp2 + self.Q
			# x_next = A.T @ x @ A - A.T @ x @ B @ \
			# 			la.inv(self.R + B.T @ x @ B) @ B.T @ x @ A + self.Q
			if (abs(x_next - x)).max() < eps:
				break
			x = x_next

		return x_next
			
	# -------------------------------------- update --------------------------------------

	def update(self):
		# goal distance
		goal_dist = utils.distance(self.goal_x, self.goal_y, self.robot.x, self.robot.y)

		# record trajectory
		self.rec_traj_x.append(self.robot.x)
		self.rec_traj_y.append(self.robot.y)
		self.rec_traj_yaw.append(self.robot.yaw)
		self.rec_w += abs(self.robot.w - self.prev_w)
		self.prev_w = self.robot.w

		# direction vector [dx_th, dy_th]
		dx_th = np.cos(self.robot.yaw)
		dy_th = np.sin(self.robot.yaw)
		
		# # update plot
		# self.plotting.update_plot(dx_th, dy_th, self.robot.pose, self.lookahead_point)
		
		# result
		print("cmd_v:", round(self.robot.v, 2), "cmd_w:", round(self.robot.w, 2))
		print(" -------------------------------------- ")
		
		return goal_dist
	
	def find_nearest_ind(self, pose):
		dists = [utils.distance(pose[0], pose[1], p[0], p[1]) for p in self.ref_traj.xy_poses]
		dists = np.array(dists)
		idx = np.argmin(dists)
		return idx, dists

	def eval_side(self, current_idx):
		side = 1
		x_robot_target = self.ref_traj.x[current_idx] - self.robot.x
		y_robot_target = self.ref_traj.y[current_idx] - self.robot.y
		yaw_robot_target = np.arctan2(y_robot_target, x_robot_target)
		ls = yaw_robot_target
		yaw_ref_ls = self.robot.yaw - ls
		yaw_ref_ls = np.arctan2(np.sin(yaw_ref_ls), np.cos(yaw_ref_ls))
		if yaw_ref_ls<0:
			side = -1
		return side