import rospy
import numpy as np
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
		self.rosi = ros_interface
		
		# plotting
		self.plotting = plotting

		# controller
		self.from_beggins = True
		self.controller_name = "MPC"	# PID or MPC

		# # settings
		self.dt = dt
		self.dist_thresh = 0.6
		self.goal_dist_thresh = 1.0
		self.dist_thresh_horizon = 1.2
		# set the next target if velocity is less than these values:
		self.stop_v_thresh = 0.06 
		self.stop_w_thresh = np.deg2rad(4)

		# MPC settings
		self.horizon = 5

		# PID settings
		pid_linear = {'kp': 0.5, 'kd': 0.1, 'ki': 0}
		pid_angular = {'kp': 3.0, 'kd': 0.1, 'ki': 0}
		pid_params = {'linear':pid_linear, 'angular':pid_angular}

		# create controller
		if self.controller_name == "PID":
			self.controller = PID(pid_params)
		elif self.controller_name == "MPC":			
			self.controller = MPC(self.dt, horizon = self.horizon)

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

		while (current_idx < self.ref_traj.count):
			if goal_dist<self.goal_dist_thresh:
				break

			# update index and horizon
			_, dists = self.find_nearest_ind(self.rosi.r_pose)
			min_dist = min(dists[current_idx:current_idx+self.horizon])
			while min_dist<self.dist_thresh and current_idx!= len(dists)-1:
				current_idx+= 1
				min_dist = min(dists[current_idx:current_idx+self.horizon])

			horizon = 1
			for i in range(1, self.horizon):
				if (current_idx+i)<self.ref_traj.count-1 and dists[current_idx+i]<self.dist_thresh_horizon:
					horizon += 1
				else:
					break

			# robot pose and lookahead point
			lookahead_point = self.ref_traj.get_pose_vec(current_idx)
			self.lookahead_point = lookahead_point

			# calculate velocity
			if self.controller_name == "PID":
				cmd_v, cmd_w = self.controller.get_control_inputs(self.rosi.r_pose, lookahead_point) # self.robot.get_points()[2]
			
			if self.controller_name == "MPC":
				cmd_v, cmd_w = self.controller.optimize(robot = self.robot, points = self.ref_traj.xy_poses[current_idx:current_idx+horizon])

			# check vel
			if cmd_v<self.stop_v_thresh and abs(cmd_w)<self.stop_w_thresh:
				current_idx += 1
				# continue

			cmd_v = min(max(cmd_v, self.robot.v_min), self.robot.v_max)
			cmd_w = min(max(cmd_w, self.robot.w_min), self.robot.w_max)

			# update
			self.robot.set_robot_velocity(cmd_v, cmd_w)
			self.robot.update_robot(self.rosi.r_pose)
			self.rosi.update(cmd_v, cmd_w, self.lookahead_point)
			goal_dist = self.update(cmd_v, cmd_w)

		self.rec_t = rospy.get_time() - self.start_time
		self.rosi.stop()
			
	# -------------------------------------- update --------------------------------------

	def update(self, cmd_v, cmd_w):
		# goal distance
		goal_dist = utils.distance(self.goal_x, self.goal_y, self.rosi.r_pose[0], self.rosi.r_pose[1])
		
		# trajectory
		self.rec_traj_x.append(self.rosi.r_pose[0])
		self.rec_traj_y.append(self.rosi.r_pose[1])
		self.rec_traj_yaw.append(self.rosi.r_pose[2])
		self.rec_w += abs(cmd_w - self.prev_w)
		self.prev_w = cmd_w
	
		# heading
		dx_th = np.cos(self.rosi.r_pose[2])
		dy_th = np.sin(self.rosi.r_pose[2])
		
		# # update plot
		# self.plotting.update_plot(dx_th, dy_th, self.rosi.r_pose, self.lookahead_point)
		
		return goal_dist
	
	def find_nearest_ind(self, pose):
		dists = [utils.distance(pose[0], pose[1], p[0], p[1]) for p in self.ref_traj.xy_poses]
		dists = np.array(dists)
		idx = np.argmin(dists)
		return idx, dists

# -------------------------------------- PID --------------------------------------

class PID:
	def __init__(self, pid_params):
		self.kp_linear = pid_params['linear']['kp']
		self.kd_linear = pid_params['linear']['kd']
		self.ki_linear = pid_params['linear']['ki']

		self.kp_angular = pid_params['angular']['kp']
		self.kd_angular = pid_params['angular']['kd']
		self.ki_angular = pid_params['angular']['ki']

		self.error_ang_last = 0
		self.error_lin_last = 0

		# self.prev_body_to_goal = 0
		# self.prev_waypoint_idx = -1


	def get_control_inputs(self, current_pose, goal_x):
		error_position = utils.distance(current_pose[0], current_pose[1], goal_x[0], goal_x[1])
		
		body_to_goal = np.arctan2(goal_x[1]- current_pose[1], goal_x[0] - current_pose[0])
		error_angle = utils.angle_diff(body_to_goal, current_pose[2])

		linear_velocity_control = self.kp_linear*error_position + self.kd_linear*(error_position - self.error_lin_last)
		angular_velocity_control = self.kp_angular*error_angle + self.kd_angular*(error_angle - self.error_ang_last)

		self.error_ang_last = error_angle
		self.error_lin_last = error_position

		# self.prev_waypoint_idx = waypoint_idx
		# self.prev_body_to_goal = body_to_goal

		if linear_velocity_control>5:
			linear_velocity_control = 5

		return linear_velocity_control, angular_velocity_control

# -------------------------------------- MPC --------------------------------------

class MPC:
	def __init__(self, dt, horizon):
		self.dt = dt
		self.horizon = horizon
		self.R = np.diag([0.01, 0.01])		# input cost matrix [0.01, 0.01] v2: [0.01, 0.01]
		self.Rd = np.diag([0.01, 0.01])		# input difference cost matrix [0.01, 1.0] v2: [0.01, 0.01]
		self.Q = np.diag([1.0, 1.0])		# state cost matrix
		self.Qf = self.Q					# state final matrix
		self.H = 0.5						# heading cost matrix
		self.CVW = 0.1

	def cost(self, u_k, robot, path):
		path = np.array(path)
		controller_robot = deepcopy(robot)
		u_k = u_k.reshape(self.horizon, 2).T
		z_k = np.zeros((2, self.horizon+1))
		# h_k = np.zeros((1, self.horizon+1))

		desired_state = path.T

		cost = 0.0

		C = 1
		for i in range(self.horizon):
			controller_robot.set_robot_velocity(u_k[0,i], u_k[1,i])
			controller_robot.update_sim(self.dt)
			current_pose, _ = controller_robot.get_state()
			z_k[:,i] = [current_pose[0, 0], current_pose[1, 0]]
			h = controller_robot.get_los(path[i])

			if i ==0:
				C = 1
			cost += self.H*utils.angle_diff(h, current_pose[2, 0])**2
			cost += self.CVW * (u_k[0,i]*u_k[1,i])**2

			cost += np.sum(np.dot(self.R, u_k[:,i]**2))  			   			#	np.sum(self.R@(u_k[:,i]**2))
			cost += C*np.sum(np.dot(self.Q, desired_state[:,i]-z_k[:,i])**2) 	#	np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
			if i < (self.horizon-1):     
				cost += np.sum(np.dot(self.Rd, u_k[:,i+1] - u_k[:,i])**2)  		#	np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))

		return cost

	def optimize(self, robot, points):
		self.horizon = len(points)
		bnd = [(0.0, 0.5),(np.deg2rad(-45), np.deg2rad(45))]*self.horizon
		result = minimize(self.cost, args=(robot, points), x0 = np.zeros((2*self.horizon)), method='SLSQP', bounds = bnd)
		return result.x[0],  result.x[1]
