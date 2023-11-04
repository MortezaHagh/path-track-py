import rospy
import numpy as np
from copy import deepcopy
from scipy.optimize import minimize
import scripts.pt_scripts.utils as utils 

class RefData:
	"""reference data class: for MPC optimization function at each iteration
	"""	
	def __init__(self):
		self.dist_flag = False
		self.updated_v_max = 0
		self.indices = []
		self.vels = []
		self.yaws = []
		self.xs = []
		self.ys = []
		self.ij = []


class Controller:
	def __init__(self, ros_interface, ref_traj, robot, plotting, dt = 0.2):
		
		# input data
		self.dt = dt
		self.robot = robot
		self.ref_traj = ref_traj
		self.rosi = ros_interface
		self.plotting = plotting

		# set controller
		self.controller_name = "MPC"	# PID or MPC

		# general settings
		self.from_beggins = True

		# initialization
		self.initialize()


	def initialize(self):
		
		# # settings -----------------------------------------------------------
		self.dt2 = self.dt # 0.5 self.dt
		self.obstacle_cost = 90
		self.obstacle_infl_cost = 60
		self.dist_thresh = 0.6
		self.diverge_thresh = 1.8
		self.goal_dist_thresh = 0.2
		self.dist_thresh_horizon = 1.1
		self.next_target_dist_thresh = 2
		
		# settings, set the next target if velocity is less than these values:
		self.stop_v_thresh = 0.06 
		self.stop_w_thresh = np.deg2rad(4)
		self.yaw_thresh_v = np.deg2rad(60)
		
		# settings, max velocity calculaiton
		self.updated_v_max = self.robot.v_max
		self.obst_min_dist_vel = 0.2
		self.obst_min_dist = 0.6
		self.obst_max_dist = 1.0
		self.obst_cost = 95

		# MPC settings
		self.horizon_max = 2
		self.horizon = 2

		# PID settings
		pid_linear = {'kp': 0.5, 'kd': 0.1, 'ki': 0}
		pid_angular = {'kp': 3.0, 'kd': 0.1, 'ki': 0.1}
		pid_params = {'linear':pid_linear, 'angular':pid_angular}
		# -------------------------------------------------------------------------

		# create controller
		if self.controller_name == "PID":
			self.controller = PID(pid_params)
		elif self.controller_name == "MPC":			
			self.controller = MPC(self.dt2, horizon = self.horizon)

		# goal point
		self.goal_x = self.ref_traj.x[-1]
		self.goal_y = self.ref_traj.y[-1]

		# waypoint status
		self.wps_status = ['queue' for i in range(self.ref_traj.count)]

		# recorded trajectory
		self.rec_l = 0
		self.rec_t = 0
		self.rec_w = 0
		self.prev_w = 0
		self.rec_traj_x = []
		self.rec_traj_y = []
		self.rec_traj_yaw = []
		self.start_time = rospy.get_time()

# *************************************************************************************************************** 

	def control(self):

		# initial distance to the goal
		goal_dist = utils.distance(self.goal_x, self.goal_y, self.rosi.r_pose[0], self.rosi.r_pose[1])

		# starting index
		if self.from_beggins:
			current_idx = 0
		else:
			current_idx, dists = self.find_nearest_ind(self.rosi.r_pose)
		
		min_dist = 0.0
		obst_flag = False

		if self.rosi.has_cost_map:
			mb_state = self.send_goal(current_idx)
		
		# main control loop --------------------------------------------------------------
		while (not rospy.is_shutdown()) and (current_idx < self.ref_traj.count):
			
			# check goal distance
			if goal_dist<self.goal_dist_thresh and current_idx>self.ref_traj.count*0.8:
				print("reached goal")
				break
			
			if self.rosi.has_cost_map:

				# adjust max speed
				self.adjust_speed_horizon()

				# check obstacles
				is_obst, current_idx, mb_succeed = self.check_obstacle(current_idx)
				if is_obst:
					obst_flag = True
					if not mb_succeed:
						self.update_wps_status(current_idx, "failed")
						current_idx+=1
					continue
				elif obst_flag:
					mb_state = self.send_goal(current_idx)
					while not mb_state:
						current_idx += 1
						mb_state = self.send_goal(current_idx)
					obst_flag = False
				
				# check distance and divergence
				is_dv, mb_succeed = self.check_divergence(min_dist, current_idx)
				if is_dv:
					if not mb_succeed:
						self.update_wps_status(current_idx, "failed")
						current_idx+=1
					continue
			
			# update target point (current_idx) and next ref data for controller
			[ref_data, min_dist, current_idx] = self.update_index_horizon(current_idx)

			# robot pose and lookahead point
			lookahead_point = self.ref_traj.get_pose_vec(current_idx)
			self.lookahead_point = lookahead_point

			# calculate velocity
			if self.controller_name == "PID":
				cmd_v, cmd_w = self.controller.get_control_inputs(self.rosi.r_pose, lookahead_point) # self.robot.get_points()[2]
				u = [cmd_v, cmd_w]

			if self.controller_name == "MPC":
				u = self.controller.optimize(robot = self.robot, ref_data = ref_data)
				cmd_v, cmd_w = u[0], u[1]

			# predict
			self.predict(self.robot, u)

			# # check vel and updarw target point (current_idx) id necessary
			# if cmd_v<self.stop_v_thresh and abs(cmd_w)<self.stop_w_thresh:
			# 	if min_dist<self.next_target_dist_thresh:
			# 		current_idx += 1
			# 		self.update_wps_status(current_idx, "failed")
			# 		# continue

			# check velocity bounds
			cmd_v = min(max(cmd_v, self.robot.v_min), self.robot.v_max)
			cmd_w = min(max(cmd_w, self.robot.w_min), self.robot.w_max)

			# set and update
			self.robot.set_robot_velocity(cmd_v, cmd_w)
			self.robot.update_robot(self.rosi.r_pose)
			self.rosi.update(cmd_v, cmd_w, self.lookahead_point)
			goal_dist = self.update(cmd_v, cmd_w)

		# finish
		self.rec_t = rospy.get_time() - self.start_time
		self.rosi.stop(msg="finished")
	
	
	# *************************************************************************************************************** 

	# -------------------------------------- vel - horizon - ref data  --------------------------------------
	
	def adjust_speed_horizon(self):
		
		dr = 0.2
		nr = 20
		r1 = 0.8
		r2 = 1.5
		ntheta = 100
		dtheta1 = 45
		dtheta2 = 45

		xr = self.rosi.r_pose[0]
		yr = self.rosi.r_pose[1]
		
		theta1 = self.rosi.r_pose[2] - np.deg2rad(dtheta1)
		theta2 = self.rosi.r_pose[2] + np.deg2rad(dtheta2)
		
		r_v = np.linspace(r1, r2, nr)
		theta_v = np.linspace(theta1, theta2, ntheta)
		
		x = None
		y = None
		flag = False
		for r in r_v:
			for t in theta_v:
				x = xr + r * np.cos(t)
				y = yr + r * np.sin(t)
				try:
					cost = self.rosi.ogm.get_cost_from_world_x_y(x, y)
				except:
					cost = self.obst_cost + 1 
				if cost > self.obst_cost:
					flag = True
					break
			if flag:
				break
		
		# update max velocity based on the distance
		self.updated_v_max = self.robot.v_max
		if flag:
			closest_dist = utils.distance(xr, yr, x, y)
			if closest_dist<self.obst_min_dist:
				self.updated_v_max = self.obst_min_dist_vel
			elif closest_dist<self.obst_max_dist:
				self.updated_v_max = self.obst_min_dist_vel + (self.updated_v_max-self.obst_min_dist_vel)*(closest_dist-self.obst_min_dist)/(self.obst_max_dist-self.obst_min_dist)
			self.controller.updated_v_max = self.updated_v_max

		# adjust horizon
		self.horizon =int(self.horizon_max*(self.updated_v_max/self.robot.v_max))
		self.horizon = max(1, self.horizon)

		print(" ==== updated_v_max ", round(self.updated_v_max, 2))
		print(" ==== updated_horizon ", self.horizon)


	def update_index_horizon(self, current_idx):
		# update index and horizon
			_, dists = self.find_nearest_ind(self.rosi.r_pose, current_idx)

			# update current_idx (target_point)
			i = 0
			min_dist = dists[0]
			while min_dist<self.dist_thresh and i!= len(dists)-1:
				i+= 1
				k = min(self.horizon, len(dists)-i)
				min_dist = min(dists[i:i+k])
			current_idx += i

			# update waypoints status
			for ind in range(current_idx-i, current_idx):
				self.update_wps_status(ind, "done")

			# # calculate horizon indices
			# m4
			
			# current_v, _ = self.robot.get_velocity_vec()
			# current_l = current_v * self.dt
			# dists_horizon = [dists[i]+self.ref_traj.dl*j for j in range(self.horizon)]

			current_l = self.dist_thresh 
			dists_horizon = dists[i:i+self.horizon]
			# dists_horizon = [dists[i]+self.ref_traj.dl*j for j in range(self.horizon)]

			i = 0
			dm = 0
			ref_ind = [current_idx]
			while len(ref_ind)<self.horizon-1 and i<len(dists_horizon):
				dm += current_l
				if dm<dists_horizon[i]:
					pass
				else:
					i+=1
				ii = min(current_idx+i, self.ref_traj.count-1)
				ref_ind.append(ii)
			
			# set reference data for this iteration
			ref = RefData()
			ref.indices = ref_ind
			ref.count = len(ref_ind)
			ref.updated_v_max = self.updated_v_max
			ref.xs = [self.ref_traj.x[i] for i in ref_ind]
			ref.ys = [self.ref_traj.y[i] for i in ref_ind]
			ref.vels = [self.ref_traj.v[i] for i in ref_ind]
			ref.yaws = [self.ref_traj.yaw[i] for i in ref_ind]

			# ij - for lateral distance calculation
			ij=[]
			ref_ind_2 = ref_ind
			flag_dist = False
			if len(set(ref_ind_2))>1:
				flag_dist = True
				temp_inds = list(range(len(ref_ind)))
				temp_inds.append(temp_inds[-1])
				ref_ind_2.append(ref_ind_2[-1])
				lri = len(ref_ind_2)
				ij = []
				for i in range(lri-1):
					flag = True
					for j in range(i+1, lri):
						if ref_ind_2[i] !=ref_ind_2[j]:
							ij.append([temp_inds[i], temp_inds[j]])
							flag = False
							break
					if flag:
						ij.append(ij[-1])
			ref.dist_flag = flag_dist
			ref.ij = ij

			# information
			print("ref_path len: ", len(ref.indices))
			print("current_idx: ", current_idx)
			print("ref_ind: ", ref_ind)

			return [ref, min_dist, current_idx]


	def find_nearest_ind(self, pose, current_idx):
		dists = [utils.distance(pose[0], pose[1], p[0], p[1]) for p in self.ref_traj.xy_poses[current_idx:]]
		dists = np.array(dists)
		idx = current_idx
		return idx, dists
	
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
	
		# # heading
		# dx_th = np.cos(self.rosi.r_pose[2])
		# dy_th = np.sin(self.rosi.r_pose[2])
		
		# # update plot
		# self.plotting.update_plot(dx_th, dy_th, self.rosi.r_pose, self.lookahead_point)
		
		return goal_dist
	
	# -------------------------------------- Check --------------------------------------
	
	def update_wps_status(self, index, status):
		# status: queue, failed, skipped, done 
		self.wps_status[index] = status
	
	
	def check_divergence(self, min_dist, index):
		mb_state = True
		if min_dist<self.diverge_thresh:
			return False, mb_state
		
		# if distance bigger than diverge_thresh: go to target point
		self.rosi.stop(msg="check_divergence")
		mb_state = self.send_goal(index)
		return  True, mb_state
		

	def check_obstacle(self, index):
		mb_state = True
		obst_ind = index
		flag_obst = False
		for i in range(index, min(index+10, self.ref_traj.count)):
			x = self.ref_traj.x[i]
			y = self.ref_traj.y[i]
			cost = self.rosi.ogm.get_cost_from_world_x_y(x, y)
			if cost > self.obstacle_cost:
				self.rosi.stop("check_obstacle")
				flag_obst = True
				obst_ind = i
				break
		if not flag_obst:
			return False, index, True
		
		for j in range(index, obst_ind):
			self.update_wps_status(j, "failed")
		
		index = obst_ind

		self.rosi.stop(msg="check_obstacle")
		while cost > self.obstacle_infl_cost:
			self.update_wps_status(index, "failed")
			index+=1
			x = self.ref_traj.x[index]
			y = self.ref_traj.y[index]
			cost = self.rosi.ogm.get_cost_from_world_x_y(x, y)
			if  index==self.ref_traj.count-1:
				mb_state = False
				return True, index, mb_state

		mb_state = self.send_goal(index)
		return True, index, mb_state

	# -------------------------------------- send_goal - predict --------------------------

	def send_goal(self, index):
		x= self.ref_traj.x[index]
		y= self.ref_traj.y[index]
		yaw= self.ref_traj.yaw[index]
		self.rosi.send_goal(x, y, yaw)

		# check state
		while self.rosi.mb_goal_state==-1:
			self.rosi.rate.sleep()
		
		if self.rosi.mb_goal_state==1:  # successful
			return True
		else:
			print("going to point " + str(index) + " failed")
			return False	# failed, should go to next point
		
	def predict(self, robot, u):
		pred_poses = []
		controller_robot = deepcopy(robot)
		for i in range(len(u)/2):
			controller_robot.set_robot_velocity(u[i*2], u[i*2+1])
			controller_robot.update_sim(self.dt2)
			current_pose, _ = controller_robot.get_state()
			xr = current_pose[0, 0] # + 0.3 * np.cos(current_pose[2,0])
			yr = current_pose[1, 0] # + 0.3 * np.sin(current_pose[2,0])
			cp = [xr, yr, current_pose[2,0]]
			pred_poses.append(cp)
		self.rosi.pred_pub(pred_poses)

# -------------------------------------- Controllers --------------------------------------
# -------------------------------------- PID ----------------------------------------------

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

		# settings
		self.dt = dt
		self.horizon = horizon
		self.R = np.diag([0.01, 0.01])		# input cost matrix [0.01, 0.01] v2: [0.01, 0.01]
		self.Rd = np.diag([0.01, 0.1])		# input difference cost matrix [0.01, 1.0] v2: [0.01, 0.01]
		self.Q = np.diag([1.0, 1.0])		# state cost matrix
		self.Qf = self.Q					# state final matrix
		self.C_heading = 0.1						# heading cost matrix
		self.C_Yaw = 0.05
		self.CVW = 0.03
		self.CVH = 0.1
		self.C_lat_dist = 0.1

	def cost(self, u_k, robot, ref_data):
		
		# data
		xx = ref_data.xs
		yy = ref_data.ys
		ij = ref_data.ij
		yaws = ref_data.yaws
		self.horizon = ref_data.count
		controller_robot = deepcopy(robot)
		u_k = u_k.reshape(self.horizon, 2).T
		z_k = np.zeros((2, self.horizon+1))

		self.pred_poses=[]
		
		# cost
		cost = 0.0
		for i in range(self.horizon):
			#
			controller_robot.set_robot_velocity(u_k[0,i], u_k[1,i])
			controller_robot.update_sim(self.dt)
			current_pose, _ = controller_robot.get_state()
			xr = current_pose[0, 0] # + 0.3 * np.cos(current_pose[2,0])
			yr = current_pose[1, 0] # + 0.3 * np.sin(current_pose[2,0])
			z_k[:,i] = [xr, yr]
			los = controller_robot.get_los([xx[i], yy[i]])
			ad_h_los = (utils.angle_diff(los, current_pose[2, 0]))
			ad_h_yaw = (utils.angle_diff(yaws[i], current_pose[2, 0]))

			# # dist cost
			# if ref_data.dist_flag:
			# 	p = ij[i][0]
			# 	q = ij[i][1]
			# 	d = ((xx[q]-xx[p])*(yr-yy[p]) - (yy[q]-yy[p])*(xr-xx[p])) / np.sqrt((xx[q]-xx[p])**2 + (yy[q]-yy[p])**2)
			# 	d = min(d, 1)
			# 	cost += self.C_lat_dist * abs(d)

			# costs
			cost += self.CVW * (u_k[0,i]*u_k[1,i])**2  #/np.deg2rad(45)
			cost += self.C_heading*ad_h_los**2
			cost += self.C_Yaw * (ad_h_yaw)**2
			cost += self.CVH*(u_k[0,i]*ad_h_los**2)

			cost += np.sum(np.dot(self.R, u_k[:,i]**2))  			   					#	np.sum(self.R@(u_k[:,i]**2))
			cost += np.sum(np.dot(self.Q, np.array([xx[i], yy[i]])-z_k[:,i])**2) 		#	np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
			if i < (self.horizon-1):     
				cost += np.sum(np.dot(self.Rd, u_k[:,i+1] - u_k[:,i])**2)  				#	np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))

		return cost

	def optimize(self, robot, ref_data):
		self.horizon = len(ref_data.xs)
		bnd = [(0.0, ref_data.updated_v_max),(np.deg2rad(-45), np.deg2rad(45))]*self.horizon
		result = minimize(self.cost, args=(robot, ref_data), x0 = np.zeros((2*self.horizon)), method='SLSQP', bounds = bnd)
		return result.x
