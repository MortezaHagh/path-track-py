import rospy
import numpy as np
import scripts.pt_scripts.utils as utils 

class Controller:
    def __init__(self, ros_interface, ref_traj, robot, plotting, dt = 0.2):

        # input data (robot & reference trajectory)
        self.robot = robot
        self.ref_traj = ref_traj
        self.goal_x = ref_traj.x[-1]
        self.goal_y = ref_traj.y[-1]
        self.rosi = ros_interface

        # Path following parameters
        self.from_beggins = True
        self.dt = dt   # Time step
        self.go_next_thresh = 0.4
        self.lookahead_dist = 0.5
        self.goal_dist_thresh = 1.0

        # # settings
        self.updated_v_max = self.robot.v_max
        self.obst_min_dist_vel = 0.2
        self.obst_min_dist = 0.6
        self.obst_max_dist = 1.0
        self.obst_cost = 95
        self.obstacle_cost = 90
        self.obstacle_infl_cost = 60
        self.diverge_thresh = 2.5

        # # control settings
        self.yaw_thresh_v = np.deg2rad(60)
        # seting pid
        self.Kp = 1.00
        self.Ki = 0.10 
        self.Kd = 0.01
        self.error_sum = 0.0
        self.error_last = 0.0

        # waypoint status
        self.wps_status = ['queue' for i in range(self.ref_traj.count)]

        # recorded trajectory
        self.rec_traj_x = []
        self.rec_traj_y = []
        self.rec_traj_yaw = []
        self.rec_l = 0
        self.rec_t = 0
        self.rec_w = 0
        self.prev_w = 0
        self.start_time = rospy.get_time()
        
        # plotting
        self.plotting = plotting
    
    # --------------------------------------- control ---------------------------------------

    def control(self):
        
        # goal distance
        goal_dist = utils.distance(self.goal_x, self.goal_y, self.rosi.r_pose[0], self.rosi.r_pose[1])
        
        # Find the target_index point on the path
        if self.from_beggins:
            target_index = 0
        else:
            distances = [utils.distance(self.rosi.r_pose[0], self.rosi.r_pose[1], p[0], p[1]) for p in self.ref_traj.xy_poses]
            target_index = distances.index(min(distances))
        target_index = self.get_target_index(target_index)
        lookahead_index = target_index

        dist = 0.0
        obs_flag = False
        if self.rosi.has_cost_map:
            mb_state = self.send_goal(target_index)

        # main loop --------------------------------------------------------------
        while (not rospy.is_shutdown()) and target_index<self.ref_traj.count-1:

			# check goal distance
            if goal_dist<self.goal_dist_thresh and target_index>self.ref_traj.count*0.8:
                print("reached goal")
                break
                                
            if self.rosi.has_cost_map:

                # adjust max speed
                self.adjust_max_speed()

                # check obstacles
                is_obst, lookahead_index, mb_succeed = self.check_obstacle(lookahead_index)
                if is_obst:
                    obs_flag = True
                    if not mb_succeed:
                        self.update_wps_status(lookahead_index, "failed")
                        lookahead_index = lookahead_index + 1
                        target_index = lookahead_index
                    continue
                elif obs_flag:
                    mb_state = self.send_goal(target_index)
                    while not mb_state:
                        target_index+=1
                        mb_state = self.send_goal(target_index)
                    obs_flag = False

                # check distance and divergence
                is_dv, mb_succeed = self.check_divergence(dist, lookahead_index)
                if is_dv:
                    if not mb_succeed:
                        self.update_wps_status(lookahead_index, "failed")
                        target_index = lookahead_index + 1
                    continue

            # calculate velocity
            lookahead_index, dist = self.pure_pursuit(target_index)
            cmd_v, cmd_w = self.move_3()                # move_1, move_2, move_3 --------
            
            # update
            goal_dist = self.update(cmd_v, cmd_w)
            self.rosi.update(cmd_v, cmd_w, self.lookahead_point)
        
            # Find the target_index point on the path
            target_index = self.get_target_index(lookahead_index)
        
        # -------------------------------------------------------------------------

        self.rec_t = rospy.get_time() - self.start_time
        self.rosi.stop(msg="finished")
        
    # -------------------------------------- pure_pursuit ----------------------------------

    def pure_pursuit(self, target_index):
        # Find the lookahead point on the path
        lookahead_index, dist = self.get_lookahead_index(target_index)
        lookahead_point = [self.ref_traj.x[lookahead_index], self.ref_traj.y[lookahead_index]]
        self.lookahead_point = lookahead_point

        # Calculate the look_ahead_angle required to follow the path
        self.look_ahead_angle = np.arctan2(lookahead_point[1] - self.rosi.r_pose[1], lookahead_point[0] - self.rosi.r_pose[0])
        return lookahead_index, dist
        

    def get_target_index(self, target_index):
        distances = [utils.distance(self.rosi.r_pose[0], self.rosi.r_pose[1], point[0], point[1]) for point in self.ref_traj.xy_poses]
        while distances[target_index]<self.go_next_thresh:
            target_index = target_index + 1
        return target_index


    def get_lookahead_index(self, target_index):
        # Find the lookahead point on the path
        lookahead_dist = self.lookahead_dist # min(self.lookahead_dist, utils.distance(self.ref_traj.x[-1], self.ref_traj.y[-1], self.rosi.r_pose[0], self.rosi.r_pose[1]))
        dist = utils.distance(self.rosi.r_pose[0], self.rosi.r_pose[1], self.ref_traj.x[target_index], self.ref_traj.y[target_index])
        next_index = target_index + 1
        while dist<lookahead_dist and next_index<self.ref_traj.count:
            next_point = self.ref_traj.get_pose_vec(next_index)
            dist = utils.distance(next_point[0], next_point[1], self.rosi.r_pose[0], self.rosi.r_pose[1]) 
            next_index = next_index + 1
        return next_index-1, dist
    
    def adjust_max_speed(self):
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

        print(" ==== updated_v_max ", round(self.updated_v_max, 2))

    # -----------------------------------   move  ---------------------------------------------------
    
    def move_3(self):

        h1 = self.rosi.r_pose[2]
        h2 = self.look_ahead_angle
        dh = utils.angle_diff(h2, h1)
        error = dh

        # pid
        # Calculate proportional term
        p_term = self.Kp * error
        # Calculate integral term
        self.error_sum += error * self.dt
        i_term = self.Ki * self.error_sum
        # Calculate derivative term
        d_term = self.Kd * (error - self.error_last) / self.dt

        # Update last error
        self.error_last = error

        # Calculate total correction
        correction = p_term + i_term + d_term

        cmd_w = correction

        # Calculate the linear velocity based on the heading difference
        if abs(dh) >= self.yaw_thresh_v:
            cmd_v = 0
        else:
            cmd_v = self.updated_v_max * (1 - (abs(dh) / self.yaw_thresh_v))

        cmd_v = max(self.robot.v_min, min(self.robot.v_max, cmd_v))
        cmd_w = max(self.robot.w_min, min(self.robot.w_max, cmd_w))

        return cmd_v, cmd_w
    
    # -------------------------------------- --------------------------------------

    def check_divergence(self, min_dist, index):
        mb_state = True
        if min_dist<self.diverge_thresh:
            return False, mb_state
        
        # if distance bigger than diverge_thresh: go to target point
        self.rosi.stop("check_divergence")
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


    def update_wps_status(self, index, status):
        # status: queue, failed, skipped, done 
        self.wps_status[index] = status
                

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
