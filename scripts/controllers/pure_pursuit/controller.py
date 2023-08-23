import numpy as np
import scripts.pt_scripts.utils as utils 

class Controller:
    def __init__(self, ref_traj, robot, plotting, dt = 0.2):

        # input data (robot & reference trajectory)
        self.robot = robot
        self.ref_traj = ref_traj
        self.goal_x = ref_traj.x[-1]
        self.goal_y = ref_traj.y[-1]

        # plotting
        self.plotting = plotting

        # Path following parameters
        self.from_beggins = True
        self.dt = dt        # Time step
        self.go_next_thresh = 0.2
        self.lookahead_dist = 0.5
        self.goal_dist_thresh = 0.3

        # # control settings
        self.yaw_thresh_v = np.deg2rad(60)
        # seting pid
        self.Kp = 1.00
        self.Ki = 0.10 
        self.Kd = 0.01
        self.error_sum = 0.0
        self.error_last = 0.0

        # recorded trajectory
        self.rec_traj_x = []
        self.rec_traj_y = []
        self.rec_traj_yaw = []
        self.rec_l = 0
        self.rec_t = 0
        self.rec_w = 0
        self.prev_w = 0
        
    # --------------------------------------- control ---------------------------------------

    def control(self):
        
        # goal distance
        goal_dist = utils.distance(self.goal_x, self.goal_y, self.robot.x, self.robot.y)
        
        # Find the target_index point on the path
        if self.from_beggins:
            target_index = 0
        else:
            distances = [utils.distance(self.robot.x, self.robot.y, p[0], p[1]) for p in self.ref_traj.xy_poses]
            target_index = distances.index(min(distances))
        target_index = self.get_target_index(target_index)

        # main loop
        while (goal_dist>self.goal_dist_thresh or target_index<(self.ref_traj.count)-10):
            
            # calculate velocity
            lookahead_index = self.pure_pursuit(target_index)
            cmd_v, cmd_w = self.move_3()                # move_1, move_2, move_3 --------
            
            # update
            goal_dist = self.update(cmd_v, cmd_w)

            # Find the target_index point on the path
            target_index = self.get_target_index(lookahead_index)

    # -------------------------------------- pure_pursuit ----------------------------------

    def pure_pursuit(self, target_index):
        # Find the lookahead point on the path
        lookahead_index = self.get_lookahead_index(target_index)
        lookahead_point = [self.ref_traj.x[lookahead_index], self.ref_traj.y[lookahead_index]]
        self.lookahead_point = lookahead_point

        # Calculate the look_ahead_angle required to follow the path
        self.look_ahead_angle = np.arctan2(lookahead_point[1] - self.robot.y, lookahead_point[0] - self.robot.x)
        return lookahead_index
        

    def get_lookahead_index(self, target_index):
        # Find the lookahead point on the path
        lookahead_dist = self.lookahead_dist # min(self.lookahead_dist, utils.distance(self.ref_traj.x[-1], self.ref_traj.y[-1], self.robot.x, self.robot.y))
        dist = utils.distance(self.robot.x, self.robot.y, self.ref_traj.x[target_index], self.ref_traj.y[target_index])
        next_index = target_index + 1
        while dist<lookahead_dist and next_index<self.ref_traj.count:
            next_point = self.ref_traj.get_pose_vec(next_index)
            dist = utils.distance(next_point[0], next_point[1], self.robot.x, self.robot.y) 
            next_index = next_index + 1
        return next_index-1
    

    def get_target_index(self, target_index):
        distances = [utils.distance(self.robot.x, self.robot.y, point[0], point[1]) for point in self.ref_traj.xy_poses]
        while distances[target_index]<self.go_next_thresh:
            target_index = target_index + 1
        return target_index
    
    # -----------------------------------   move  ---------------------------------------------------
    
    def move_1(self):
        self.robot.yaw = self.look_ahead_angle
        dx = np.cos(self.robot.yaw)
        dy = np.sin(self.robot.yaw)
        self.robot.x = self.robot.x + dx
        self.robot.y = self.robot.y + dy
        goal_dist = utils.distance(self.goal_x, self.goal_y, self.robot.x, self.robot.y)
        self.rec_traj_x.append(self.robot.x)
        self.rec_traj_y.append(self.robot.y)
        self.rec_traj_yaw.append(self.robot.yaw)
        return goal_dist, dx, dy

    def move_2(self):
        self.yaw_thresh_v = np.deg2rad(60)
        dh_thresh_1 = np.deg2rad(40)
        dh_thresh_2 = np.deg2rad(5)

        h1 = self.robot.yaw
        h2 = self.look_ahead_angle
        dh = utils.angle_diff(h2, h1)
        
         # Calculate the angular velocity
        c_dh_thresh_2 = 1
        if abs(dh) >= dh_thresh_1:
            cmd_w = self.robot.w_max * np.sign(dh)
        else:
            if abs(dh) < dh_thresh_2:
                c_dh_thresh_2 = 0.2
            cmd_w = self.robot.w_max * (1 - (abs(dh) / dh_thresh_1) ** 1) ** 2 * np.sign(dh) * 0.9 * c_dh_thresh_2

        # Calculate the linear velocity based on the heading difference
        if abs(dh) >= self.yaw_thresh_v:
            cmd_v = 0
        else:
            cmd_v = self.robot.v_max * ((1 - (abs(dh) / self.yaw_thresh_v) ** 1) ** 1) * 1.0

        print ("cmd_v:", round(cmd_v, 2), "  cmd_w:", round(cmd_w, 2))
        return cmd_v, cmd_w

    def move_3(self):

        h1 = self.robot.yaw
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
            cmd_v = self.robot.v_max * (1 - (abs(dh) / self.yaw_thresh_v))

        cmd_v = max(self.robot.v_min, min(self.robot.v_max, cmd_v))
        cmd_w = max(self.robot.w_min, min(self.robot.w_max, cmd_w))
        print ("cmd_v:", round(cmd_v, 2), "  cmd_w:", round(cmd_w, 2))

        return cmd_v, cmd_w
    
    # -------------------------------------- update --------------------------------------

    def update(self, cmd_v, cmd_w):
        dl = cmd_v*self.dt  # linear displacement
        dw = cmd_w*self.dt  # angular turn 

        self.robot.yaw = self.robot.yaw + dw # update robot yaw
        # direction vector [dx_th, dy_th]
        dx_th = np.cos(self.robot.yaw)  
        dy_th = np.sin(self.robot.yaw)
        dx = dx_th * dl # x displacement
        dy = dy_th * dl # y displacement

        # update robot pose
        self.robot.update_pose([self.robot.x + dx, self.robot.y + dy, self.robot.yaw])
        
        # update goal distance
        goal_dist = utils.distance(self.goal_x, self.goal_y, self.robot.x, self.robot.y)
        
        # record trajectory 
        self.rec_traj_x.append(self.robot.x)
        self.rec_traj_y.append(self.robot.y)
        self.rec_traj_yaw.append(self.robot.yaw)
        self.rec_l += dl
        self.rec_t += self.dt
        self.rec_w += abs(cmd_w-self.prev_w)
        self.prev_w = cmd_w

        # update plot
        self.plotting.update_plot(dx_th, dy_th, self.robot.pose, self.lookahead_point)

        return goal_dist
