import numpy as np


class Robot:
    """The above class defines a Robot object with state variables and methods for updating the state, velocity, and pose of the robot.
    """

    def __init__(self, x=0, y=0, yaw=0, v=0, w=0):
		
        # state
        self.x = x
        self.y = y
        self.yaw = yaw # heading angle
        self.v = v  # linear velocity
        self.w = w  # angular velocity
        self.pose = [x, y, yaw]

        self.dt = 0.1

        # Robot velocity limits
        self.v_max = 0.5
        self.v_min = 0.0
        self.w_max = 1.5
        self.w_min = -1.5

        # robot angular thresholds
        self.steer_max = np.deg2rad(45.0)  # maximum steering angle[rad]
        self.yaw_thresh_v = np.deg2rad(45) # for trajectory speed calculation (speed profile)

        # robot geometric data
        self.wheelbase = 0.5  # distance between the two wheels in meters
        

        # state variables
        self.x_state = np.array([
                            [x],
                            [y],
                            [0]
                            ])

        self.x_dot = np.array([
                            [0],
                            [0],
                            [0]
                            ])

    # set --------------------------------

    def set_robot_velocity(self, linear_velocity, angular_velocity):
        self.update_velocity(v = linear_velocity, w = angular_velocity)
        self.x_dot = np.array([
                                [linear_velocity],
                                [0],
                                [angular_velocity]
                                ])

    def set_dt(self, dt):
        self.dt = dt

    # get --------------------------------

    def get_state(self):
        return self.x_state, self.x_dot

    def get_model_pose(self):
        return [self.x_state[0, 0], self.x_state[1, 0], self.x_state[2, 0]]

    def get_pose_vec(self):
        return [self.x, self.y, self.yaw]

    def get_velocity_vec(self):
        return [self.v, self.w]

    def get_los(self, point):
        xp = point[0]
        yp = point[1]
        x = self.x_state[0, 0]
        y = self.x_state[1, 0]
        theta = np.arctan2(yp-y, xp-x)
        return theta
    
    # update --------------------------------

    def update_pose(self, pose):
        self.x = pose[0]
        self.y = pose[1]
        self.yaw = pose[2]
        self.pose = pose

    def update_velocity(self, v, w):
        self.v = v
        self.w = w

    def update_state(self, dt):
        A = np.array([
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]
                    ])
		
        B = np.array([
                        [np.cos(self.x_state[2, 0])*dt,  0],
                        [np.sin(self.x_state[2, 0])*dt,  0],
                        [0					 , dt]
                    ])

        u = np.array([
                            [self.x_dot[0, 0]],
                            [self.x_dot[2, 0]]
                        ])
        self.x_state = np.dot(A, self.x_state) + np.dot(B, u)

    def update_sim(self, dt):
        self.update_state(dt)

    def update_robot(self, pose):
        self.update_pose(pose)
        self.x_state = np.array([
                            [pose[0]],
                            [pose[1]],
                            [pose[2]]
                            ])

    def update_lqr(self, acc, w):
        self.x = self.x + self.v * np.cos(self.yaw) * self.dt
        self.y = self.y + self.v * np.sin(self.yaw) * self.dt
        self.yaw = self.yaw + w * self.dt
        self.v = self.v + acc * self.dt
        self.w = w
        self.pose = [self.x, self.y, self.yaw]