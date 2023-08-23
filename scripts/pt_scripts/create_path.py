import os
import json
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline


class CreatePath:
    """Create reference path and trajectory.
    """

    def __init__(self, data_dir = ''):
        """_summary_

        Args:
            data_dir (str, optional): _description_. Defaults to ''.
        """
        
        self.min_dist = 0.2
        self.data_dir = os.path.join(data_dir, 'Paths')

    def path_1(self):
        """
        This function generates a circular path with a specified radius and number of points.
        """
        self.name = "path1"
        n = 80
        self.r = 10
        self.x_c = 0
        self.y_c = 0
        theta = np.linspace(0, np.pi, n)
        self.x = [self.x_c+self.r*np.cos(t) for t in theta]
        self.y = [self.y_c+self.r*np.sin(t) for t in theta]
        self.yaw = [np.pi/2+t for t in theta]
        self.post_proc()
        
    def path_2(self):
        """
        This function generates a path consisting of two straight lines connected at a right angle.
        """
        self.name = "path2"
        n = 20
        dl = 0.3
        x_init = 12
        x1 = [x_init for i in range(n)]
        y1 = [dl*i for i in range(n)]
        h1 = [np.pi/2 for i in range(n)]
        x2 = [x_init+dl*i for i in range(n)]
        y2 = [dl*n for i in range(n)]
        h2 = [0 for i in range(n)]
        x1.extend(x2)
        y1.extend(y2)
        h1.extend(h2)
        self.x = x1
        self.y = y1
        self.yaw = h1
        self.post_proc()

    def generate_curved_waypoints(self, num_waypoints=5, max_x=20, max_y=20, smoothness=3):
        """create random curved path

        Args:
            num_waypoints (int, optional): number of waypoints. Defaults to 5.
            max_x (int, optional): max x value. Defaults to 20.
            max_y (int, optional): max y value. Defaults to 20.
            smoothness (int, optional): smoothness. Defaults to 3.
        """
        self.name = 'path7'
        x_init = 12
        y_init = 0

        # Generate random waypoints
        x = np.random.uniform(0, max_x, num_waypoints)
        y = np.random.uniform(0, max_y, num_waypoints)

        # Create a smooth spline interpolation
        t = np.arange(num_waypoints)
        spl = make_interp_spline(t, np.c_[x, y], k=smoothness)

        # Generate a new set of evenly spaced points on the spline
        new_t = np.linspace(0, num_waypoints - 1, num_waypoints * 50)
        new_points = spl(new_t)

        # Extract x, y coordinates and compute theta values
        x_list = new_points[:, 0].tolist()
        y_list = new_points[:, 1].tolist()
        theta_list = [np.arctan2(y_list[i+1] - y_list[i], x_list[i+1] - x_list[i]) for i in range(len(x_list)-1)]
        theta_list.append(theta_list[-1])

        x0 = x_list[0]
        y0 = y_list[0]
        x_list = [x - x0 + x_init for x in x_list]
        y_list = [y - y0 + y_init for y in y_list]

        self.x = x_list
        self.y = y_list
        self.yaw = theta_list
        self.post_proc()
        self.to_json(self.name)

    def from_json(self, ind=1, name=None):
        """create reference trajectory from json file

        Args:
            ind (int, optional): _description_. Defaults to 1.
            name (_type_, optional): _description_. Defaults to None.
        """
        ind = str(ind)
        self.name = "pathj"+ind
        file_dir = os.path.join(self.data_dir, 'path' + ind + '.json')
        
        if name is not None:
            file_dir = os.path.join(self.data_dir, name + '.json')
            self.name = name

        with open(file_dir, 'r') as f:
            items = json.load(f)
        self.x = []
        self.y = []
        self.yaw = []
        for item in items:
            self.x.append(item['x'])
            self.y.append(item['y'])
            self.yaw.append(item['theta'])
        self.post_proc()

    def from_plan(self, plan):
        self.name = "global_plan"
        xx, yy, hh = plan[0], plan[1], plan[2]
        count = len(xx)

        # filter points
        i = 0
        selected_inds = []
        while i<count-2:
            selected_inds.append(i)
            xi = xx[i]
            yi = yy[i]
            j = i+1
            dist = np.sqrt((xi - xx[j])**2 + (yi - yy[j])**2)
            while dist<self.min_dist and j<count-1:
                j+=1
                dist = np.sqrt((xi - xx[j])**2 + (yi - yy[j])**2)
            i = j

        self.x = [xx[i] for i in selected_inds]
        self.y = [yy[i] for i in selected_inds]
        self.yaw = [hh[i] for i in selected_inds]
        self.post_proc()

    # ------------------------------------------------------------------

    def post_proc(self):
        """
        calculate metadata for reference trajectory. 
        """
        self.xy_poses = [[self.x[i], self.y[i]] for i in range(len(self.x))]
        self.count = len(self.x)
        self.x_min = min(self.x)
        self.x_max = max(self.x)
        self.y_min = min(self.y)
        self.y_max = max(self.y)
        self.dl = np.sqrt((self.x[0]-self.x[1])**2 + (self.y[0]-self.y[1])**2)

    def plot(self):
        """
        The function plots a line connecting a series of waypoints and marks the start point with a
        green circle.
        """
        plt.plot(self.x, self.y, 'b--', label="waypoints")
        plt.plot(self.x[0], self.y[0], 'go', label="start")
        plt.legend()
        plt.show()

    def to_json(self, name='path'):
        """This function converts a set of x, y, and yaw values into a JSON file.
        """
        items = []
        for i in range(self.count):
            item = {
                'x': round(self.x[i], 2),
                'y': round(self.y[i], 2),
                'theta': round(self.yaw[i], 2)
            }
            items.append(item)
        file_dir = os.path.join(self.data_dir, name + '.json') 
        with open(file_dir, 'w') as f:
            json.dump(items, f)
  
    def speed_profile(self, robot):
        """calculate linear speed profile

        Args:
            robot (Robot): Robot object
        """
        v = [robot.v_max for i in range(self.count)]
        for i in range(0, len(self.yaw)-1):
            dy = self.yaw[i+1] - self.yaw[i]
            dy = np.arctan2(np.sin(dy), np.cos(dy))
            dy = abs(dy)
            if dy >= robot.yaw_thresh_v:
                v_i = 0
            else:
                v_i = v[i]*(1-dy/robot.yaw_thresh_v)
            v_i = max(0.1, v_i)
            v[i] = round(v_i, 3)
        v[-1] = v[-2]
        self.v = v

    def get_pose_vec(self, ind):
        """returns x, y, yaw of waypoint[ind] 

        Args:
            ind (int): index of the waypoint

        Returns:
            list: [x, y, yaw]
        """
        return [self.x[ind], self.y[ind], self.yaw[ind]]
    
    
# ------------------------------------------------------------------

if __name__=="__main__":
    
    cp = CreatePath("/home/piotr/catkin_ws/src/mh/PathTrackPy/data")
    
    # random
    cp.generate_curved_waypoints()
    cp.plot()
