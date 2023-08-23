import json
import rospkg
import sys, os
from pt_scripts.robot import Robot
import pt_scripts.utils as utils
from pt_scripts.plotting import Plotting
from pt_scripts.create_path import CreatePath

sys.path.insert(1, '/home/piotr/catkin_ws/src/mh/path_tracking')

METHOD = 'MPC2'  # LQR - MPC2 - MPC - PurePursuit
if METHOD =='MPC':
    r_path = '/home/piotr/catkin_ws/src/mh/path_tracking/scripts/controllers/mpc/'
elif METHOD == 'MPC2':
    r_path = '/home/piotr/catkin_ws/src/mh/path_tracking/scripts/controllers/mpc2/'
elif METHOD == 'PurePursuit':
    r_path = '/home/piotr/catkin_ws/src/mh/path_tracking/scripts/controllers/pure_pursuit/'
elif METHOD == 'LQR':
    r_path = '/home/piotr/catkin_ws/src/mh/path_tracking/scripts/controllers/lqr/'

sys.path.insert(0, r_path)
from controller import Controller



class Tracking:
    """Tracking Class
    """

    def __init__(self):
        
        # initialize
        self.initialize()

        # run tracker
        self.run()

        # final plot and post processing
        self.final()
    
    # ----------------------- initialize -----------------------------------

    def initialize(self):
        """
            Create reference path or trajectory.
            Create Robot object.
            Create Initial Plot.
            Create Controller (tracker).
        """
        
        # base directory
        ros_pkg = rospkg.RosPack()
        pkg_dir = ros_pkg.get_path('path_tracking')
        data_dir = os.path.join(pkg_dir, 'data')

        # create reference Path/Trajectory/Waypoints
        ref_traj = CreatePath(data_dir)
        ref_traj.path_1()         # path_1, path_2, from_json(Volkswagen_results), generate_curved_waypoints  ************ Volkswagen_results_2
        self.ref_traj = ref_traj
        goal = [ref_traj.x[-1], ref_traj.y[-1]]  # last waypoint

        # data
        name = METHOD + '_' + self.ref_traj.name
        save_pic_path = os.path.join(data_dir, 'Pics')
        self.save_pic_path = os.path.join(save_pic_path, name)
        save_res_path = os.path.join(data_dir, 'Results')
        self.save_res_path = os.path.join(save_res_path, name +'.json')

        # create robot with initial pose and velocity
        robot_ = Robot(x=ref_traj.x[0]-2, y=ref_traj.y[0], yaw=0, v=0, w=0)

        # speed profile
        ref_traj.speed_profile(robot_)

        # plotting
        self.my_plot = Plotting(ref_traj, robot_.pose, goal)

        # controller
        self.dt = 0.2
        self.controller = Controller(ref_traj, robot_, self.my_plot, self.dt)

    # ----------------------- run -----------------------------------

    def run(self):
        """Start tracking
        """
        print("tracking started!")
        self.controller.control()
        print("tracking finished!")

    # ----------------------- final -----------------------------------

    def final(self):
        """
        This function calculates and reports errors, and then plots the final trajectory.
        """
        
        # results reporting
        self.results()
        print("error: ", self.dist_error)

        # plot final
        print("final plotting ...")
        self.my_plot.plot_final(self.save_pic_path, self.controller.rec_traj_x, self.controller.rec_traj_y)

    def results(self):
        """processing and saving path tracking results.
            Calculate error.
        """
        trav_len = round(self.controller.rec_l, 2)
        trav_t = round(self.controller.rec_t, 2)
        trav_w = round(self.controller.rec_w, 2)
        rec_traj_x = self.controller.rec_traj_x
        rec_traj_y = self.controller.rec_traj_y
        dist_error = 0
        for x, y in zip(rec_traj_x, rec_traj_y):
            dists = [utils.distance(x, y, self.ref_traj.x[i], self.ref_traj.y[i]) for i in range(self.ref_traj.count)]
            dist_min = min(dists)
            dist_error += dist_min
        self.dist_error = round(dist_error, 3)

        save_path = self.save_res_path
        data = {'method': METHOD, 'path': self.ref_traj.name, 'dist_error': self.dist_error, 'len': trav_len, 't': trav_t, 'trav_w':trav_w, "rec_traj_x":rec_traj_x, "rec_traj_y":rec_traj_y}
        with open(save_path, "w") as outfile:
            json.dump(data, outfile, indent=2)
            outfile.write("\n")
                

# ------------------------------------------------------------------

if __name__=="__main__":
    tr = Tracking()