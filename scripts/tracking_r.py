#! /usr/bin/env python

import json
import rospy
import rospkg
import sys, os
from pt_scripts.robot import Robot
import pt_scripts.utils as utils
from pt_scripts.plotting import Plotting
from pt_scripts.create_path import CreatePath
from pt_scripts.ros_interface import ROSInterface

sys.path.insert(1, '/home/piotr/catkin_ws/src/mh/PathTrackPy')

METHOD = 'MPC2'  # LQR - [MPC2 mpc2] - MPC - [PurePursuit pure_pursuit]
r_path = '/home/piotr/catkin_ws/src/mh/PathTrackPy/scripts/controllers/mpc2/'

sys.path.insert(0, r_path)

from controller_r import Controller


class Tracking:
    def __init__(self):

        # setting
        self.by_cost_map = False   # False - True                    # ******************************* set
        self.pose_method = "Odom" # Tf  Odom                        # ******************************* set
        self.start_method = "normal" # from_global_plan - normal    # ******************************* set

        # initialize
        rospy.on_shutdown(self.shutdown)
        self.ros_interface = ROSInterface(start_method=self.start_method, by_cost_map=self.by_cost_map, pose_method=self.pose_method)
        self.initialize()
        self.ros_interface.start(self.ref_traj)
        
        # run
        self.run()

        # final 
        self.final()
    
    # ----------------------- initialize -----------------------------------

    def initialize(self):
        """
            Create reference path or trajectory.
            Create Robot object.
            Create Initial Plot.
            Create Controller (tracker).
        """
        print("initializing tracking")

        # base directory
        ros_pkg = rospkg.RosPack()
        pkg_dir = ros_pkg.get_path('path_tracking_py')
        data_dir = os.path.join(pkg_dir, 'data')

        # create reference Path/Trajectory/Waypoints
        print("Creating Path")
        ref_traj = CreatePath(data_dir)
        
        if not self.start_method=="normal":
            while not self.ros_interface.got_plan:
                print("waiting for global plan")
                self.ros_interface.rate.sleep()
            if self.ros_interface.global_plan == []:
                raise ValueError("global_plan is empty")
            print("got global plan")
            ref_traj.from_plan(self.ros_interface.global_plan)
        else:
            ref_traj.from_json(name="path1")  #from_json(name="path_rviz_4")          # path_1, path_2, from_json, generate_curved_waypoints  ************ set
        self.ref_traj = ref_traj
        goal = [ref_traj.x[-1], ref_traj.y[-1]]  # last waypoint

        # data
        name = METHOD + '_' + self.ref_traj.name
        save_pic_path = os.path.join(data_dir, 'PicsR')
        self.save_pic_path = os.path.join(save_pic_path, name)
        save_res_path = os.path.join(data_dir, 'ResultsR')
        self.save_res_path = os.path.join(save_res_path, name +'.json')

        # create robot with initial pose and velocity
        print("Creating Robot")
        r_pose = self.ros_interface.r_pose
        robot_ = Robot(x=r_pose[0], y=r_pose[1], yaw=r_pose[2], v=0, w=0)

        # speed profile
        print("Generate speeed profile")
        self.ref_traj.speed_profile(robot_)

        # plotting
        print("Creating Plotting")
        self.my_plot = Plotting(ref_traj, robot_.pose, goal)

        # controller
        print("Creating Controller")
        self.dt = 0.1
        self.controller = Controller(self.ros_interface, ref_traj, robot_, self.my_plot, self.dt)

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
        print("mean error: ", self.dist_error)

        # # plot final
        # print("final plotting ...")
        # self.my_plot.plot_final(self.save_pic_path, self.controller.rec_traj_x, self.controller.rec_traj_y)


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
        self.dist_error = round(dist_error/len(rec_traj_x), 3)

        save_path = self.save_res_path
        data = {'method': METHOD, 'path': self.ref_traj.name, 'dist_error': self.dist_error, 'len': trav_len, 't': trav_t, 'trav_w':trav_w}
        with open(save_path, "w") as outfile:
            json.dump(data, outfile, indent=2)
            outfile.write("\n")

    def shutdown(self):
        self.ros_interface.stop()     

# ------------------------------------------------------------------

if __name__=="__main__":
    rospy.init_node("tracking")
    tr = Tracking()