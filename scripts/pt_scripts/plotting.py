import matplotlib.pyplot as plt

class Plotting:
    """The Plotting class is used to create and update a plot for path tracking, and can also save the final plot to a file.
    """
    def __init__(self, ref_traj, r_pose, goal):
        """_summary_

        Args:
            ref_traj (CreatePath): _description_
            r_pose (pose): robot's initial pose [x, y]
            goal (pose): last waypoint [x, y]
        """
        self.t_pause = 0.1
        self.arrow_width = 0.3
        self.ref_traj = ref_traj

        fig, self.ax = plt.subplots()
        self.ax.set_title("Path Tracking")
        self.ax.set_xlim(self.ref_traj.x_min-3, self.ref_traj.x_max+3)
        self.ax.set_ylim(self.ref_traj.y_min-3, self.ref_traj.y_max+3)
        self.ax.plot(self.ref_traj.x, self.ref_traj.y, 'b-o', label='waypoints', markersize=3)
        self.pose_p, = self.ax.plot(r_pose[0], r_pose[1], 'go', label='Pose')
        self.look_ahead_p, = self.ax.plot(goal[0], goal[1], 'ro', label='look-ahead point')
        plt.legend()

    def update_plot(self, dx, dy, r_pose, lookahead_point):
        """update plot based on last robot pose and heading

        Args:
            dx (float): x displacement in last timestep
            dy (float): y displacement in last timestep
            r_pose (pose): robot pose
            lookahead_point (pose): look ahead pose.
        """
        
        # return
        self.pose_p.set_data(r_pose[0], r_pose[1])
        self.look_ahead_p.set_data(lookahead_point[0], lookahead_point[1])
        pose_arrow = self.ax.arrow(r_pose[0], r_pose[1], dx, dy, width=self.arrow_width, color="g")
        plt.pause(self.t_pause)
        pose_arrow.remove()

    def plot_final(self, pic_dir, rec_traj_x, rec_traj_y):
        """plot traversed and reference trajectory

        Args:
            path (str): pictures directory
            rec_traj_x (list[float]): x coords of the traversed trajectory
            rec_traj_y (list[float]): y coords of the traversed trajectory
        """
        self.ax.plot(self.ref_traj.x, self.ref_traj.y, 'b--', label='waypoints')
        self.ax.plot(rec_traj_x, rec_traj_y, 'r', label='trajectory')
        plt.legend()
        plt.savefig(pic_dir+'.png', format="png", dpi=600)
        plt.show()