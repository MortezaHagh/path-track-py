import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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
        self.t_pause = 0.01
        self.arrow_width = 0.3
        self.ref_traj = ref_traj

        self.initialize()

        # reference plan and init pose
        self.pose_p, = self.ax.plot(r_pose[0], r_pose[1], 'go', label='Robot Pose')
        self.look_ahead_p, = self.ax.plot(goal[0], goal[1], 'ro', label='look-ahead point')
        plt.legend(loc="upper right")  # bbox_to_anchor=(1.04, 1)

    def initialize(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Path Tracking")
        self.ax.set_xlim(self.ref_traj.x_min-3, self.ref_traj.x_max+3)
        self.ax.set_ylim(self.ref_traj.y_min-3, self.ref_traj.y_max+3)
        self.ax.plot(self.ref_traj.x, self.ref_traj.y, 'b-o', label='Reference Path', markersize=3)

    def update_plot(self, dx, dy, r_pose, lookahead_point):
        """update plot based on last robot pose and heading

        Args:
            dx (float): x displacement in last timestep
            dy (float): y displacement in last timestep
            r_pose (pose): robot pose
            lookahead_point (pose): look ahead pose.
        """

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
        self.initialize()
        self.ax.plot(rec_traj_x, rec_traj_y, 'r', label='Robot Path')
        plt.legend(loc="upper right")
        plt.savefig(pic_dir+'.png', format="png", dpi=600)
        plt.show()

    def plot_anim(self,  pic_dir, rec_traj_x, rec_traj_y, rec_traj_yaw):
        self.initialize()
        self.lmax = len(rec_traj_x)
        self.rec_traj_x = rec_traj_x
        self.rec_traj_y = rec_traj_y
        self.rec_traj_yaw = rec_traj_yaw

        self.lines = [None, None, None]
        self.lines[0], = self.ax.plot(rec_traj_x[1], rec_traj_y[1], 'o', markerfacecolor='g',
                                      markeredgecolor='g', markersize=5, label="Robot Pose")
        self.lines[1], = self.ax.plot(rec_traj_x[0:2], rec_traj_y[0:2], 'r', label='Robot Path')

        dx, dy = math.cos(math.pi/4), math.sin(math.pi/4)
        self.lines[2] = self.ax.arrow(rec_traj_x[1], rec_traj_y[1], dx, dy, width=self.arrow_width, color="g")

        self.animate()
        plt.legend(loc="upper right")
        self.anim.save(pic_dir+'.gif', fps=200)
        # plt.show()

    def ani_init(self):
        self.lines[0].set_data([], [])
        self.lines[1].set_data([], [])
        return self.lines

    def ani_update(self, i):
        self.lines[2].remove()
        th = self.rec_traj_yaw[i]
        dx, dy, = math.cos(th), math.sin(th)
        self.lines[0].set_data(self.rec_traj_x[i], self.rec_traj_y[i])
        self.lines[1].set_data(self.rec_traj_x[0:i], self.rec_traj_y[0:i])
        self.lines[2] = self.ax.arrow(self.rec_traj_x[i], self.rec_traj_y[i], dx, dy, width=self.arrow_width, color="g")
        return self.lines

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig,  self.ani_update, frames=self.lmax, blit=False, interval=500)
