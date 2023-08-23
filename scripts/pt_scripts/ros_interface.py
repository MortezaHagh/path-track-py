#! /usr/bin/env python

import tf
import rospy
import actionlib
import numpy as np
from nav_msgs.msg import Path, Odometry
from tf.listener import TransformListener
from visualization_msgs.msg import Marker
from occupancy_grid_python import OccupancyGridManager
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 

class ROSInterface:
    def __init__(self, start_method="normal", by_cost_map="false", pose_method="Odom"):
        
        self.by_cost_map = by_cost_map
        self.pose_method = pose_method
        self.start_method = start_method

        # initialize
        self.ros_initialize()
        
        
    def ros_initialize(self):
        """
        The `ros_initialize` function initializes the necessary ROS components and publishers.
        """

        rospy.loginfo("initialize ros ... ")
        
        # data
        self.rate = rospy.Rate(10)
        self.r_x = None
        self.r_y = None
        self.r_h = None
        self.global_plan = []

        self.got_plan = False
        self.pose_method = self.pose_method # Odom, Tf

        # tf
        self.tf_listener = TransformListener()
        self.get_tf()
        while self.r_x is None:
            self.get_tf()
            self.rate.sleep()
        
        # subscribers
        self.odom_name = 'odom' # odom /ur_driver_cleaner/odom
        self.check_odom()
        rospy.Subscriber(self.odom_name, Odometry, self.odom_callback)
        rospy.Subscriber("/move_base/CleaningGlobalPlanner/plan", Path, self.global_plan_callback)

        # publishers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.traj_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher('/ref_path', Path, queue_size=10)
        self.target_marker_pub = rospy.Publisher('/target', Marker, queue_size=2)
        self.pred_path_pub = rospy.Publisher('/pred_path', Path, queue_size=10)

        # clients
        if self.by_cost_map:
            rospy.loginfo("waiting for move_base server")
            self.mb_clinet = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            self.mb_clinet.wait_for_server()
            self.mb_goal_state = -1

        # cost map
        self.has_cost_map = False
        if self.by_cost_map:
            self.has_cost_map = True
            self.ogm = OccupancyGridManager('/move_base/global_costmap/costmap', subscribe_to_updates=True)  # default False

        # trajectory
        self.traj = Path()
        self.traj.header.stamp = rospy.Time.now()
        self.traj.header.frame_id = "map"
        self.traj.header.seq = 0

        # marker
        self.marker = Marker()
        self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = "map"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.3
        self.marker.scale.z = 0.3
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0


    def start(self, ref_traj):
        """
        This function sets the reference trajectory, publishes waypoints, and logs the start time.
        
        :param ref_traj: ref_traj is a reference trajectory that the robot will follow.
        """

        self.ref_traj = ref_traj

        # publish path
        self.publish_waypoints()

        # start time
        self.time_start = rospy.get_time()
        rospy.loginfo("start time: " + str(self.time_start))


    def update(self,cmd_v, cmd_w, lookahead_point):
        # publish velocity
        self.publish_vel(cmd_v, cmd_w)

        # update trajectory
        self.record_trajectory()

        # updayte markers
        self.marker.pose.position.x = lookahead_point[0]
        self.marker.pose.position.y = lookahead_point[1]
        self.target_marker_pub.publish(self.marker)

        # res
        rospy.loginfo("cmd_v: " + str(round(cmd_v, 2)) + " cmd_w: " + str(round(cmd_w, 2)))
        rospy.loginfo(" -------------------------------------- ")

        self.rate.sleep()

    # -------------------------- Odom - Tf -------------------
    
    def check_odom(self):
        self.topic_msg = None
        rospy.loginfo("checking odom ...")
        while (not rospy.is_shutdown()) and self.topic_msg is None:
            try:
                self.topic_msg = rospy.wait_for_message(self.odom_name, Odometry, timeout=3.0)
                rospy.loginfo("odom is ready!")
            except:
                rospy.loginfo("odom is not ready yet, retrying ...")

        position = self.topic_msg.pose.pose.position
        quaternion = self.topic_msg.pose.pose.orientation
        orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        self.r_x = position.x
        self.r_y = position.y
        self.r_h = orientation[2]
        self.r_pose = [self.r_x, self.r_y, self.r_h]
        self.r_vel = 0

    def odom_callback(self, odom):
        if self.pose_method == 'Odom':
            position = odom.pose.pose.position
            quaternion = odom.pose.pose.orientation
            orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            self.r_x = position.x
            self.r_y = position.y
            self.r_h = orientation[2]
            self.r_pose = [self.r_x, self.r_y, self.r_h]
        else:
            self.get_tf()
        vx = odom.twist.twist.linear.x
        vy = odom.twist.twist.linear.y
        self.r_vel = np.sqrt(vx**2 + vy**2)

    def get_tf(self):
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf exception")
                self.rate.sleep()

        self.r_x = trans[0]
        self.r_y = trans[1]
        eu = euler_from_quaternion(rot)
        self.r_h = eu[2]
        self.r_pose = [self.r_x, self.r_y, self.r_h]

    # ------------------------------------------------------------------

    def global_plan_callback(self, plan):
        xx = [p.pose.position.x for p in plan.poses]
        yy = [p.pose.position.y for p in plan.poses]
        hh = []
        for p in plan.poses:
            quaternion = p.pose.orientation
            orientation = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            hh.append(orientation[2])
        self.got_plan = True
        self.global_plan = [xx, yy, hh]

    
    def send_goal(self, x,y,yaw):
        self.mb_goal_state = -1
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.mb_clinet.send_goal(goal, done_cb=self.mb_cone_cb)

    def mb_cone_cb(self, state, result):
        if state==3: # success
            self.mb_goal_state = 1
        else: # failure
            self.mb_goal_state = 0

    def pred_pub(self, pred_poses):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        path_msg.header.seq = 0
        for pp in pred_poses:
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "map"
            p.pose.position.x = pp[0]
            p.pose.position.y = pp[1]
            p.pose.position.z = 0
            quat = quaternion_from_euler(0, 0, pp[2])
            p.pose.orientation = Quaternion(*quat)
            path_msg.poses.append(p)
        self.pred_path_pub.publish(path_msg)

    # --------------------------------------------------------------------------

    def publish_waypoints(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        path_msg.header.seq = 0
        for i in range(self.ref_traj.count):
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "map"
            p.pose.position.x = self.ref_traj.x[i]
            p.pose.position.y = self.ref_traj.y[i]
            p.pose.position.z = 0
            quat = quaternion_from_euler(0, 0, self.ref_traj.yaw[i])
            p.pose.orientation = Quaternion(*quat)
            path_msg.poses.append(p)
        self.ref_path_pub.publish(path_msg)
        self.publish_once(self.ref_path_pub, path_msg)

    def record_trajectory(self):
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.position.x = self.r_pose[0]
        p.pose.position.y = self.r_pose[1]
        p.pose.position.z = 0;        

        quat = quaternion_from_euler(0, 0, self.r_pose[2])
        p.pose.orientation = Quaternion(*quat)
        self.traj.poses.append(p)
        self.traj_pub.publish(self.traj)

    def publish_vel(self, cmd_v, cmd_w):
        cmd_vel = Twist()
        cmd_vel.linear.x = cmd_v
        cmd_vel.angular.z = cmd_w
        self.vel_pub.publish(cmd_vel)

    def publish_once(self, publisher, msg):
        while not rospy.is_shutdown(): 
            if publisher.get_num_connections()>0:
                publisher.publish(msg)
                break
            else:
                rospy.sleep(0.1)

    # ---------------------------------------------------------------

    def stop(self, msg=""):
        rospy.loginfo("Stopping " + msg)
        self.publish_once(self.vel_pub, Twist())

    def shutdown_hook(self):
        self.stop()
        # finish time
        self.time_finish = rospy.get_time()
        rospy.loginfo("finish time: " + str(self.time_finish))
        rospy.loginfo("total time: " + str(self.time_finish - self.time_start))

        self.stop()
        rospy.loginfo("Shutting down")