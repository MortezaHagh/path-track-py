#!/usr/bin/env python

import os
import json
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion, PoseWithCovariance

class CreatePathRviz:
    """The CreatePathRviz class is responsible for creating and publishing a path in RViz based on received 
        poses, and saving the path as a JSON file.
    """
    
    def __init__(self):

        # Constants
        self.d_min = 0.3  # Minimum distance threshold (to add midpoints)

        # Global variables
        self.xyt = []
        self.poses = []  # List to store received poses
        self.new_poses = []

        # 
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)
        path = Path()
        path_online = Path()
        path.header.frame_id = 'map'
        path_online.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        path_online.header.stamp = rospy.Time.now()
        self.path = path
        self.path_online = path_online
        self.path_pub = rospy.Publisher('new_path_rviz', Path, queue_size=2)
        self.path_online_pub = rospy.Publisher('online_path_rviz', Path, queue_size=2)

    def pose_callback(self, msg):
        """Callback function for received poses

        Args:
            msg (PoseWithCovarianceStamped): PoseWithCovarianceStamped
        """

        self.poses.append(msg.pose)
        self.path_online.poses.append(self.create_pose(msg.pose.pose))
        if len(self.path_online.poses) > 1:
            self.path_online_pub.publish(self.path_online)

    def create_path(self):
        """create path from received poses list
        """
        if len(self.poses) > 1:
            # Calculate the average orientation
            for i in range(len(self.poses) - 1):
                pose = self.poses[i].pose
                next_pose = self.poses[i + 1].pose
                self.new_poses.append(self.create_pose(pose))
                theta = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                self.xyt.append({"x": pose.position.x, "y": pose.position.y, "theta": theta[-1]})

                # Set the orientation of the middle poses as the average quaternion
                dx = next_pose.position.x - pose.position.x
                dy = next_pose.position.y - pose.position.y
                distance = (dx ** 2 + dy ** 2) ** 0.5
                
                theta = np.arctan2(dy, dx)
                average_quaternion = quaternion_from_euler(0, 0, theta)
                q = Quaternion(average_quaternion[0], average_quaternion[1], average_quaternion[2], average_quaternion[3])

                if distance > self.d_min:
                    num_middle_poses = int(distance / self.d_min)
                    for j in range(num_middle_poses):
                        new_pose = PoseWithCovariance()
                        x = pose.position.x + dx * (j + 1) / (num_middle_poses + 1)
                        y = pose.position.y + dy * (j + 1) / (num_middle_poses + 1)
                        new_pose.pose.position.x = x
                        new_pose.pose.position.y = y
                        new_pose.pose.position.z = 0
                        new_pose.pose.orientation.x = q.x
                        new_pose.pose.orientation.y = q.y
                        new_pose.pose.orientation.z = q.z
                        new_pose.pose.orientation.w = q.w
                        self.new_poses.append(self.create_pose(new_pose.pose))
                        self.xyt.append({"x": new_pose.pose.position.x, "y": new_pose.pose.position.y, "theta": theta})


        pose = self.poses[-1].pose
        self.new_poses.append(self.create_pose(pose))
        theta = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self.xyt.append({"x": pose.position.x, "y": pose.position.y, "theta": theta[-1]})

    def create_pose(self, pose):
        """Create PoseStamped from PoseWithCovariance
        """
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time.now()
        p.pose = pose
        return p

    def publish_path(self):
        """publish created path
        """
        self.path.poses = self.new_poses
        for i in range(10):
            self.path_pub.publish(self.path)
            self.rate.sleep()

    def save_path(self):
        """save path as json file
        """
        name = 'path_rviz_4' + '.json'
        ros_pkg = rospkg.RosPack()
        pkg_dir = ros_pkg.get_path("path_tracking")
        paths_dir = os.path.join(pkg_dir, "data/Paths")
        file_dir = os.path.join(paths_dir, name)
        with open(file_dir, 'w') as f:
            json.dump(self.xyt, f)

    def shutdown_hook(self):
        """shutdown_hook
        """
        self.create_path()
        self.publish_path()
        self.save_path()
            


if __name__ == '__main__':
    rospy.init_node('pose_subscriber_node')
    cp = CreatePathRviz()
    rospy.spin()
    
