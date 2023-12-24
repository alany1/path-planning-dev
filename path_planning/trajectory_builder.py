#!/usr/bin/env python3

import time
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray, Point
from visualization_msgs.msg import Marker

from path_planning.utils import LineTrajectory
import rospkg

from ament_index_python.packages import get_package_share_directory



# TODO: change the name of trajectory file so that it saves to a name that is passed in
# makes things easier since we can't save on shutdown, just continuously save
# Also, might need to hard code the path to package instead of the share directory
class BuildTrajectory(Node):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """
    def __init__(self):
        super().__init__("build_trajectory")

        package_share_directory = get_package_share_directory("path_planning")
        self.save_path = os.path.join(package_share_directory+"/trajectories/", time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj") #%Y-%m-%d-%H-%M-%S
        self.trajectory = LineTrajectory(self, "/built_trajectory")
        self.data_points = []
        self.count = 0
        self.click_sub = self.create_subscription(PointStamped, "/clicked_point", self.clicked_pose, 10)
        self.traj_pub = self.create_publisher(PoseArray, "/trajectory/current", 10)
        self.trajectory_points = self.create_publisher(Marker, "/traj_pts", 20)
        self.trajectory.publish_viz()

        # save the built trajectory on shutdown
        self.get_logger().info("Press Ctrl+C to save the trajectory and exit.")

    def publish_trajectory(self):
        self.traj_pub.publish(self.trajectory.toPoseArray())

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)
        self.get_logger().info("Trajectory saved to: {}".format(self.save_path))

    def clicked_pose(self,msg):
        self.count += 1
        point = Point()
        point.x = msg.point.x
        point.y = msg.point.y
        self.trajectory.addPoint(point)
        self.data_points.append(point)
        self.mark_pt(self.trajectory_points, (0.0,1.0,0.0), self.data_points)
        if self.count > 2:
            self.get_logger().info("Publishing trajectory")
            self.publish_trajectory()
            self.trajectory.publish_viz()
            self.saveTrajectory()


    def mark_pt(self, subscriber, color_tup, data):
        mark_pt = Marker()
        mark_pt.header.frame_id = "/map"
        mark_pt.header.stamp = self.get_clock().now().to_msg()
        mark_pt.type  = mark_pt.SPHERE_LIST
        mark_pt.action = mark_pt.ADD
        mark_pt.scale.x = .5
        mark_pt.scale.y = .5
        mark_pt.scale.z= .5
        mark_pt.color.a =1.0
        mark_pt.color.r=color_tup[0]
        mark_pt.color.g = color_tup[1]
        mark_pt.color.b = color_tup[2]
        mark_pt.points = data
        subscriber.publish(mark_pt)


def main(args=None):
    rclpy.init(args=args)
    build_traj = BuildTrajectory()
    rclpy.spin(build_traj)
    build_traj.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
