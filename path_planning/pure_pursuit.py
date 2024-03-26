import numpy as np
from .utils import LineTrajectory
import math
from tf_transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("pure_pursuit")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = .5
        self.speed = 2.0
        self.wheelbase_length = .325

        self.trajectory = LineTrajectory("/followed_trajectory")

        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)

        self.pose_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)

        self.initialized_traj = False

        self.circ_pub = self.create_publisher(Marker, "/lookahead_circ", 1)
        self.pt_pub = self.create_publisher(Marker, "/intersection", 1)

    def shortest_dist(self, start, end, curr):
        """
        Find vectors between each point, use dot products of vectors to figure out which of three cases the configuration is:
        1. curr is to the left of the line segment, and therefore closest to start
            a) find distance to 
        2. curr is to the right of the line segment, and therefore closest to end
        3. curr is between start and end, take perpendicular distance to segment
        """

        x, y = curr[0], curr[1]
        x1, y1 = start[0], start[1]
        x2, y2 = end[0], end[1]

        AB = (x2 - x1, y2 - y1)
        BE = (x - x2, y - y2)
        AE = (x - x1, y - y1)
        AB_dot_BE = (AB[0] * BE[0] + AB[1] * BE[1])
        AB_dot_AE = (AB[0] * AE[0] + AB[1] * AE[1])

        if AB_dot_BE > 0:  # if dot between AB and BE points away from segment

            return math.sqrt(BE[0] ** 2 + BE[1] ** 2)

        elif AB_dot_AE < 0:  # if dot between AB and AE points away from segment

            return math.sqrt(AE[0] ** 2 + AE[1] ** 2)

        else:  # use A = 1/2 * b * h and solve for h -----> find 2A from det of point matrix, b is length of segment

            points_matrix = np.array([[x, y, 1], [x1, y1, 1], [x2, y2, 1]])

            double_A = abs(np.linalg.det(points_matrix))
            b = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            h = double_A / b

            return h

    def find_circ_intersect(self, start, end, curr, theta_curr):
        r = self.lookahead
        x1, y1 = start[0], start[1]
        x2, y2 = end[0], end[1]

        v = (x2 - x1, y2 - y1)

        a = self.dot(v, v)
        b = 2 * self.dot(v, self.sub_vector(start, curr))
        c = self.dot(start, start) + self.dot(curr, curr) - 2 * self.dot(start, curr) - r ** 2

        disc = b ** 2 - 4 * a * c
        if disc < 0:
            return None

        t1 = (-b + math.sqrt(disc)) / (2 * a)
        t2 = (-b - math.sqrt(disc)) / (2 * a)

        # if neither solution is on the line segment
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return None

        # return start + t * v if the other sol is not on the line
        if not (0 <= t1 <= 1):
            return self.add_vector(start, self.multiply_vector((t2, t2), (v)))

        if not (0 <= t2 <= 1):
            return self.add_vector(start, self.multiply_vector((t1, t1), (v)))

        # if both sol valid, return the one with greater inner product btwn **orientation of resultant btwn the point and the car** and **orientation of car**
        sol1 = self.add_vector(start, self.multiply_vector((t1, t1), (v)))
        sol2 = self.add_vector(start, self.multiply_vector((t2, t2), (v)))

        resultant1 = self.sub_vector(sol1, curr)
        resultant2 = self.sub_vector(sol2, curr)

        # unit vectors for each solution
        resultant1 = self.multiply_vector(resultant1, (
            1 / math.sqrt(self.dot(resultant1, resultant1)), 1 / math.sqrt(self.dot(resultant1, resultant1))))
        resultant2 = self.multiply_vector(resultant2, (
            1 / math.sqrt(self.dot(resultant2, resultant2)), 1 / math.sqrt(self.dot(resultant2, resultant2))))

        # unit vector for car
        car_unit = (math.cos(theta_curr), math.sin(theta_curr))

        dot1 = self.dot(resultant1, car_unit)
        dot2 = self.dot(resultant2, car_unit)

        # find max dot product
        return sol1 if dot1 == max(dot1, dot2) else sol2

    def dot(self, v1, v2):
        return v1[0] * v2[0] + v1[1] * v2[1]

    def add_vector(self, v1, v2):
        return (v1[0] + v2[0], v1[1] + v2[1])

    def sub_vector(self, v1, v2):
        return (v1[0] - v2[0], v1[1] - v2[1])

    def multiply_vector(self, v1, v2):
        return (v1[0] * v2[0], v1[1] * v2[1])

    def pose_callback(self, odometry_msg):
        
        pose = odometry_msg.pose
        
        points = self.trajectory.points

        if not self.initialized_traj or not points: return

        self.plot_circle()

        curr_position = np.array([pose.pose.position.x, pose.pose.position.y])

        distances = []

        for count in range(1, len(points)):
            p1 = points[count - 1]
            p2 = points[count]

            distances.append(self.shortest_dist(p1, p2, curr_position))

        min_indx = np.argmin(distances)

        theta_curr = euler_from_quaternion(
            [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])[2]

        good_intersect = None

        for indx in range(min_indx, len(points) - 1):
            intersect = self.find_circ_intersect(points[indx], points[indx + 1], curr_position, theta_curr)

            if intersect:
                good_intersect = intersect

        if not good_intersect:
            drive_cmd = AckermannDriveStamped()
            self.drive_pub.publish(drive_cmd)
            return

        self.plot_point(good_intersect[0], good_intersect[1])

        V = self.sub_vector(good_intersect, curr_position)

        angle = np.arctan2(V[1], V[0]) - theta_curr  # angle to intersect point

        distance = math.sqrt(V[0] ** 2 + V[1] ** 2)  # distance to intersect point

        delta = math.atan(2 * self.wheelbase_length * np.sin(angle) / distance)  # pure pursuit equation

        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = delta  # setting car angle
        drive_cmd.drive.speed = self.speed  # setting car velocity

        self.drive_pub.publish(drive_cmd)  # publishing to the car

        return

    def plot_circle(self, color=(0., 0., 1.), frame="/base_link"):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        circle = Marker()
        circle.type = Marker.CYLINDER
        circle.header.frame_id = frame

        # Set the size and color
        circle.scale.x = 2 * self.lookahead
        circle.scale.y = 2 * self.lookahead
        circle.color.a = 1.
        circle.color.r = color[0]
        circle.color.g = color[1]
        circle.color.b = color[2]

        # Publish the line
        self.circ_pub.publish(circle)

    def plot_point(self, x, y, color=(1., 0., 0.), frame="/map"):
        pt = Marker()
        pt.type = Marker.CYLINDER
        pt.header.frame_id = frame

        pt.pose.position.x, pt.pose.position.y = x, y

        # Set the size and color
        pt.scale.x = .4
        pt.scale.y = .4
        pt.color.a = 1.
        pt.color.r = color[0]
        pt.color.g = color[1]
        pt.color.b = color[2]

        # Publish the line
        self.pt_pub.publish(pt)

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.initialized_traj = True

        return


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
