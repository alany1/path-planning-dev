import rclpy
from rclpy.node import Node

assert rclpy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory
import heapq
from scipy import ndimage
import tf_transformations as tfm
import time
class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("path_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.set = False
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

        self.map = None
        self.start_position = 0

        self.orientation = None
        self.position = None

        self.get_logger().info(f"Subscribed to {self.map_topic} and {self.odom_topic} and {self.initial_pose_topic}")

    def convertMapToOG(self, point):
        inv = np.matmul(np.linalg.inv(self.rotation_matrix), np.array([point[0], point[1], 0, 1]))
        x, y = inv[1] / (self.resolution * 1.0), inv[0] / (self.resolution * 1.0)

        return x, y

    def convertOGToMap(self, point):
        y, x = point[0] * self.resolution, point[1] * self.resolution
        new_without_trans = np.matmul(self.rotation_matrix, np.array([x, y, 0, 1]))

        return new_without_trans[0], new_without_trans[1]

    def preprocess_map(self, expansion_size=0):
        # expand obstacles by expansion_size
        # map = np.array(self.map)
        map = np.array(self.map)
        map[map == 100] = 1
        map[map == 0] = 0
        map[map == -1] = 0
        map = ndimage.binary_dilation(map, iterations=expansion_size)
        map = map.astype(int)
        map = map * 100
        self.map = map

    def map_cb(self, msg):
        # split up the map into a 2D array

        self.get_logger().info("map received")
        self.get_logger().info("resolution " + str(msg.info.resolution))
        self.get_logger().info("dimensions " + str(msg.info.width) + " " + str(msg.info.height))

        ori = msg.info.origin.orientation
        pos = msg.info.origin.position
        self.rotation_matrix = tfm.quaternion_matrix([ori.x, ori.y, ori.z, ori.w])
        self.rotation_matrix[0:2, 3] = [pos.x, pos.y]
        self.resolution = msg.info.resolution

        self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.preprocess_map(expansion_size=13)

    @staticmethod
    def euclidean_heuristic(distance, path, end_point):
        return distance + np.linalg.norm(np.array(end_point) - np.array(path[-1])) + len(path)

    # def odom_cb(self, msg):
    #     old_start_position = self.start_position
    #     if self.map is None:
    #         return
    #     # get current car pose
    #     if self.devel:
    #         self.start = msg.pose.pose.position, msg.pose.pose.orientation
    #         self.start_position = self.convertMapToOG((self.start[0].x, self.start[0].y)) # int(self.start[0].x), int(self.start[0].y)
    #         self.start_position = (int(self.start_position[0]), int(self.start_position[1]))
    #         self.start_orientation = self.start[1].z, self.start[1].w
    #         # self.get_logger().info("Received current position: {}".format(msg.pose.pose.position))
    #         # self.get_logger().info("Converted current position: {}".format(self.start_position))
    #         # self.start_pose = (self.start_position, self.start_orientation)
    #     else:
    #         raise NotImplementedError("TODO: need to use localization for non-development setting.")

    #     if self.set and self.start_position != old_start_position:
    #         # self.get_logger().info("Planning Path")
    #         self.plan_path(self.start_position, self.goal_position, self.map, PathPlan.euclidean_heuristic)

    def pose_cb(self, pose):
        if self.map is None:
            self.get_logger().info('not set')
            return

        self.get_logger().info('yo')

        old_start_position = self.start_position

        self.start = pose.pose.pose.position, pose.pose.pose.orientation
        self.start_position = self.convertMapToOG((self.start[0].x, self.start[0].y))
        self.start_position = (int(self.start_position[0]), int(self.start_position[1]))

        self.start_orientation = self.start[1].z, self.start[1].w

        if self.set and self.start_position != old_start_position:
            self.get_logger().info("Planning Path")
            self.plan_path(self.start_position, self.goal_position, self.map, PathPlan.euclidean_heuristic)

    def goal_cb(self, msg):
        self.get_logger().info("Received goal position: {}".format(msg.pose.position))
        self.goal = msg.pose.position.x, msg.pose.position.y
        self.goal_position = self.convertMapToOG(self.goal)  # int(self.start[0].x), int(self.start[0].y)
        self.goal_position = (int(self.goal_position[0]), int(self.goal_position[1]))
        self.goal_orientation = msg.pose.orientation.z, msg.pose.orientation.w
        # self.plan_path(self.start_position, self.goal_position, self.map)
        self.get_logger().info("Converted goal position: {}".format(self.goal_position))
        self.set = True

    def neighbors(self, point, diags=False):
        x, y = point
        tuples = []
        if not y - 1 < 0:
            tuples.append((x, y - 1, 1))
        if not y + 1 >= self.map.shape[1]:
            tuples.append((x, y + 1, 1))
        if not x - 1 < 0:
            tuples.append((x - 1, y, 1))
        if not x + 1 >= self.map.shape[0]:
            tuples.append((x + 1, y, 1))

        if diags and not y - 1 < 0 and not x - 1 < 0:
            tuples.append((x - 1, y - 1, np.sqrt(2)))
        if diags and not y - 1 < 0 and not x + 1 >= self.map.shape[0]:
            tuples.append((x + 1, y - 1, np.sqrt(2)))
        if diags and not y + 1 >= self.map.shape[1] and not x - 1 < 0:
            tuples.append((x - 1, y + 1, np.sqrt(2)))
        if diags and not y + 1 >= self.map.shape[1] and not x + 1 >= self.map.shape[0]:
            tuples.append((x + 1, y + 1, np.sqrt(2)))
        return tuples

    def plan_path(self, start_point, end_point, map, heuristic):
        # Should be parent, cost up to that point, path
        shortest_paths = dict()
        visited = set()

        if map[start_point] != 0:
            print('Start point failure')
        if map[end_point] != 0:
            print('End point failure')

        start_time = time.perf_counter()

        front = [(np.linalg.norm(np.array(end_point) - np.array(start_point)), 0, start_point, None)]

        while front:
            
            if time.perf_counter() - start_time > 10:
                self.get_logger().info("Took too long")
                break
            
            next = heapq.heappop(front)
            fn, gn, pos, previous = next

            if pos in visited:
                continue
            visited.add(pos)
            if previous is None:
                shortest_paths[pos] = (previous, gn, [pos])
            else:
                shortest_paths[pos] = (previous, gn, shortest_paths[previous][2] + [pos])
            if pos == end_point:
                break
            for neighbor in self.neighbors(pos, diags=True):
                if map[neighbor[0:2]] == 0 and neighbor[0:2] not in visited:
                    heapq.heappush(front, (
                        gn + neighbor[2] + np.linalg.norm(np.array(end_point) - np.array(neighbor[0:2])),
                        gn + neighbor[2],
                        neighbor[0:2], pos))

        # Convert to trajectory
        self.trajectory = LineTrajectory(self, "/planned_trajectory")
        # print(shortest_paths[end_point][2])
        print(self.compress_path(shortest_paths[end_point][2]))
        for point in self.compress_path(shortest_paths[end_point][2]):
            point = self.convertOGToMap(point)
            print("point: " + str(point))
            self.trajectory.addPoint(point)

        planning_time = (time.perf_counter() - start_time)
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()
        # self.get_logger().info("Published trajectory")
        self.get_logger().info("The path length is " + str(self.trajectory.distances[-1]) + " and consists of " + str(
            len(self.trajectory.points)) + " points.")
        self.get_logger().info("It took " + str(planning_time) + "seconds.")

    def reachable(self, start, end):
        distx = end[0] - start[0]
        disty = end[1] - start[1]
        dist = int(np.linalg.norm(np.array(end) - np.array(start)))
        for i in range(dist):
            if self.map[int(start[0] + distx / (float(dist)) * i), int(start[1] + disty / (float(dist)) * i)] != 0:
                return False
        return True

    def compress_path(self, path):
        i = 0
        while i < len(path):
            for j in range(len(path) - 1, i, -1):
                if self.reachable(path[i], path[j]):
                    path = path[:i + 1] + path[j:]
                    break
            i += 1
        return path


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
