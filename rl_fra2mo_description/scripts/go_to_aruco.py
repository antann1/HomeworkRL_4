#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import tf2_geometry_msgs
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
import yaml, os, math
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import StaticTransformBroadcaster

# Carica i waypoints da YAML
waypoints = yaml.safe_load(
    open(os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "aruco_goto.yaml"))
)

# Carica la posizione iniziale
initial_pose = yaml.safe_load(
    open(os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "initial_pose.yaml"))
)

# Aggiungi il nodo rclpy per il recupero della posa ArUco
class ArucoNavigationNode(Node):
    def __init__(self):
        super().__init__('aruco_navigation_node')
        self.navigator = BasicNavigator()
        #self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, self)
       # self.aruco_sub = self.create_subscription(PoseStamped, '/aruco_single/pose', self.aruco_callback, 10)
        self.current_marker_pose = None
        self.map_offset_x = -3.0
        self.map_offset_y = 3.5


    def start_navigation(self):
        # Inizializza il navigator
        self.navigator.waitUntilNav2Active(localizer="smoother_server")
        
        # 1. Invia il robot vicino all'ostacolo 9
        self.navigate_to_waypoints()


    def navigate_to_waypoints(self):
        all_goal_poses = list(map(self.create_pose, waypoints["waypoints"]))
        
        # Segui i waypoint
        self.navigator.followWaypoints(all_goal_poses)

        # Attendi che il robot completi la navigazione
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(all_goal_poses)}')

 
    def create_pose(self, transform):
        initial_translation = {"x": -3, "y": 3.5}
        initial_rotation = -math.pi / 2  # -90° in radianti

        map_position = transform["position"]
        map_orientation = transform["orientation"]

        translated_x = map_position["x"] - initial_translation["x"]
        translated_y = map_position["y"] - initial_translation["y"]

        odom_x = math.cos(-initial_rotation) * translated_x - math.sin(-initial_rotation) * translated_y
        odom_y = math.sin(-initial_rotation) * translated_x + math.cos(-initial_rotation) * translated_y

        def rotate_quaternion_z(q, theta):
            rot_q = {
                "x": 0,
                "y": 0,
                "z": math.sin(theta / 2),
                "w": math.cos(theta / 2)
            }
            new_q = {
                "x": rot_q["w"] * q["x"] + rot_q["x"] * q["w"] + rot_q["y"] * q["z"] - rot_q["z"] * q["y"],
                "y": rot_q["w"] * q["y"] - rot_q["x"] * q["z"] + rot_q["y"] * q["w"] + rot_q["z"] * q["x"],
                "z": rot_q["w"] * q["z"] + rot_q["x"] * q["y"] - rot_q["y"] * q["x"] + rot_q["z"] * q["w"],
                "w": rot_q["w"] * q["w"] - rot_q["x"] * q["x"] - rot_q["y"] * q["y"] - rot_q["z"] * q["z"]
            }
            return new_q

        odom_orientation = rotate_quaternion_z(map_orientation, -initial_rotation)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = odom_x
        pose.pose.position.y = odom_y
        pose.pose.position.z = map_position["z"]
        pose.pose.orientation.x = odom_orientation["x"]
        pose.pose.orientation.y = odom_orientation["y"]
        pose.pose.orientation.z = odom_orientation["z"]
        pose.pose.orientation.w = odom_orientation["w"]

        return pose
    

def main():
    rclpy.init()
    node = ArucoNavigationNode()
    node.start_navigation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()