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
    open(os.path.join(get_package_share_directory('rl_fra2mo_description'), "config", "aruco_goal.yaml"))
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

        # Inizializza il broadcaster TF statico
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Sottoscrizione al topic della posa dell'ArUco
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.aruco_pose_callback,
            10
        )

        self.get_logger().info("Aruco Static TF publisher initialized.")

    def start_navigation(self):
        # Inizializza il navigator
        self.navigator.waitUntilNav2Active(localizer="smoother_server")
        
        # 1. Invia il robot vicino all'ostacolo 9
        self.navigate_to_waypoints()

        # 2. Cerca il marker ArUco e ottieni la sua posizione
        self.wait_for_marker_detection()

        # 3. Fai tornare il robot alla posizione iniziale
        self.return_to_initial_position()

    def navigate_to_waypoints(self):
        all_goal_poses = list(map(self.create_pose, waypoints["waypoints"]))
        reordered_goal_indices = [0, 1, 2]
        goal_poses = [all_goal_poses[i] for i in reordered_goal_indices]
        
        # Segui i waypoint
        self.navigator.followWaypoints(goal_poses)

        # Attendi che il robot completi la navigazione
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}')

    def wait_for_marker_detection(self):
        # Aspetta fino a che il marker viene rilevato
        while not self.current_marker_pose:
            rclpy.spin_once(self)

        # Log della posa del marker ArUco nel frame "map"
        self.get_logger().info(f"ArUco marker position in map frame: "
                            #    f"Position -> [x: {self.current_marker_pose.pose.position.x}, "
                            #    f"y: {self.current_marker_pose.pose.position.y}, z: {self.current_marker_pose.pose.position.z}], "
                            #    f"Orientation -> [x: {self.current_marker_pose.pose.orientation.x}, "
                            #    f"y: {self.current_marker_pose.pose.orientation.y}, "
                            #    f"z: {self.current_marker_pose.pose.orientation.z}, "
                            #    f"w: {self.current_marker_pose.pose.orientation.w}]")
                             f"Position -> [x: {self.current_marker_pose.transform.translation.x}, "
                               f"y: {self.current_marker_pose.transform.translation.y}, z: {self.current_marker_pose.transform.translation.z}], "
                               f"Orientation -> [x: {self.current_marker_pose.transform.rotation.x}, "
                               f"y: {self.current_marker_pose.transform.rotation.y}, "
                               f"z: {self.current_marker_pose.transform.rotation.z}, "
                               f"w: {self.current_marker_pose.transform.rotation.w}]")

    def aruco_pose_callback(self, msg):
        # try:
        #     # Trasformiamo la posizione e orientamento del marker ArUco nel frame "map"
        #     transformed_pose = self.tf_buffer.transform(msg, "map", timeout=Duration(seconds=1.0))
            
        #     # Memorizza la posa trasformata

        # except TransformException as e:
        #     self.get_logger().error(f"Failed to transform ArUco marker pose: {e}")
         # Legge la posizione e l'orientamento del marker ArUco
        aruco_x = msg.pose.position.x
        aruco_y = msg.pose.position.y
        aruco_z = msg.pose.position.z

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # Applica l'offset iniziale al frame map
        map_x = aruco_x + self.map_offset_x
        map_y = aruco_y + self.map_offset_y
        map_z = aruco_z

        # Creazione e pubblicazione della trasformazione statica
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'aruco_marker_frame'
        transform_stamped.child_frame_id = 'map'

        transform_stamped.transform.translation.x = map_x
        transform_stamped.transform.translation.y = map_y
        transform_stamped.transform.translation.z = map_z
        transform_stamped.transform.rotation.x = qx
        transform_stamped.transform.rotation.y = qy
        transform_stamped.transform.rotation.z = qz
        transform_stamped.transform.rotation.w = qw

        # Pubblica la trasformazione statica
        self.tf_broadcaster.sendTransform(transform_stamped)
        self.current_marker_pose = transform_stamped


        # Log per debug
        self.get_logger().debug(
            f"Published static transform: [frame: aruco_marker_frame -> map] "
            f"[x: {map_x:.2f}, y: {map_y:.2f}, z: {map_z:.2f}]"
        )


    def return_to_initial_position(self):
                
        # Recupera la posa iniziale dal file YAML

        initial_position = initial_pose.get("initial_pose", None)
        if not initial_position:

            self.get_logger().error("Initial pose not defined in initial_pose.yaml.")
            return# Crea una singola posa di destinazione

        goal_pose = self.create_pose(initial_position)
        
            # Naviga verso la posa iniziale

        self.navigator.goToPose(goal_pose)
        
            # Attendi il completamento della navigazione
        while not self.navigator.isTaskComplete():

            feedback = self.navigator.getFeedback()
            if feedback:
                print(f"Returning to initial position: Distance remaining: {feedback.distance_remaining:.2f} meters.")
            
                # Verifica il risultato della navigazione

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:

                self.get_logger().info("Successfully returned to the initial position.")
        else:

                self.get_logger().error("Failed to return to the initial position.")
        


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