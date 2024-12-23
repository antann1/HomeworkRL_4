#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

using namespace std::chrono_literals;

class ArucoToMapNode : public rclcpp::Node
{
public:
  ArucoToMapNode()
  : Node("aruco_correction_node")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("aruco_to_map", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&ArucoToMapNode::on_timer, this));
  }

private:
  void on_timer()
  {
    try {
      // Ottieni la trasformazione del marker rispetto alla camera
      geometry_msgs::msg::TransformStamped aruco_to_camera_msg =
        tf_buffer_->lookupTransform("camera_link", "aruco_marker_frame", tf2::TimePointZero);

      tf2::Transform tf_aruco_to_camera;
      tf2::fromMsg(aruco_to_camera_msg.transform, tf_aruco_to_camera);

      // Correzione: Rotazione di -90° sull'asse Y
      tf2::Transform correction_transform;
      correction_transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

      tf2::Quaternion correction_rotation_y, additional_rotation_z;
      correction_rotation_y.setRPY(0.0, -M_PI / 2.0, 0.0);  // Rotazione -90° su Y
      additional_rotation_z.setRPY(M_PI, 0.0, 0.0);         // Rotazione aggiuntiva di 180° su Z

      correction_transform.setRotation(correction_rotation_y * additional_rotation_z);

      // Applica la correzione al frame aruco_marker_frame
      tf2::Transform corrected_aruco_to_camera = correction_transform * tf_aruco_to_camera;

      // Ottieni la trasformazione da camera_link a map
      geometry_msgs::msg::TransformStamped camera_to_map_msg =
        tf_buffer_->lookupTransform("map", "camera_link", tf2::TimePointZero);

      tf2::Transform tf_camera_to_map;
      tf2::fromMsg(camera_to_map_msg.transform, tf_camera_to_map);

      // Combina con la trasformazione corretta
      tf2::Transform corrected_aruco_to_map = tf_camera_to_map * corrected_aruco_to_camera;

      // Trasformazione da map a world (posizione e orientamento di map rispetto a world)
      tf2::Transform map_to_world;
      map_to_world.setOrigin(tf2::Vector3(-3.0, 3.5, 0.0));  // Traslazione da map a world
      tf2::Quaternion q_map_to_world;
      q_map_to_world.setRPY(0.0, 0.0, -M_PI / 2.0);  // Rotazione di -90° su Z (yaw)
      map_to_world.setRotation(q_map_to_world);

      // Combina la trasformazione da map a world con la trasformazione di aruco
      tf2::Transform aruco_to_world = map_to_world * corrected_aruco_to_map;

      // Pubblica il risultato
      geometry_msgs::msg::TransformStamped result;
      result.header.stamp = this->get_clock()->now();
      result.header.frame_id = "world";  // World come frame di riferimento
      result.child_frame_id = "aruco_marker_frame";
      result.transform = tf2::toMsg(aruco_to_world);

      transform_publisher_->publish(result);

      // Calcola gli angoli roll, pitch e yaw per il debug
      double roll, pitch, yaw;
      tf2::Matrix3x3(aruco_to_world.getRotation()).getRPY(roll, pitch, yaw);

      // Stampa i risultati
      RCLCPP_INFO(this->get_logger(), "Aruco Marker in World: x=%.2f, y=%.2f, z=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
        aruco_to_world.getOrigin().x(),
        aruco_to_world.getOrigin().y(),
        aruco_to_world.getOrigin().z(),
        roll, pitch, yaw);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  // Variabili membro
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoToMapNode>());
  rclcpp::shutdown();
  return 0;
}
