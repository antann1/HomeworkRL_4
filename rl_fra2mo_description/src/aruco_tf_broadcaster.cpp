#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath> 
#include <geometry_msgs/msg/pose_stamped.hpp>

class ArucoTfPublisher : public rclcpp::Node {
public:
    ArucoTfPublisher() : Node("aruco_tf_publisher") {
        // Inizializzazione del TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Sottoscrizione al topic di odometria
        aruco_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10,
            std::bind(&ArucoTfPublisher::arucoCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Aruco TF publisher initialized.");
    }

private:
    void arucoCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Legge la posizione e l'orientamento dal messaggio
        double aruco_x = msg->pose.position.x;
        double aruco_y = msg->pose.position.y;
        double aruco_z = msg->pose.position.z;

        // Legge l'orientamento (quaternione)
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        // Aggiungi l'offset alla posizione
        double map_x = aruco_x + map_offset_x_;
        double map_y = aruco_y + map_offset_y_;
        double map_z = aruco_z;

        // Creazione e pubblicazione della trasformazione statica
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = "aruco_marker_frame"; // frame di partenza
        transform_stamped.child_frame_id = "map"; // frame di destinazione

        transform_stamped.transform.translation.x = map_x;
        transform_stamped.transform.translation.y = map_y;
        transform_stamped.transform.translation.z = map_z;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Pubblica la trasformazione
        tf_broadcaster_->sendTransform(transform_stamped);

        // Log per debug
        RCLCPP_DEBUG(this->get_logger(),
                     "Published static transform: [frame: aruco_marker_frame -> map] "
                     "[x: %.2f, y: %.2f, z: %.2f]",
                     map_x, map_y, map_z);
    }
    

    // TF Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double map_offset_x_ = -3.0;
    double map_offset_y_ = 3.5;

    // Sottoscrizione al topic di odometria
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoTfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
