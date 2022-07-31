#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class TrajVisPublisher: public rclcpp::Node
{
    float x;
    float y;
    float z;
    std::string frameid = "odom";
    std::vector<geometry_msgs::msg::PoseStamped> poses_vec;

    public:
    TrajVisPublisher() : Node("traj_vis_node")
    {
        real_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odom", 10, std::bind(&TrajVisPublisher::odom_callback, this, _1));
        real_path_pub = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
    }

    ~TrajVisPublisher()
    {

    }
    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {

            //x = msg->pose.pose.position.x;
            rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::PoseStamped pose_stamp;
            pose_stamp.header.frame_id = frameid;     //relative to ...
            pose_stamp.header.stamp = now;
            pose_stamp.pose = msg->pose.pose;
            poses_vec.push_back(pose_stamp);    //similar to append
            //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", x);

            nav_msgs::msg::Path path;
            path.header.stamp = now;
            path.header.frame_id = frameid;     //relative to ...
            path.poses = poses_vec;
            real_path_pub->publish(path);

        }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr real_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_path_pub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajVisPublisher>());
    rclcpp::shutdown();
    return 0;
}