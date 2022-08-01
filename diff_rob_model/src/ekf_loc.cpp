//get the known landmark values (rosparams)
//get the encoders and feed them into the prediction (callback1)
//get the landmarks id and position for update  (callback2)
//publish x,y,theta odom or path
#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "diff_rob_msgs/msg/landmark.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class EKFlocalization : public rclcpp::Node //inherits from Node
{
    public:
        EKFlocalization() : Node("ekf_node")    //declaring the constructors
        {
            enc_sub = this->create_subscription<geometry_msgs::msg::Point>("/encoder", 10, std::bind(&EKFlocalization::encoder_callback, this, _1));
            landmark_cam_sub = this->create_subscription<diff_rob_msgs::msg::Landmark>("/landmark_dist", 10, std::bind(&EKFlocalization::cam_callback, this, _1));
            pose_estimate_pub = this->create_publisher<geometry_msgs::msg::Pose2D>("/pose_estimate", 10);
             timer = this->create_wall_timer(500ms, std::bind(&EKFlocalization::publisher_callback, this));
        }
    private:
        void encoder_callback(const geometry_msgs::msg::Point::SharedPtr msg)
        {
            d_enc_l = msg->x;
            d_enc_r = msg->y;

        }

        void cam_callback(const diff_rob_msgs::msg::Landmark::SharedPtr msg)
        {
            id_aruco = msg->id.data;
            x_aruco = msg->x.data;
            z_aruco = msg->z.data;
            //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", d_enc_l);

        }

        void publisher_callback()
        {

        }

    //declaring class variables
    int id_aruco;
    float x_aruco;
    float z_aruco;
    float d_enc_l;
    float d_enc_r;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr enc_sub;
    rclcpp::Subscription<diff_rob_msgs::msg::Landmark>::SharedPtr landmark_cam_sub;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_estimate_pub;
    rclcpp::TimerBase::SharedPtr timer;
};








int main(int argc,char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFlocalization>()); //create a shared pointer class object
    rclcpp::shutdown();
    return 0;
}