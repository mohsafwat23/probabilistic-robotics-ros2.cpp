#ifndef PF_NODE_h
#define PF_NODE_h

#include "pf.hpp"
#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diff_rob_msgs/msg/encoder.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
using std::placeholders::_1;
using namespace std::chrono_literals;

class PF_Node : public rclcpp::Node
{
    public:
        PF_Node();
    
    private:
        void encoder_callback(const diff_rob_msgs::msg::Encoder::SharedPtr msg);
        void cam_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void publisher_callback();
        std::string frameid;
        PF pf;
        int nlandmarks;
        int W;
        double tf, ti;
        bool cam_recieved = false;
        bool encoder_recieved = false;
        std::vector<float> data;
        int vec_size;
        int id_aruco;
        tf2::Quaternion quat;



        //Landmarks from YAML
        std::vector<double> lmP[12];   //landmark position
        std::vector<double> lmR[12];  //landmark orientation
        //ROS stuff
        std::vector<geometry_msgs::msg::PoseStamped> poses_vec;
        rclcpp::Subscription<diff_rob_msgs::msg::Encoder>::SharedPtr enc_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr landmark_cam_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_estimate_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
        rclcpp::TimerBase::SharedPtr timer;
};

#endif