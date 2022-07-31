#include <memory>
#include <functional>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

const int SIZE = 12;

class MarkerPublisher: public rclcpp::Node
{
    public:
    MarkerPublisher() : Node("marker_node")
    {

        // //Declare Parameters
        for(int i=0; i<12; i++)
        {
            declare_parameter("landmark" + std::to_string(i) + ".xyz", std::vector<double>{0.0,0.0,0.0});
            declare_parameter("landmark" + std::to_string(i) + ".rpy", std::vector<double>{0.0,0.0,0.0});
            lmP[i] = get_parameter("landmark" + std::to_string(i) + ".xyz").as_double_array();
            lmR[i] = get_parameter("landmark" + std::to_string(i) + ".rpy").as_double_array();
            //RCLCPP_INFO(get_logger(), "Double vector parameter [0]: %f", static_cast<double>(lm[i][0]));
        }

        //Create marker publisher
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/landmark_markers", 10);
        timer = this->create_wall_timer(
        100ms, std::bind(&MarkerPublisher::timer_callback, this));

    }
    private:
        void timer_callback()
        {

            for(int i=0; i<SIZE; ++i)
            {
                //convert euler to quat
                quat.setRPY(lmR[i][0], lmR[i][1], lmR[i][2]);
                quat = quat.normalize();

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "marker_loc";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::CUBE;//::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.1778;
                marker.scale.y = 0.1778;
                marker.scale.z = 0.05;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.pose.position.x = lmP[i][0];
                marker.pose.position.y = lmP[i][1];
                marker.pose.position.z = lmP[i][2];
                marker.pose.orientation.w = quat.w();
                marker.pose.orientation.x = quat.x();
                marker.pose.orientation.y = quat.y();
                marker.pose.orientation.z = quat.z();

                marker_pub->publish(marker);

              //  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", lm[i][0]);
            }
        }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    std::vector<double> lmP[SIZE];
    std::vector<double> lmR[SIZE];
    tf2::Quaternion quat;


};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerPublisher>());
    rclcpp::shutdown();
    return 0;
}