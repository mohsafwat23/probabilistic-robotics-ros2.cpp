#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <random>

using std::placeholders::_1;
float pi = 3.14159265359;
std::random_device rd; // obtain a random number from hardware
std::mt19937 gen(rd()); // seed the generator
std::uniform_int_distribution<> distr(-10, 10); // define the range



class EncoderPublisher : public rclcpp::Node
{
    std::string wheel;
    std::string l_wheel = "drivewhl_l_link";
    std::string r_wheel = "drivewhl_r_link";
    float d_enc_l = 0.0;
    float d_enc_r = 0.0;
    float ti_l = 0;
    float ti_r = 0;
    tf2::Quaternion q0_l, q0_inv_l, q_l;
    tf2::Quaternion q0_r, q0_inv_r, q_r;

    public:
        EncoderPublisher() : Node("encoder_node") //constructor
        {
        wheel_state_sub = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 5, std::bind(&EncoderPublisher::angle_callback, this, _1));
        enc_pub = this->create_publisher<geometry_msgs::msg::Point>("/encoder", 10);
        }


    private:
        void angle_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
        {
            auto message = geometry_msgs::msg::Point();
            int size = msg->transforms.size();  ///sizeof(msg->transforms[0]);

            //simulate an encoder skipping counts
            int encoder_fault = distr(gen);
            if(encoder_fault != 0)
            {
                for (int i=0; i < size; i++)
                {
                    wheel = msg->transforms[i].child_frame_id;
                    if(wheel == l_wheel)
                    {
                        float tf_l = float(msg->transforms[i].header.stamp.sec) + float(msg->transforms[i].header.stamp.nanosec)*1e-9;
                        // q.w() = msg->transforms[i].transform.rotation.w;
                        // q.x() = msg->transforms[i].transform.rotation.x;
                        // q.y() = msg->transforms[i].transform.rotation.y;
                        // q.z() = msg->transforms[i].transform.rotation.z;                    
                        //auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                        float dt_l = tf_l - ti_l;
                        // Orientation quaternion
                        tf2::Quaternion q1_l(
                                    msg->transforms[i].transform.rotation.x,
                                    msg->transforms[i].transform.rotation.y,
                                    msg->transforms[i].transform.rotation.z,
                                    msg->transforms[i].transform.rotation.w);
                        q0_inv_l = tf2::inverse(q0_l);
                        q_l = q1_l*q0_inv_l;
                        // 3x3 Rotation matrix from quaternion
                        tf2::Matrix3x3 m_l(q_l);

                        // Roll Pitch and Yaw from rotation matrix
                        double d_roll_l, d_pitch_l, d_yaw_l;
                        m_l.getRPY(d_roll_l, d_pitch_l, d_yaw_l);

                        d_enc_l = d_pitch_l*0.1/dt_l;
                        q0_l = q1_l;
                        ti_l = tf_l;

                        //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", d_enc_l);

                    }
                    wheel = msg->transforms[i].child_frame_id;
                    if (wheel == r_wheel)
                    {
                        float tf_r = float(msg->transforms[i].header.stamp.sec) + float(msg->transforms[i].header.stamp.nanosec)*1e-9;
                        float dt_r = tf_r- ti_r;
                        // Orientation quaternion
                        tf2::Quaternion q1_r(
                                    msg->transforms[i].transform.rotation.x,
                                    msg->transforms[i].transform.rotation.y,
                                    msg->transforms[i].transform.rotation.z,
                                    msg->transforms[i].transform.rotation.w);
                        q0_inv_r = tf2::inverse(q0_r);
                        q_r = q1_r*q0_inv_r;
                        // 3x3 Rotation matrix from quaternion
                        tf2::Matrix3x3 m_r(q_r);

                        // Roll Pitch and Yaw from rotation matrix
                        double d_roll_r, d_pitch_r, d_yaw_r;
                        m_r.getRPY(d_roll_r, d_pitch_r, d_yaw_r);

                        d_enc_r = d_pitch_r*0.1/dt_r;
                        q0_r = q1_r;
                        ti_r = tf_r;
                    }
                    message.x = d_enc_l;
                    message.y = d_enc_r;
                    enc_pub->publish(message);
                }

            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "I heard: '%i'", encoder_fault);
                message.x = 0.0;
                message.y = 0.0;
                enc_pub->publish(message);
            }

        }

        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr wheel_state_sub;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr enc_pub;


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EncoderPublisher>());
  rclcpp::shutdown();
  return 0;
}