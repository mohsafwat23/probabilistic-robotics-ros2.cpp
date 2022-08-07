//get the known landmark values (rosparams)
//get the encoders and feed them into the prediction (callback1)
//get the landmarks id and position for update  (callback2)
//publish x,y,theta odom or path
#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <eigen3/Eigen/Dense>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "diff_rob_msgs/msg/encoder.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <math.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
const double pi = 3.14159265359;


const int SIZE = 12;

class EKFlocalization : public rclcpp::Node //inherits from Node
{
    public:
        EKFlocalization() : Node("ekf_node")    //declaring the constructors
        {
            enc_sub = this->create_subscription<diff_rob_msgs::msg::Encoder>("/encoder", 10, std::bind(&EKFlocalization::encoder_callback, this, _1));
        
            landmark_cam_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/landmark_dist", 10, std::bind(&EKFlocalization::cam_callback, this, _1));
            pose_estimate_pub = this->create_publisher<nav_msgs::msg::Path>("/pose_estimate", 10);
            marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/robot_pose_estimate", 10);
            timer = this->create_wall_timer(50ms, std::bind(&EKFlocalization::publisher_callback, this));

            //Declare Parameters
            for(int i=0; i<12; i++)
            {
                declare_parameter("landmark" + std::to_string(i) + ".xyz", std::vector<double>{0.0,0.0,0.0});
                declare_parameter("landmark" + std::to_string(i) + ".rpy", std::vector<double>{0.0,0.0,0.0});
                lmP[i] = get_parameter("landmark" + std::to_string(i) + ".xyz").as_double_array();
                lmR[i] = get_parameter("landmark" + std::to_string(i) + ".rpy").as_double_array();
            }

        }

        void predict() //prior
        {
            if (enconders_recieved && !std::isnan(u_enc.sum())){
                //prior motion model
                //x position
                state(0) += cos(state(2))*dt*((u_enc(0)+u_enc(1))/2.0);
                //y position
                state(1) += sin(state(2))*dt*((u_enc(0)+u_enc(1))/2.0);
                //theta
                state(2) += dt*((u_enc(1)-u_enc(0))/Width);
                //state(2) += atan2(sin(dt*((u_enc(1)-u_enc(0))/Width)), cos(dt*((u_enc(1)-u_enc(0))/Width))) ;

                //this handles angle wrapping
                state(2) = atan2(sin(state(2)), cos(state(2)));

                //motion model Jacobian
                J_fx << 1.0, 0.0, -sin(state(2))*dt*((u_enc(0)+u_enc(1))/2.0),
                        0.0, 1.0,  cos(state(2))*dt*((u_enc(0)+u_enc(1))/2.0),
                        0.0, 0.0, 1.0;

                //control model Jacobian 
                J_fu << cos(state(2))*dt/2.0, cos(state(2))*dt/2.0,
                        sin(state(2))*dt/2.0, sin(state(2))*dt/2.0,
                        -dt/Width                ,                  dt/Width;

                //prior covariance 
                sigma = J_fx*sigma*J_fx.transpose() + J_fu*Q*J_fu.transpose();
                
                // RCLCPP_INFO(this->get_logger(), "sigma2: '%f'", sigma(1,1));
                // RCLCPP_INFO(this->get_logger(), "sigma3: '%f'", sigma(2,2));
            }
            else
            {
                u_enc(0) = 0.0;
                u_enc(1) = 0.0;

            }
            enconders_recieved = false;
            dt = tf - ti;
            if(dt > 1)
            {
                dt = 0.0;
            }
            ti = tf;

        }

        void update()   //posterior
        {
            //check if camera data is coming
            if(cam_recieved)
            {
                vec_size = data.size();
            }else{
                vec_size = 0;
            }

            //check if the vector is valid and contains data
            if(vec_size == nlandmarks*W && vec_size > 0){

                //Eigen::Map<Eigen::MatrixXf> Landmarks(data.data(), nlandmarks, W);
                Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Landmarks(data.data(), nlandmarks, W);
                
                //Loop over each observation
                for(int i=0; i<nlandmarks; i++){
                    id_aruco = Landmarks(i,0);
                    //distance measurement
                    measurement(0) = Landmarks(i,1);
                    //angle measurement    
                    measurement(1) = Landmarks(i,2);
                    //actual landmark x position    
                    x_lm = lmP[id_aruco][0]; 
                    //actual landmark y position           
                    y_lm = lmP[id_aruco][1];            
                    float q = pow((state(0) - x_lm),2) + pow((state(1) - y_lm),2);
                    //distance predicted measurment
                    measurement_hat(0) = sqrt(q);       
                    //angle predicted measurement
                    measurement_hat(1) = atan2((y_lm-state(1)), (x_lm-state(0))) - state(2); 
                    
                    //this handles angle wrapping
                    measurement_hat(1) = atan2(sin(measurement_hat(1)), cos(measurement_hat(1)));

                    RCLCPP_INFO(this->get_logger(), "id: '%i'", id_aruco);
                    //RCLCPP_INFO(this->get_logger(), "sigma1: '%f'", measurement_hat(1)*180./3.14159);


                    //Observation jacobian
                    J_H <<  (state(0)-x_lm)/measurement_hat(0), (state(1)- y_lm)/measurement_hat(0), 0.0,
                            (-state(1)+y_lm)/q,             (state(0)-x_lm)/q,              -1.0;

                    //measurement noise covariance
                    S_obs = J_H*sigma*J_H.transpose() + R;

                    //Kalman gain
                    K = sigma*J_H.transpose()*S_obs.inverse();

                    //update state
                    state = state + K*(measurement - measurement_hat);

                    //update covarance
                    sigma = (I - K*J_H)*sigma;

                }
                //Eigen::MatrixXf Landmarks = Eigen::Map< Eigen::Matrix<float, nlandmarks, W> >(data);

            }
            else
            {
                state = state;
                sigma = sigma;
            }
            cam_recieved = false; 

        

        }

    private:
        void encoder_callback(const diff_rob_msgs::msg::Encoder::SharedPtr msg)
        {
            tf = double(msg->stamp.sec) + double(msg->stamp.nanosec)*1e-9;
            //left encoder
            u_enc(0) = msg->enc_l.data;
            //right encoder  
            u_enc(1) = msg->enc_r.data;  
            enconders_recieved = true;

        }

        void cam_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            nlandmarks = msg->layout.dim[0].size;
            W = msg->layout.dim[1].size;
            data = msg->data;
            cam_recieved = true;
            //id_aruco = msg->data[0]; 
            //x_aruco = msg->data[1];  
            //y_aruco = msg->data[2];
        }

        void publisher_callback()
        {
            //belief_bar
            predict();
            //belief
            update();
            //convert euler to quat
            quat.setRPY(0.0, 0.0, state(2));
            quat = quat.normalize();
            auto message = geometry_msgs::msg::Pose();
            visualization_msgs::msg::Marker marker;


            message.position.x = state(0);
            message.position.y = state(1);
            //x = msg->pose.pose.position.x;
            rclcpp::Time now = this->get_clock()->now();
            geometry_msgs::msg::PoseStamped pose_stamp;
            pose_stamp.header.frame_id = frameid;     //relative to ...
            pose_stamp.header.stamp = now;
            pose_stamp.pose = message;
            poses_vec.push_back(pose_stamp);    //similar to append
            //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", x);

            nav_msgs::msg::Path path;
            path.header.stamp = now;
            path.header.frame_id = frameid;     //relative to ...
            path.poses = poses_vec;
            marker.header.frame_id = frameid;
            marker.header.stamp = now;
            marker.ns = "robot_marker";
            marker.id = 1;
            marker.type = visualization_msgs::msg::Marker::ARROW;//::Marker::SPHERE;
            //marker.action = visualization_msgs::msg::Marker::DELETE;
            marker.scale.x = 0.8;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.pose.position.x = state(0);
            marker.pose.position.y = state(1);
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = quat.w();
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();

            marker_pub->publish(marker);
            pose_estimate_pub->publish(path);

        }


        bool enconders_recieved = false;
        bool cam_recieved = false;

        //declaring class variables
        std::string frameid = "odom";
        Eigen::Vector2d u_enc;    //enc_l, enc_r
        Eigen::Vector3d state=Eigen::Vector3d::Random();    //x,y,theta
        Eigen::Vector2d measurement;    //measurement
        Eigen::Vector2d measurement_hat;    //predicted measurement
        //Eigen::Matrix3d sigma; 
        Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero(); //covariance matrix
        Eigen::Matrix2d S_obs = Eigen::Matrix2d::Zero(); //measurement noise covariance
        //Eigen::Matrix2d Q;
        Eigen::Matrix2d Q = (Eigen::Matrix2d() << 0.1*0.1,0.0,0.0,0.1*0.1).finished();      //process noise covariance
        //Todo: measurement noise changes with distance from landmark
        Eigen::Matrix2d R = (Eigen::Matrix2d() << 0.2*0.2,0.0,0.0,0.1*0.1).finished();      //Measurement noise
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d J_fx; //motion Jacobian
        Eigen::Matrix<double, 3,2> J_fu; //controls Jacobian
        Eigen::Matrix<double, 2,3> J_H; //observation Jacobian
        Eigen::Matrix<double, 3,2> K = Eigen::Matrix<double, 3,2>::Zero(); //Kalman gain
        std::vector<double> lmP[SIZE];   //landmark position
        std::vector<double> lmR[SIZE];  //landmark orientation
        double dt;           //check this
        double ti;
        int nlandmarks;
        int W;
        float Width = 0.4;        //add to params
        std::vector<float> data;
        int id_aruco;
        float x_lm;
        float y_lm;
        float d_enc_l;
        float d_enc_r;
        double tf;
        int vec_size;
        tf2::Quaternion quat;


        //ROS stuff
        std::vector<geometry_msgs::msg::PoseStamped> poses_vec;
        rclcpp::Subscription<diff_rob_msgs::msg::Encoder>::SharedPtr enc_sub;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr landmark_cam_sub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pose_estimate_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
        rclcpp::TimerBase::SharedPtr timer;
};




int main(int argc,char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFlocalization>()); //create a shared pointer class object
    rclcpp::shutdown();
    return 0;
}