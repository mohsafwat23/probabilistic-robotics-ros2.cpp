#include "ekf.hpp"
#include "ekf_node.hpp"

EKF_Node::EKF_Node() : Node("ekf_node") //inherits from Node

    {
        enc_sub = this->create_subscription<diff_rob_msgs::msg::Encoder>("/encoder", 10, std::bind(&EKF_Node::encoder_callback, this, _1));
        landmark_cam_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/landmark_dist", 10, std::bind(&EKF_Node::cam_callback, this, _1));
        pose_estimate_pub = this->create_publisher<nav_msgs::msg::Path>("/pose_estimate", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/robot_pose_estimate", 10);
        timer = this->create_wall_timer(50ms, std::bind(&EKF_Node::publisher_callback, this));
        frameid = "odom";
        //Declare Parameters
        for(int i=0; i<12; i++)
        {
            declare_parameter("landmark" + std::to_string(i) + ".xyz", std::vector<double>{0.0,0.0,0.0});
            declare_parameter("landmark" + std::to_string(i) + ".rpy", std::vector<double>{0.0,0.0,0.0});
            lmP[i] = get_parameter("landmark" + std::to_string(i) + ".xyz").as_double_array();
            lmR[i] = get_parameter("landmark" + std::to_string(i) + ".rpy").as_double_array();
        }

    }

    void EKF_Node::encoder_callback(const diff_rob_msgs::msg::Encoder::SharedPtr msg)
    {
        tf = double(msg->stamp.sec) + double(msg->stamp.nanosec)*1e-9;
        //left encoder
        //ekf.u_enc(0) = msg->enc_l.data;
        //right encoder  
        //ekf.u_enc(1) = msg->enc_r.data;  
        //enconders_recieved = true;
        ekf.setEncoders(msg->enc_l.data, msg->enc_r.data);
        encoder_recieved = true;

    }

    void EKF_Node::cam_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        nlandmarks = msg->layout.dim[0].size;
        W = msg->layout.dim[1].size;
        data = msg->data;
        cam_recieved = true;
        //id_aruco = msg->data[0]; 
        //x_aruco = msg->data[1];  
        //y_aruco = msg->data[2];
    }

    void EKF_Node::publisher_callback()
    {
        if (encoder_recieved && !std::isnan(ekf.getEncoders().sum()))
        {
            ekf.setDt(tf, ti);
            ekf.predict();
            RCLCPP_INFO(this->get_logger(), "id: '%f'", ekf.getState()(0));
        }
        ti = tf;
        //check if camera data is coming
        if(cam_recieved)
        {
            vec_size = data.size();
        }
        else{
            vec_size = 0;
        }
        //check if the vector is valid and contains data
        if(vec_size == nlandmarks*W && vec_size > 0){
            //Eigen::Map<Eigen::MatrixXf> Landmarks(data.data(), nlandmarks, W);
            Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Landmarks(data.data(), nlandmarks, W);
            //Loop over each observation
            for(int i=0; i<nlandmarks; i++)
            {
                id_aruco = Landmarks(i,0);
                ekf.update(lmP[id_aruco], Landmarks(i,1), Landmarks(i,2));

            }
        
        }
        quat.setRPY(0.0, 0.0, ekf.getState()(2));
        quat = quat.normalize();
        visualization_msgs::msg::Marker marker;
        auto message = geometry_msgs::msg::Pose();
        message.position.x = ekf.getState()(0);
        message.position.y = ekf.getState()(1);

        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped pose_stamp;
        pose_stamp.header.frame_id = frameid;     //relative to ...
        pose_stamp.header.stamp = now;
        pose_stamp.pose = message;
        poses_vec.push_back(pose_stamp);    //similar to append
        nav_msgs::msg::Path path;
        path.header.stamp = now;
        path.header.frame_id = frameid;     //relative to ...
        path.poses = poses_vec;

        //markers
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
        marker.pose.position.x = ekf.getState()(0);
        marker.pose.position.y = ekf.getState()(1);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = quat.w();
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker_pub->publish(marker);
        pose_estimate_pub->publish(path);

        cam_recieved = false;
        encoder_recieved = false;


    }




int main(int argc,char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKF_Node>()); //create a shared pointer class object
    rclcpp::shutdown();
    return 0;
}