#include "pf.hpp"
#include "pf_node.hpp"

PF_Node::PF_Node() : Node("pf_node")

    {
        struct limits range;
        range.Xlower = -2.0;
        range.Xupper = 2.0;
        range.Ylower = -2.0;
        range.Yupper = 2.0;
        enc_sub = this->create_subscription<diff_rob_msgs::msg::Encoder>("/encoder", 10, std::bind(&PF_Node::encoder_callback, this, _1));
        landmark_cam_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/landmark_dist", 10, std::bind(&PF_Node::cam_callback, this, _1));
        marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/particles", 2);
        timer = this->create_wall_timer(100ms, std::bind(&PF_Node::publisher_callback, this));
        frameid = "odom";

        //Declare Parameters
        for(int i=0; i<12; i++)
        {
            declare_parameter("landmark" + std::to_string(i) + ".xyz", std::vector<double>{0.0,0.0,0.0});
            declare_parameter("landmark" + std::to_string(i) + ".rpy", std::vector<double>{0.0,0.0,0.0});
            lmP[i] = get_parameter("landmark" + std::to_string(i) + ".xyz").as_double_array();
            lmR[i] = get_parameter("landmark" + std::to_string(i) + ".rpy").as_double_array();
        }
        pf.genParticles(range);
 


    }

    void PF_Node::encoder_callback(const diff_rob_msgs::msg::Encoder::SharedPtr msg)
    {
        tf = double(msg->stamp.sec) + double(msg->stamp.nanosec)*1e-9;
        //RCLCPP_INFO(get_logger(), "xPosition: %f", msg->enc_l.data);
        if(!std::isnan(msg->enc_l.data + msg->enc_r.data))
        {
            pf.setEncoders(msg->enc_l.data, msg->enc_r.data);
        }
        encoder_recieved = true;

    }

    void PF_Node::cam_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        nlandmarks = msg->layout.dim[0].size;
        W = msg->layout.dim[1].size;
        data = msg->data;
        cam_recieved = true;
        //id_aruco = msg->data[0]; 
        //x_aruco = msg->data[1];  
        //y_aruco = msg->data[2];
    }


    void PF_Node::publisher_callback()
    {
        double std_model[] = {0.01, 0.01, 0.01};
        double std_sensor[] = {0.4, 0.4};


        std::vector<particle> particles = pf.getParticles();
        if(encoder_recieved)
        {
            pf.prediction(std_model);
            
            pf.setDt(tf, ti);

            ti = tf;
        }

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

                //RCLCPP_INFO(get_logger(), "int: %i", id_aruco);


                pf.weigh(lmP[id_aruco], Landmarks(i,1), Landmarks(i,2), std_sensor);

            }
            pf.resample();
    
        }


        visualization_msgs::msg::MarkerArray particle_markers;
        visualization_msgs::msg::Marker marker;
        for(int i=0; i < pf.n_particles; ++i)
        {
            //RCLCPP_INFO(get_logger(), "error: %f", pf.getError());
            //RCLCPP_INFO(get_logger(), "weight: %f", particles[i].weight);
            
            // if(particles[i].weight > 0)
            // {
            //     RCLCPP_INFO(get_logger(), "weight: %f", particles[i].weight);
            //     // RCLCPP_INFO(get_logger(), "xPosition: %f", particles[i].x);
            //     // RCLCPP_INFO(get_logger(), "yPosition: %f", particles[i].y);
            // }
            tf2::Quaternion quat;
            quat.setRPY(0, 0, particles[i].theta);
            quat = quat.normalize();

            rclcpp::Time now = this->get_clock()->now();
            //markers
            marker.header.frame_id = frameid;
            marker.header.stamp = now;
            marker.ns = "robot_marker";
            marker.id = i;
            //RCLCPP_INFO(get_logger(), "index: %i", i);

            marker.type = visualization_msgs::msg::Marker::ARROW;//::Marker::SPHERE;
            //marker.action = visualization_msgs::msg::Marker::ADD;

            marker.scale.x = 0.5;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;
            //alpha is marker visibility 
            marker.color.a = particles[i].weight*pf.n_particles; 
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.pose.position.x = particles[i].x;
            marker.pose.position.y = particles[i].y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = quat.w();
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            particle_markers.markers.push_back(marker);

        }
        marker_pub->publish(particle_markers);
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        particle_markers.markers.clear();
        encoder_recieved = false;
        cam_recieved = false;







    }




int main(int argc,char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PF_Node>()); //create a shared pointer class object
    rclcpp::shutdown();
    return 0;
}