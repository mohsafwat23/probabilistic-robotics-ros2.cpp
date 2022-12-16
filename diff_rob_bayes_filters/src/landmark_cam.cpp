#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "diff_rob_msgs/msg/landmark.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
//#include "image_transport/image_transport.h"
//#include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

class LandmarkPublisher: public rclcpp::Node
{

    public:
    LandmarkPublisher() : Node("landmark_node")
    {

        declare_parameter("markerlength", 0.1778);
        markerLength = get_parameter("markerlength").as_double();
        RCLCPP_INFO(get_logger(), "Double parameter: %f", markerLength);
        img_sub = this->create_subscription<sensor_msgs::msg::Image>("/diff_robot/image_raw", 10, std::bind(&LandmarkPublisher::img_callback, this, _1));
        //dist_pub = this->create_publisher<diff_rob_msgs::msg::Landmark>("/landmark_dist", 10);
        dist_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/landmark_dist", 10);

    }
    
    private:
        void img_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            auto message = diff_rob_msgs::msg::Landmark();  
            cv::Mat imageCopy;
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv_ptr->image.copyTo(imageCopy);
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners;
            cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);
            int size = ids.size();
            auto ms = std_msgs::msg::Float32MultiArray();
            ms.data.clear();

            // if at least one marker detected
            if (ids.size() > 0)
                {
                ms.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                ms.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
                ms.layout.dim[0].label = "height";
                ms.layout.dim[1].label = "width";
                ms.layout.dim[0].size = size;
                ms.layout.dim[1].size = W;
                ms.layout.dim[0].stride = size*W;
                ms.layout.dim[1].stride = W;
                ms.layout.data_offset = 0;
                std::vector<float> vec(size*W, 0);
                cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

                std::vector<cv::Vec3d> rvecs, tvecs;
                cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
                for(int i=0; i<size; i++)
                {
                    //message.id.data.push_back(ids[i]);
                    //ms.data.push_back(ids[i]);
                    //ms.data.push_back(5.0);
                    cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.2);
                    //cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.3);
                    cv::Vec3d tvec = tvecs[i];
                    cv::Vec3d rvec = rvecs[i];
                    float id = ids[i];

                    //float dist = std::sqrt(x*x + z*z);
                    //float angle = std::atan2(x, z); //check this! for robot heading
    
                    float y = -tvec[0];  //left and right
                    float x = tvec[2];  //depth into camera

                    float dist = sqrt(pow(x,2) + pow(y,2));
                    float angle = std::atan2(y, x);
                    vec[W*i] = id;
                    vec[W*i + 1] = dist;
                    vec[W*i + 2] = angle;

                    //double x = tvec[0];
                    //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", dist);
                    //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", tvecs[i][1]);
                    //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", tvecs[i][2]);


                }
                ms.data = vec;
                dist_pub->publish(ms);

        // draw axis for each marker
                }
            cv::imshow(OPENCV_WINDOW, imageCopy);
            cv::waitKey(1);

        }
    // Parameters declaration
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    const std::string OPENCV_WINDOW = "Image window";
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) 
                            << 921.9938565545156, 0.0,             480.5, 
                                0.0,            921.9938565545156, 360.5, 
                                0.0,              0.0,              1.0);
    float markerLength;// = 0.1778;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 0.0,0.0,0.0,0.0,0.0);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
    //rclcpp::Publisher<diff_rob_msgs::msg::Landmark>::SharedPtr dist_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dist_pub;
    const int W = 3;

    //auto markerLength = get_parameter("markerlength").as_double();
    // cv::Mat extrinsic = (cv::Mat_<double>(3, 4) 
    //                         << 921.9938565545156, 0.0,             480.5, 0.0, 
    //                             0.0,            921.9938565545156, 360.5, 0.0,
    //                             0.0,              0.0,              1.0 , 0.0);

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LandmarkPublisher>());
    rclcpp::shutdown();
    return 0;
}