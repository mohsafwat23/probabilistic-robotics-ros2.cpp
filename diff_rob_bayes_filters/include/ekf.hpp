#ifndef EKF_h
#define EKF_h

#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>

class EKF {
    public:
        EKF();

        void predict();

        void update(std::vector<double> landmark, const double dist, const double bearing);

        void setEncoders(const float encl, const float encr);

        void setDt(const double tf, const double ti);

        Eigen::VectorXd getEncoders();
        
        Eigen::VectorXd getState();

        Eigen::MatrixXd getCovariance();


    private:

        double dt;           //time
        float Width;
        Eigen::Vector2d u_enc;    
        Eigen::Vector3d state;    //x,y,theta
        Eigen::Matrix3d sigma;
        
        //How uncertain are you with model; how far is x_hat from model and x
        Eigen::Matrix2d Q;       //process noise covariance

        //How uncertain are you with measurement; how far is x_hat from measurement and x
        //Todo: measurement noise changes with distance from landmark
        Eigen::Matrix2d R;      //Measurement noise
        Eigen::Matrix3d I; 
        
        //motion model Jacobian 
        Eigen::Matrix3d J_fx; 
        
        //controls inputs Jacobian
        Eigen::Matrix<double, 3,2> J_fu; 

        //observation model Jacobian
        Eigen::Matrix<double, 2,3> J_H; 

        Eigen::Vector2d measurement;    //measurement
        Eigen::Vector2d measurement_hat;    //predicted measurement

        //Eigen::Vector2d landmark;

        Eigen::Matrix<double, 3,2> K; //= Eigen::Matrix<double, 3,2>::Zero(); //Kalman gain

        Eigen::Matrix2d S_obs; // = Eigen::Matrix2d::Zero(); //measurement noise covariance

};

#endif