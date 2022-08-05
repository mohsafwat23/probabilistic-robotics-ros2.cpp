#pragma once
#include <Eigen/Dense>

class EKF {
    public:
        void predict(){}

        void update(){}

    private:
        int lms;                    //number of lms
        Eigen::Vector2d u_enc();    //enc_l, enc_r
        Eigen::Vector3d state();    //x,y,theta
        Eigen::MatrixX3d measurement(lms);
        // https://campar.in.tum.de/Chair/KalmanFilter  
                                    //How uncertain are you with model; how far is x_hat from model and x
                                    //How uncertain are you with measurement; how far is x_hat from measurement and x


};