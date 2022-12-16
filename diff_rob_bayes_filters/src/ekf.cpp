#include "ekf.hpp"

EKF::EKF()
{
    Width = 0.4;
    dt = 0.01;
    state = Eigen::Vector3d::Zero();
    u_enc = Eigen::Vector2d::Zero();
    sigma = Eigen::Matrix3d::Zero();
    //landmark = Eigen::Vector2d::Zero();
    Q = (Eigen::Matrix2d() << 0.1*0.1,0.0,0.0,0.1*0.1).finished();
    R = (Eigen::Matrix2d() << 0.2*0.2,0.0,0.0,0.1*0.1).finished();
    I = Eigen::Matrix3d::Identity();


}

void EKF::predict()
{
    //prior motion model
    //x position
    state(0) += cos(state(2))*dt*((u_enc(0)+u_enc(1))/2.0);
    //y position
    state(1) += sin(state(2))*dt*((u_enc(0)+u_enc(1))/2.0);
    //theta
    state(2) += dt*((u_enc(1)-u_enc(0))/Width);

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
}

void EKF::update(std::vector<double> landmark, const double dist, const double bearing)
{
    measurement(0) = dist;
    measurement(1) = bearing;
    measurement_hat(0) = sqrt(pow((state(0) - landmark[0]),2) + pow((state(1) - landmark[1]),2));
    measurement_hat(1) = atan2((landmark[1]-state(1)), (landmark[0]-state(0))) - state(2); 
    
    //this handles angle wrapping
    measurement_hat(1) = atan2(sin(measurement_hat(1)), cos(measurement_hat(1)));
    
    
    //Observation jacobian
    J_H <<  (state(0)-landmark[0])/measurement_hat(0), (state(1)- landmark[1])/measurement_hat(0), 0.0,
            (-state(1)+landmark[1])/pow(measurement_hat(0),2),             (state(0)-landmark[0])/pow(measurement_hat(0),2),              -1.0;

    //measurement noise covariance
    S_obs = J_H*sigma*J_H.transpose() + R;

    //Kalman gain
    K = sigma*J_H.transpose()*S_obs.inverse();

    //update state
    state = state + K*(measurement - measurement_hat);

    //update covarance
    sigma = (I - K*J_H)*sigma;

}

void EKF::setEncoders(const float encl, const float encr)
{
    u_enc(0) = encl;
    u_enc(1) = encr;
}

void EKF::setDt(const double tf, const double ti)
{
    dt = tf - ti;
    if (dt > 1.0){
        dt = 0.0;
    }
}

Eigen::VectorXd EKF::getEncoders()
{
    return u_enc;
}

Eigen::VectorXd EKF::getState()
{
    return state;
}

Eigen::MatrixXd EKF::getCovariance()
{
    return sigma;
}