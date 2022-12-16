#ifndef PF_h
#define PF_h

#include <iostream>
#include <random>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <vector>
const double pi = 3.14159265359;


struct particle
{
    int i;
    double weight;
    double x;
    double y;
    double theta;
};

struct limits
{
    float Xlower;
    float Xupper;
    float Yupper;
    float Ylower;
};

class PF 
{
    public:
        PF();

        void genParticles(struct limits range);

        void prediction(double std_pose[]);

        void weigh(std::vector<double> landmark, const double dist, const double bearing, double std_sensor[]);

        void resample();

        double gaussian(double mu, double sigma, double x);

        void setEncoders(const float encl, const float encr);

        void setDt(const double tf, const double ti);

        std::vector<particle> getParticles();

        double getError(int i);

        int n_particles;

    private:
        std::vector<particle> particles;
        std::vector<double> weights;
        std::vector<double> error;
        double dt;
        float Width;
        Eigen::Vector2d u_enc;    
        Eigen::Vector2d measurement;    //measurement
        Eigen::Vector2d measurement_hat;    //predicted measurement
        double weight_total;



};


#endif
