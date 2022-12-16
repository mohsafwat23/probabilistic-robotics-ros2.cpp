#include "pf.hpp"

PF::PF()
{
    //number of particles
    n_particles = 1500;

    //resize all vectors to n_particles
    particles.resize(n_particles);
    weights.resize(n_particles);
    error.resize(n_particles);

    //width of robot (todo:create global param)
    Width = 0.4;

    //avoids NaN
    dt = 0.00;

    //initialize weight sum
    weight_total = 0.0;

    //avoids NaN
    u_enc = Eigen::Vector2d::Zero();
}

void PF::genParticles(struct limits range)
{
    // Initializing of uniform_real_distribution class
    std::default_random_engine generator;

    //particles state x,y,theta
    std::uniform_real_distribution<double> distributionX(range.Xlower, range.Xupper);
    std::uniform_real_distribution<double> distributionY(range.Ylower, range.Yupper);
    std::uniform_real_distribution<double> distributionTheta(-pi, pi);

    for(int i=0; i < n_particles; i++)
    {
        //particle structure
        particle p;
        p.i = i;

        //weights placeholder
        p.weight = 1.0;
        p.x = distributionX(generator);
        p.y = distributionY(generator);
        p.theta = distributionTheta(generator);

        //vector of particles
        particles[i] = p;

    }

}

void PF::prediction(double std_model[])
{
    std::default_random_engine gen;

    //for each particle, propogate
    for(int i=0; i<n_particles; i++)
    {
        //vector address
        particle *p = &particles[i];

        //this also works
        // particles[i].x += cos(particles[i].theta)*dt*((u_enc(0)+u_enc(1))/2.0);
        // particles[i].y += sin(particles[i].theta)*dt*((u_enc(0)+u_enc(1))/2.0);
        // particles[i].theta += dt*((u_enc(1)-u_enc(0))/Width);
        p->x += cos(p->theta)*dt*((u_enc(0)+u_enc(1))/2.0);
        p->y += sin(p->theta)*dt*((u_enc(0)+u_enc(1))/2.0);
        p->theta += dt*((u_enc(1)-u_enc(0))/Width);

        //this handles angle wrapping
        p->theta = atan2(sin(p->theta), cos(p->theta));

        //gaussian distribution 
        std::normal_distribution<double> dist_x(p->x, std_model[0]);
        std::normal_distribution<double> dist_y(p->y, std_model[1]);
        std::normal_distribution<double> dist_theta(p->theta, std_model[2]);

        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->theta = dist_theta(gen);
        // // update the particle attributes
        // particles[i].x = dist_x(gen);
        // particles[i].y = dist_y(gen);
        // particles[i].theta = dist_theta(gen);    
    }
}

void PF::weigh(std::vector<double> landmark, const double dist, const double bearing, double std_sensor[])
{
    measurement(0) = dist;
    measurement(1) = bearing;

    //temporary: avoids readding 
    weight_total = 0.0;

    for(int i=0; i<n_particles; i++)
    {
        particle *p = &particles[i];

        //observation model
        measurement_hat(0) = sqrt(pow((p->x - landmark[0]),2) + pow((p->y - landmark[1]),2));
        measurement_hat(1) = atan2((landmark[1]-p->y), (landmark[0] - p->x)) - p->theta; 
        
        //this handles angle wrapping
        measurement_hat(1) = atan2(sin(measurement_hat(1)), cos(measurement_hat(1)));
        p->weight *= gaussian(measurement(0), std_sensor[0], measurement_hat(0));
        p->weight *= gaussian(measurement(1), std_sensor[1], measurement_hat(1));
        error[i] = measurement_hat(1) - measurement(1);

        //p->weight *= (exp(-0.5 * (pow((measurement_hat(0) - measurement(0)), 2) / pow(std_sensor[0], 2) + pow((measurement_hat(1) - measurement(1)), 2) / pow(std_sensor[1], 2))))/(2*pi*std_sensor[0]*std_sensor[1]);
        weight_total += p->weight;  //change this because unneccasry adding
       

    }

    for(int i=0; i<n_particles; i++)
    {
        particle *p = &particles[i];

        //this stops from particle weights going to 0
        p->weight += 1.e-300;   

        //normalize
        p->weight /= weight_total;
        weights[i] = p->weight;
    }

}

//Todo: better resampling algorithm
void PF::resample()
{
    std::default_random_engine gen;

    //resample with probabilty of particle is its weight
    std::discrete_distribution<int> distribution(weights.begin(), weights.end());
    std::vector<particle> resampled_particles;
    for(int i=0; i<n_particles; i++)
    {
        resampled_particles.push_back(particles[distribution(gen)]);
    }

    particles = resampled_particles;

}

double PF::gaussian(double mu, double sigma, double x)
{
    return (1.0/(sigma*sqrt(2.0*pi)))*exp(-0.5*pow(((x - mu)/sigma), 2));
}


void PF::setEncoders(const float encl, const float encr)
{
    u_enc(0) = encl;
    u_enc(1) = encr;
}

void PF::setDt(const double tf, const double ti)
{
    dt = tf - ti;
    if (dt > 1.0){
        dt = 0.0;
    }
}

std::vector<particle> PF::getParticles()
{
    return particles;
}

double PF::getError(int i)
{
    return error[i];
}

