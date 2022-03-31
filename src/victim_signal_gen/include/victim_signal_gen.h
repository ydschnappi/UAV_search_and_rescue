//created by Dian Yuan at 03.11.2022

#ifndef VICTIM_SIGNAL_GEN_H
#define VICTIM_SIGNAL_GEN_H

#include <vector>
#include <eigen3/Eigen/Dense>


//double TX_to_RX_strength(double fx,double fy,double fz,double vx,double vy,double vz);
double TX_to_RX_strength(Eigen::Vector3d uav_pos,Eigen::Vector3d victim_pos);


class Victim{
    int _id;
    Eigen::Vector3d _coord;
  
public:
    double fRand(double fMin,double fMax);
    Victim(int id);
    int getID();
    Eigen::Vector3d getCoord();
    
};

std::vector<Victim> victim_pos_generator();

#endif
