//Author: Dian Yuan; Jianyu Tang
//Comments:
//11.03.2022:  Contains the functions required by this package； currently just basic idea sensor model
//12.03.2022:  Change the structure of coordinate into Eigen::Vector3d
//18.03.2022:  Change the logic of sensor and add noise
//#include <ros/ros.h>
#include <victim_signal_gen.h>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <random>
#include <chrono>
#include <iostream>


#define VICTIMS_NUM 4
#define X_LOW -110.0
#define X_UP  70.0
#define Y_LOW 0.0
#define Y_UP 70.0
#define Z_LOW 1.0
#define Z_UP 10.0

#define TX_strength 10000.0
#define c0 1
#define n 1


double Victim::fRand(double fMin,double fMax){
    //srand(time(nullptr));
    double f = (double)rand()/RAND_MAX;
    return fMin+f*(fMax-fMin);
}

Victim::Victim(int id){
    _id=id;
    _coord<<fRand(X_LOW,X_UP),fRand(Y_LOW,Y_UP),fRand(Z_LOW,Z_UP);
    //_coord<<victims_X[id-1],victims_Y[id-1],victims_Z[id-1];
}

int Victim::getID(){
    return _id;
}

Eigen::Vector3d Victim::getCoord(){
    return _coord;

}

double random_norm(double u, double sigma){
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(u, sigma); 
  double v = distribution(generator);
  v = int(v*1000)/1000.0;
  return v;
}

// basic signal spread model without noise
double TX_to_RX_strength(Eigen::Vector3d uav_pos, Eigen::Vector3d victim_pos){
    double distance = (uav_pos-victim_pos).norm();
    double RX = TX_strength*c0/pow(distance,n);
    if (distance > 0.1 && distance <= 22){
        double total = 0;
        for(int i=0;i<10;i++){
            double noise = random_norm(0,distance/2)/5;
            total += TX_strength*c0/(pow(distance,n)+0.9) + noise;
            //total += TX_strength*c0/(pow(distance,n)+0.9);
        }
        double RX_strength = total/10.0;
        return RX_strength;
    }
    else if (distance>22 && distance<=60) {
        double total = 0;
        for(int i=0;i<10;i++){
            double noise = random_norm(0,distance*2)/3;
            total += TX_strength*c0/(pow(distance,n)+0.9) + noise;
        }
        double RX_strength = total/10.0;
        return RX_strength;
    }
    else if (distance <= 0.1){
        return TX_strength;
    }
    else if (distance > 60){
        return 0;
    }
}

std::vector<Victim> victim_pos_generator(){
    srand(time(nullptr));
    std::vector <Victim> victims; 
    for(int i=1;i<=VICTIMS_NUM;i++){
        victims.push_back(Victim(i));
    }
    return victims;
}
