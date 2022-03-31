#ifndef PLANNING_H
#define PLANNING_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>

#define ACCURACY_LIMIT 490 //10000/20

#define ACCURACY_UPPER 458 //10000/22

struct signal_info{
    Eigen::Vector3d pos;
    double signal_strength;
};

struct victim_signal_collected{
    int id;
    std::vector<signal_info> cal_data_set;
};

bool check_pos(Eigen::Vector3d &cur_pos, Eigen::Vector3d &desired_pos);

Eigen::Vector3d cal_pos(const std::vector<signal_info> &signal);



#endif
