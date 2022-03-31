#include "ros/ros.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class BasicPlanner {
    
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub;
    ros::Subscriber current_state;

    ros::NodeHandle nh_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;
    Eigen::Vector3d current_point;
    Eigen::Vector3d former_goal_position;
    double max_v_ = 5.0; // m/s
    double max_a_ = 2.0; // m/s^2
    double max_ang_v_;
    double max_ang_a_;
    
public:
    BasicPlanner(){

        sub = nh_.subscribe("next_waypoint",1,&BasicPlanner::callback,this);
        current_state = nh_.subscribe("current_state",10,&BasicPlanner::current_callback,this);
        former_goal_position << 0.0, 0.0, 1.29;
    }

    void current_callback(const nav_msgs::Odometry& cur_state){

        current_point<<cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
        current_velocity_ << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
        
    }


    void callback(const nav_msgs::Odometry& next_state){

        // define goal point

        Eigen::Vector3d goal_position, goal_velocity;
        goal_position << next_state.pose.pose.position.x,next_state.pose.pose.position.y,next_state.pose.pose.position.z;
        goal_velocity << next_state.twist.twist.linear.x,next_state.twist.twist.linear.y,next_state.twist.twist.linear.z;


        if(former_goal_position != goal_position && goal_position != current_point){
            pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
            pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);
    
            former_goal_position = goal_position;
            mav_trajectory_generation::Trajectory trajectory;
            planTrajectory(goal_position, goal_velocity, &trajectory);
            publishTrajectory(trajectory);

        }
    }

    bool planTrajectory(const Eigen::VectorXd& goal_pos,
                        const Eigen::VectorXd& goal_vel,
                        mav_trajectory_generation::Trajectory* trajectory){
        // 3 Dimensional trajectory => through carteisan space, no orientation
        const int dimension = 3;

        // Array for all waypoints and their constrains
        mav_trajectory_generation::Vertex::Vector vertices;

        // Optimze up to 4th order derivative (SNAP)
        const int derivative_to_optimize =
                mav_trajectory_generation::derivative_order::SNAP;

        // we have 2 vertices:
        // Start = current position
        // end = desired position and velocity
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);


        /******* Configure start point *******/
        //set start point constraints to current position and set all derivatives to zero
        //start.makeStartOrEnd(current_pose_.translation(),derivative_to_optimize);
        //std::cout << "current_point:"<< current_point(0) << current_point(1) <<current_point(2);
        start.makeStartOrEnd(current_point,derivative_to_optimize);



        // set start point's velocity to be constrained to current velocity
        start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                            current_velocity_);

        // add waypoint to list
        vertices.push_back(start);


        Eigen::Vector3d pos, vel;
        pos = (goal_pos + current_point) / 2.0;

        // if (current_velocity_.norm()>1)
        //     vel = current_velocity_;
        // else{
        //     vel = (goal_pos - current_point).normalized();
        // }
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,pos);
        //middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,vel);
        vertices.push_back(middle);
        middle.removeConstraint(mav_trajectory_generation::derivative_order::POSITION);
        //middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);



        /******* Configure end point *******/
        // set end point constraints to desired position and set all derivatives to zero
        end.makeStartOrEnd(goal_pos,
                        derivative_to_optimize);

        // set start point's velocity to be constrained to current velocity
        end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        goal_vel);

        // add waypoint to list
        vertices.push_back(end);

        // setimate initial segment times
        std::vector<double> segment_times;
        segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

        // Set up polynomial solver with default params
        mav_trajectory_generation::NonlinearOptimizationParameters parameters;

        // set up optimization problem
        const int N = 10;
        mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

        // constrain velocity and acceleration
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

        // solve trajectory
        opt.optimize();

        // get trajectory as polynomial parameters
        opt.getTrajectory(&(*trajectory));

        return true;
            
            
            
    }

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
        //send trajectory as markers to display them in RVIZ
        visualization_msgs::MarkerArray markers;
        double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";

        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        pub_markers_.publish(markers);

        //send trajectory to be executed on UAV
        mav_planning_msgs::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
        msg.header.frame_id = "world";
        pub_trajectory_.publish(msg);

        return true;
    }  
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_generator");
    BasicPlanner b;
    ros::spin();
}