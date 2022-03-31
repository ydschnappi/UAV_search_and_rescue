// Author: Dian Yuan
// This node is aimed at visualization in RVIZ
// The location of victims and the trajectory of UAV can be seen in RVIZ
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <victim_signal_gen/VictimCoord.h>
#include <victim_signal_gen/VictimArray.h>
#include <nav_msgs/Odometry.h>


class traj_visualize_node{ 
    ros::NodeHandle nh;
    ros::Publisher vic_pub, traj_pub, av_pub;
    ros::Subscriber vic_pos, uav_now;
    ros::Time start;
    ros::Timer vic_timer, traj_timer, av_timer;
    visualization_msgs::Marker victims;
    visualization_msgs::Marker traj;
    visualization_msgs::Marker av; 
    

public:
    traj_visualize_node(){

        vic_pos = nh.subscribe("victim_coordinate",1, &traj_visualize_node::getVicPos,this);
        uav_now = nh.subscribe("current_state", 100, &traj_visualize_node::getTraj, this);
        start = ros::Time::now();
        vic_timer = nh.createTimer(ros::Duration(1.0),&traj_visualize_node::victimLoop,this);
        vic_pub = nh.advertise<visualization_msgs::Marker>("vic_marker",10);
        traj_timer = nh.createTimer(ros::Duration(1/100.0),&traj_visualize_node::trajLoop,this);
        traj_pub = nh.advertise<visualization_msgs::Marker>("traj_marker",100);
        av_timer = nh.createTimer(ros::Duration(1/100.0),&traj_visualize_node::avLoop,this);
        av_pub = nh.advertise<visualization_msgs::Marker>("av_marker",100);
        ROS_INFO("[traj_visualize_node] Node initialised");

        //setup attributes for RVIZ
        victims.header.frame_id = traj.header.frame_id = "world";
        victims.header.stamp = traj.header.stamp = ros::Time::now();
        victims.ns = traj.ns = "traj_visualize_node";
        victims.id = 0;
        victims.action = visualization_msgs::Marker::ADD;
        victims.pose.orientation.w = 1.0;
        victims.type = visualization_msgs::Marker::POINTS;
        victims.scale.x = 1.5;
        victims.scale.y = 1.5;
        victims.scale.z = 1.5;
        victims.color.b = 1.0f;
        victims.color.a = 1.0;
        victims.lifetime = ros::Duration();

        traj.id = 1;
        traj.action = visualization_msgs::Marker::ADD;
        traj.pose.orientation.w = 1.0;
        traj.type = visualization_msgs::Marker::LINE_STRIP;
        traj.scale.x = 0.75;
        traj.scale.y = 0.75;
        traj.scale.z = 0.75;
        traj.color.r = 0.75;
        traj.color.a = 0.75;
        traj.lifetime = ros::Duration();

        av.header.frame_id = "world";
        av.ns = "traj_visualize_node"; 
        av.id = 2;
        av.header.stamp = ros::Time();
        av.type = visualization_msgs::Marker::MESH_RESOURCE;
        av.mesh_resource = "package://trajectory_visualization/mesh/quadrotor.dae";
        av.action = visualization_msgs::Marker::ADD;
        av.pose.orientation.w = 1.0;
        av.scale.x = av.scale.y = av.scale.z = 15.0;
        av.color.a = 1.0;
        av.color.r = .25; av.color.g = .52; av.color.b = 1;
        av.lifetime = ros::Duration(0.1);
    }

    void getVicPos(const victim_signal_gen::VictimArray& posArray){
        geometry_msgs::Point v;
        for(int i = 0; i < posArray.coordinate.size();i++){
            v.x = posArray.coordinate[i].x;
            v.y = posArray.coordinate[i].y;
            v.z = posArray.coordinate[i].z;
            victims.points.push_back(v);
        }
    }

    void getTraj(const nav_msgs::Odometry& cur_state){
        geometry_msgs::Point p;
        p.x = cur_state.pose.pose.position.x;
        p.y = cur_state.pose.pose.position.y;
        p.z = cur_state.pose.pose.position.z;
        traj.points.push_back(p);
        av.pose=cur_state.pose.pose;
    }

    void victimLoop(const ros::TimerEvent& t){
        vic_pub.publish(victims);
    }

    void trajLoop(const ros::TimerEvent& t){
        traj_pub.publish(traj);
    } 

    void avLoop(const ros::TimerEvent& t){
        av_pub.publish(av);
    } 

};


int main (int argc, char** argv){
    ros::init(argc,argv,"traj_visualize_node");
    traj_visualize_node n;
    ROS_INFO("initialised");
    ros::spin();
    return 0;
}