//Author: Dian Yuan
//Comments:
//12.03.2022 UAV start pastion [0,0,1.2954)
//15.03.2022 add victim coordinate publisher (for presentation in rviz)

#include <victim_signal_gen.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <victim_signal_gen/SignalArray.h>
#include <victim_signal_gen/VictimSignal.h>
#include <victim_signal_gen/VictimCoord.h>
#include <victim_signal_gen/VictimArray.h>


class victim_signal_node{
  ros::NodeHandle nh;
  ros::Subscriber uav_pos; //current postion subcriber
  ros::Publisher signal_strength_pub; //signal strength publisher
  ros::Publisher victim_coordinate_pub;

  ros::Time start;
  ros::Timer signal_timer; //timer for main signal loop

  Eigen::Vector3d cur_pos; //current postion of UAV in the world frame

  double hz; //frequency of the signal publishing

  std::vector<Victim> victims;
  int flag;


public:
  victim_signal_node():hz(15.0),flag(1),victims(victim_pos_generator()){
    uav_pos = nh.subscribe("current_state", 100, &victim_signal_node::onCurrentState, this);
    start = ros::Time::now();
    signal_timer = nh.createTimer(ros::Duration(1/hz), &victim_signal_node::signalLoop, this);
    signal_strength_pub = nh.advertise<victim_signal_gen::SignalArray>("victims_signal_strength",1);
    victim_coordinate_pub = nh.advertise<victim_signal_gen::VictimArray>("victim_coordinate",1);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
    cur_pos << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
  }

  void signalLoop(const ros::TimerEvent& t){
    victim_signal_gen::VictimSignal signal;
    victim_signal_gen::SignalArray strength_msg;
    victim_signal_gen::VictimCoord coord;
    victim_signal_gen::VictimArray coord_msg;

    //To make sure that the SignalArray would't be empty
    signal.ID=-1;
    signal.signal_strength=-1;
    strength_msg.signal.push_back(signal);

    for(int i=0; i<victims.size();i++){
      int id = victims[i].getID();
      Eigen::Vector3d victim_pos = victims[i].getCoord();
      coord.x=victim_pos(0);
      coord.y=victim_pos(1);
      coord.z=victim_pos(2);
      double signal_strength = TX_to_RX_strength(cur_pos,victim_pos);
      if (signal_strength>0){
        signal.ID = id;
        signal.signal_strength = signal_strength;
        strength_msg.signal.push_back(signal); 
      }
      coord_msg.coordinate.push_back(coord);
    }
    signal_strength_pub.publish(strength_msg); //publish the signal strength
    victim_coordinate_pub.publish(coord_msg); // publish the victim coordinate (Only for presetation in RVIZ)
    if(flag){
      for(int i=0; i<coord_msg.coordinate.size();i++){
        ROS_INFO_ONCE("Generate %d victims in the area",coord_msg.coordinate.size());
        auto data = coord_msg.coordinate[i];
        ROS_INFO_STREAM("x: "<<data.x<<" y: "<<data.y<<" z: "<<data.z<<"\n");
      }
      flag = 0;
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "victim_signal_node");
  victim_signal_node n;
  ros::spin();
  return 0;
}

