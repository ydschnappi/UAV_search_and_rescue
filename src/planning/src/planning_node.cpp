// Created by Yinglei Song, Jianyu Tang , Dian Yuan
// This node implements the algorithm for locating and also publish the next waypoint.
// For details on the algorithm, you can visit our project's Gitlab page 
// https://gitlab.lrz.de/ge23ged/autonomous-systems-2021-group-terminus/-/tree/main/project

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <map>
#include <string.h>
#include <geometry_msgs/Point.h>
#include <victim_signal_gen/SignalArray.h>
#include <victim_signal_gen/VictimSignal.h>

#include <planning.h>

class planning_node{
    ros::NodeHandle nh;
    ros::Subscriber uav_pos;                                        //current position subscriber
    ros::Subscriber victim_signal_strength;                         //victim signal subscriber
    ros::Publisher next_waypoint_pub;                               //next waypoint publisher

    ros::Time start;
    ros::Timer planning_Timer;

    Eigen::Vector3d cur_pos, cur_v;                                 //current position and velocity of UAV in the world frame
    std::vector<int> already_located;                               //IDs of already rescued victims
    std::vector<victim_signal_gen::VictimSignal> received_signal;   //received signal in high noise range after deleting already rescued victims

    double hz;                                                      //frequency of the waypoint publishing

    char mission, last_mission;                                    //flag in the planningloop represents the state of the algorithm
    int observe_item, direction_item, collect_item = 0;            //item of current goal observe point
    int cur_goal_ID = -1;
    int max_ID;                                                     //ID of current being searched victim and the victim with strongest signal
    int num_observe_point;                                          //number of the observe points
    double max_strength;                                            //max strength of the signal in the high noise range
    double last_strength;                                           //last signal strength for comparision

    std::vector <Eigen::Vector3d> observe_points;                    //points which UAV should visit at the beginning
    std::vector <int> ID_list;                                       //list of ID whose signal strength should be collected to cal_pos
    std::vector <int> observe_list;                                   //list of ID in the observe point;
    std::map <int,std::vector<signal_info>> collected_data_set;     //dict between the victim id and its signal set
    
    std::vector<Eigen::Vector3d> victims_pos;                       //stores the victim location

    double increased_signal[4] = {0};                               //stores the increased signal strength
    Eigen::Vector3d direction;                                      //direction of next waypoint
    Eigen::Vector3d next_waypoint, next_v, last_next_waypoint;        //stores the current goal point and its v
    Eigen::Vector3d direction_test_point[4], collect_point[6];       //stores the coordinates of the directional test/signal collect points       

    int count = 0;                                                    //counter for number of planning loop
  
    std::vector <int> collect_id;
    std::vector <double> collect_strength;

public:
    
    planning_node():hz(15.0),mission('s'), observe_item(0), direction_item(0),max_strength(0.0),last_strength(0.0){
        uav_pos = nh.subscribe("current_state", 100, &planning_node::onCurrentState, this);
        victim_signal_strength = nh.subscribe("victims_signal_strength", 100, &planning_node::onReceivedSignal, this);
        start = ros::Time::now();
        planning_Timer = nh.createTimer(ros::Duration(1/hz), &planning_node::planningLoop, this);
        //next_waypoint_pub = nh.advertise<geometry_msgs::Point>("next_waypoint",1);
        next_waypoint_pub = nh.advertise<nav_msgs::Odometry>("next_waypoint",1);
        
        //declare observe points, which can cover all area based on the sensor capability assumption
        Eigen::Vector3d observe_point_0;
        Eigen::Vector3d observe_point_1;
        Eigen::Vector3d observe_point_2;
        observe_point_0 << 30.0, 35.0, 20.0;
        observe_point_1 << -20.0, 35.0, 20.0;
        observe_point_2 << -70.0, 35.0, 20.0;
        observe_points.push_back(observe_point_0);
        observe_points.push_back(observe_point_1);
        observe_points.push_back(observe_point_2);
        num_observe_point = observe_points.size();

        ROS_INFO("[planning_node] Initialised");
    }
    
    //This callback function gets the current postion and velocity of the UAV
    void onCurrentState(const nav_msgs::Odometry& cur_state) {
        cur_pos << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
        if(cur_pos(0)<-120 || cur_pos(0)>80 || cur_pos(1)<-10 ||cur_pos(1)>80){  //if fly too far, go back to observe point
            observe_item--;
            mission = 'o';
        }

        cur_v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
    }

    //This callback function receives the signal from the victims and divided the signal with high/low noise
     void onReceivedSignal(const victim_signal_gen::SignalArray::ConstPtr& msg) {
        if(mission != 's'){
            int flag=0;
            if(msg->signal.size()>1){
                for(int i=1;i<msg->signal.size();i++){
                    const victim_signal_gen::VictimSignal &data = msg->signal[i];
                
                    if(mission != 't'){
                        //all ID not in the already_located list should be collected
                    

                        if(data.signal_strength > ACCURACY_LIMIT && !AlreadyLocated(data.ID) && !IDinList(data.ID)){
                            ID_list.push_back(data.ID);
                            flag=1;
                            if(IDinObserveList(data.ID)>=0)
                                observe_list.erase(observe_list.begin()+IDinObserveList(data.ID));
                        }

                        if(!AlreadyLocated(data.ID)){
                            if(data.signal_strength < ACCURACY_LIMIT)
                                received_signal.push_back(data);
                        }
                    }

                    if (IDinList(data.ID)){
                        collect_id.push_back(data.ID);
                        collect_strength.push_back(data.signal_strength);
                    }
                }
            }

            if(flag){
                last_mission = mission;
                last_next_waypoint = next_waypoint;
                mission = 't';
                ROS_INFO("Find victim(s), are trying to locate them!");
                create_collect_point(cur_pos);
                next_waypoint = collect_point[collect_item];
                next_v <<0,0,0;
            }

            
        }
    }


    //function which returns to the signal strength with desired ID
    double get_signal_with_desired_id(int id) {
        for (int i=0; i<received_signal.size(); i++) {
            if (received_signal[i].ID == id) {
                return received_signal[i].signal_strength;
            }
        }
    }    
        
    

    //check if the id has been located
    bool AlreadyLocated(int id){
        if (std::find(already_located.begin(), already_located.end(), id) != already_located.end())
            return true;
        else
            return false;
    }

    //check if id is in the list which its signal strength should be collected
    bool IDinList(int id){
        if ( std::find(ID_list.begin(), ID_list.end(), id) != ID_list.end() )
            return true;
        else
            return false;
    }

    //check if id is in the observe list and if so return the positon
    int IDinObserveList(int id){
        int n = observe_list.size();
        if (n==0) 
            return -1; 
        else{
            for(int i=0;i<n;i++)
                if (observe_list[i] == id)
                    return i; //return the postion
        }
        return -1; //not found 
    }   

    //create fout test point around current postion in 4 direction
    void create_test_point(Eigen::Vector3d cur_pos){
        direction_item = 0;
        Eigen::Vector3d v1, v2;
        v1 << 2,0,0;
        v2 << 0,2,0;
        direction<<0.0,0.0,0.0;
        direction_test_point[0] = cur_pos + 2.5*v1;
        direction_test_point[1] = cur_pos - 2.5*v1;
        direction_test_point[2] = cur_pos + 2.5*v2;
        direction_test_point[3] = cur_pos - 2.5*v2;
    }

    //estimate the direction where the signal_strength would inrease most rapidly
    void cal_direction(int flag){
        if(flag==2){ //+y
            if(increased_signal[0]>0) //+x +y
                direction<<increased_signal[0],increased_signal[2],0; 
            else //-x +y
                direction<<-increased_signal[1],increased_signal[2],0;
        }
        if(flag==3){ //-y
            if(increased_signal[0]>0) //+x/-y
                direction<<increased_signal[0],-increased_signal[3],0;
            else //-x, -y
                direction<<-increased_signal[1],-increased_signal[3],0;
        }

        memset(increased_signal,0,sizeof(increased_signal));
        memset(direction_test_point,0,sizeof(direction_test_point));
        double estimated_distance = pow((pow((10000/max_strength+0.9),2)-pow(14,2)),0.5);
       
        next_waypoint = cur_pos + estimated_distance*direction.normalized();
        next_v <<0,0,0;
        mission = 'c';
    }

    //to determine the next goal (ID with max strength (in observe list) in high noise range)
    void refresh(){
        max_strength = 0;
        if(observe_list.empty()){
                mission  = 'o';
                next_waypoint = observe_points[observe_item];
        }
        else{
            for(int i=0;i<received_signal.size();i++){
                if(IDinObserveList(received_signal[i].ID)>=0) //find the current max signal id in observe list in current postion
                    if(received_signal[i].signal_strength>max_strength){
                        cur_goal_ID = received_signal[i].ID;
                        max_strength = received_signal[i].signal_strength;
                    }
            }
            if(max_strength){ // this means that ID in observe list has benn detected and cur_goal_ID has been determined
                create_test_point(cur_pos);
                next_waypoint = direction_test_point[direction_item];  
                next_v <<0,0,0;
                mission = 'd'; // try to determine the direction
            }else{ //if none of the signal in observe list can be detected, then go back to last observe point
                observe_item--;
                next_waypoint = observe_points[observe_item];
                next_v << 0,0,0;
                mission = 'o';
            }
        }

    }
    
    //create postion for collecting singal data for least square method
    void(create_collect_point(Eigen::Vector3d cur_pos)){
        Eigen::Vector3d v1, v2, v3;
        v1 << 1.5,0,0;
        v2 << 0,1.5,0;
        v3 << 0,0,1.5;
        collect_item = 0;
        collect_point[0] = cur_pos + v2;
        collect_point[1] = cur_pos + v1 + v2;
        collect_point[2] = cur_pos + v1 + v2 - v3;
        collect_point[3] = cur_pos + v1 - v3;
        collect_point[4] = cur_pos - v3;
        collect_point[5] = cur_pos;
    }

    // this filter function provide publish waypoint outside area
    void filter_waypoint(){
        if(next_waypoint(0) < -115)
            next_waypoint(0)=-115;
        if(next_waypoint(0) > 75 )
            next_waypoint(0) = 75;
        if(next_waypoint(1) <-5)
            next_waypoint(1) = -5;
        if(next_waypoint(1) > 75)
            next_waypoint(1) = 75;
    }

    //please refer to our paper to understand this loop
    void planningLoop(const ros::TimerEvent& t) {
        nav_msgs::Odometry waypoint;

       // ROS_INFO_STREAM("current mission" << mission <<" "<< cur_goal_ID <<"\n");

        switch(mission){
            case 's':  // start: to set the UAV to the desired attitude
                if(count<10){
                    next_waypoint << 0,0,10;
                    next_v <<0,0,2;
                }
                if(count>15){
                    next_waypoint << 0,0,20;
                    next_v <<0,0,0;
                }
                if (count > 20 && check_pos(cur_pos,next_waypoint)){
                    mission = 'o';
                    count = -1; //reset counter
                    ROS_INFO_ONCE("Ready to departure");
                }
                count++;

                break;
        
            case 'o': //to fly to observe point

                if(observe_item == num_observe_point){ 
                    mission = 'e';
                    next_waypoint << 0,0,2; 
                    next_v <<0,0,0;
                }
                else{
                    if(check_pos(cur_pos,observe_points[observe_item])){
                        if(observe_item<num_observe_point)
                            observe_item++;
                        if(received_signal.size()==0){ //this means no victims around this observe point
                            next_waypoint = observe_points[observe_item];
                            break;
                        }
                        for(int i=0;i<received_signal.size();i++){
                            if(IDinObserveList(received_signal[i].ID == -1))
                                observe_list.push_back(received_signal[i].ID);   //push the signal id which can be detected in current observe point
                            if(received_signal[i].signal_strength>max_strength){
                                cur_goal_ID = received_signal[i].ID;
                                max_strength = received_signal[i].signal_strength;
                            }
                        }
                        create_test_point(cur_pos);
                        next_waypoint = direction_test_point[direction_item];  
                        next_v <<0,0,0; 
                        mission = 'd'; // try to determine the direction
                    }
                    else{
                        //if not arrive at desired pos, keep trying fly there
                        next_waypoint = observe_points[observe_item]; 
                        next_v <<0,0,0;
                    }
                }
                break;

            case 'd': //to get the direction
                if(IDinObserveList(cur_goal_ID)>=0){
                    if(check_pos(cur_pos,next_waypoint)){
                        double goalID_signal_strength = get_signal_with_desired_id(cur_goal_ID);
                        switch(direction_item){
                            case 0: //+x direction
                                if(goalID_signal_strength > max_strength){
                                    increased_signal[direction_item] = goalID_signal_strength - max_strength;
                                    direction_item = 2;
                                    next_waypoint = direction_test_point[direction_item];
                                    next_v <<0,0,0;
                                }else{
                                    direction_item++;
                                    next_waypoint = direction_test_point[direction_item]; 
                                    next_v <<0,0,0;
                                }
                                break;

                            case 1: //-x direction
                                increased_signal[direction_item] = abs(goalID_signal_strength - max_strength);
                                direction_item++;
                                next_waypoint = direction_test_point[direction_item]; 
                                next_v <<0,0,0;
                                break;
                            
                            case 2: //+y direction
                                if(goalID_signal_strength > max_strength){
                                    increased_signal[direction_item] = goalID_signal_strength - max_strength;
                                    cal_direction(2);
                                }
                                else{
                                    direction_item++;
                                    next_waypoint = direction_test_point[direction_item]; 
                                    next_v <<0,0,0;
                                }
                                break;

                            case 3: //-y direction
                                increased_signal[direction_item] = abs(goalID_signal_strength -max_strength);
                                cal_direction(3);
                                break;
                        }
                    }
                }
                else{
                    refresh();
                }
                break;


            case 'c': 
                if(IDinObserveList(cur_goal_ID)>=0){
                    if(check_pos(cur_pos,next_waypoint)){
                        count=0;
                        create_test_point(cur_pos);
                        mission = 'd';
                        next_waypoint = direction_test_point[0];
                        next_v << 0,0,0;
                    }
                    else{
                        if(count == 100){
                            double  goalID_strength = get_signal_with_desired_id(cur_goal_ID);
                            if(goalID_strength < max_strength){
                                create_test_point(cur_pos);
                                mission = 'd';
                                next_waypoint = direction_test_point[0];
                                next_v << 0,0,0;
                            }
                            count = 0;
                            max_strength = goalID_strength;
                        }else{
                            count++;
                        }

                    }
                }else{
                    refresh();
                }
                break;
        
         
            case 't':
                if(check_pos(cur_pos,next_waypoint)){
                    for(int i=0;i<collect_id.size();i++){
                        signal_info info={cur_pos,collect_strength[i]};
                        //std::cout<<collect_item<<std::endl;
                        collected_data_set[collect_id[i]].push_back(info);
                    }
                    collect_item++;
                    next_waypoint = collect_point[collect_item];
                    next_v << 0,0,0;
                }
                else{
                    next_waypoint = collect_point[collect_item]; 
                    next_v <<0,0,0;
                }
                collect_id.clear();
                collect_strength.clear();

                if (collect_item > 5){ 
                    int i=0;
                    while(!ID_list.empty()){
                        //observe_list.erase(observe_list.begin()+IDinObserveList(ID_list[i]));
                        Eigen::Vector3d position;
                        position = cal_pos(collected_data_set[ID_list[i]]);
                        victims_pos.push_back(position);

                        ROS_WARN_STREAM("Locate a victim!!! pos: "<<position(0)<<" "<<position(1)<<" "<<position(2)<<"\n");

                        already_located.push_back(ID_list[i]);
                        ID_list.erase(ID_list.begin()+i);
                    }  
                    mission = last_mission; 
                    next_waypoint = last_next_waypoint;
                    next_v << 0,0,0;
                    memset(collect_point,0,sizeof(collect_point));             
                }
                break;

            case 'e': //end
                ROS_INFO("Finished!!! Please wait the UAV to go back.\n");
                ROS_INFO("%d Victims have been found.\n Their locations are: \n",victims_pos.size());
                for (int i=0;i<victims_pos.size();i++)
                    ROS_INFO("(%f, %f, %f)\n", victims_pos[i](0),victims_pos[i](1),victims_pos[i](2));
                ROS_INFO("The rescuers are ready to departure");
                mission = 'b';
            break;

            case 'b':
                break;
        }

        received_signal.clear();
        waypoint.pose.pose.position.x = next_waypoint(0);
        waypoint.pose.pose.position.y = next_waypoint(1);
        waypoint.pose.pose.position.z = next_waypoint(2);
        waypoint.twist.twist.linear.x = next_v(0);
        waypoint.twist.twist.linear.y = next_v(1);
        waypoint.twist.twist.linear.z = next_v(2);

        filter_waypoint();

        if (waypoint.pose.pose.position.z >=2 && waypoint.pose.pose.position.z <=25) //avoid publish some unrealistic wayoints
            next_waypoint_pub.publish(waypoint); 
    }
};

//check if the UAV has arrived our desired point.
bool check_pos(Eigen::Vector3d &cur_pos, Eigen::Vector3d &desired_pos){
    if((cur_pos-desired_pos).norm()<=0.1)
        return true;
    else
        return false;
}


//This function calucate the postion of the victims based on the signal with low noise
Eigen::Vector3d cal_pos(const std::vector<signal_info> &signal){
    int n = signal.size(); 

    Eigen::MatrixXd B(n,4);
    Eigen::VectorXd l(n);
    for(int i=0;i<n;i++){
        for(int j=0; j<3;j++){
            B(i,j) = -2*signal[i].pos(j);
            double d = 10000.0 / signal[i].signal_strength - 0.9;
            l(i) = pow(d,2) - pow(signal[i].pos.norm(),2);
        }
        B(i,3)=1; 
    }
    Eigen::Vector4d sol = (B.transpose()*B).inverse()*(B.transpose())*(l);  //ordinary least squares
    return sol.head(3);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "planning_node");
    planning_node n;
    ros::spin();
}
