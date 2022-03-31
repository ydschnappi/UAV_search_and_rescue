#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PolygonStamped.h>
#include <vector>
#include <map>
#include <math.h>

typedef struct point{
    float x;
    float y;
    float z;
}point;

class victimestimate{
    ros::NodeHandle n;
    ros::Subscriber current_state;

    std::vector<point> points;
    point point_v;

    std::map<int,int> if_found;

    public:
        victimestimate(){
            
            for(int i =0;i<8;i++){
                point_v.x = (rand()%(50-0))+0;
                point_v.y = (rand()%(50-0))+0;
                point_v.z = 10;
                points.push_back(point_v);
            }

            
            for(int i=0;i<points.size();i++){
                if_found[i]=0;
            }
            current_state = n.subscribe("current_state",10,&victimestimate::victim_judge,this);
        }

        void victim_judge(const nav_msgs::Odometry& cur_state){
            float distance_x;
            float distance_y;
            float distance_z;
            float distance;

            for(int i =0; i <points.size();++i){
                distance_x = pow((cur_state.pose.pose.position.x - points[i].x),2);
                distance_y = pow((cur_state.pose.pose.position.y - points[i].y),2);
                distance_z = pow((cur_state.pose.pose.position.z - points[i].z),2);
                distance = sqrt(distance_x + distance_y);
                if(distance < 10 && if_found[i]==0){
                    std::cout <<"Found Victim id:"<< i <<" coordinate: "<<points[i].x << "," << points[i].y << "," << points[i].z <<"\n";
                    if_found[i] = 1;
                }
            }
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "victim");
    victimestimate v;
    ros::spin();
}

