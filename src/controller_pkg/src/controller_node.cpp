#include <ros/ros.h>

#include <ros/console.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define PI M_PI


//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a C++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf provides useful functions to convert tf 
// messages to eigen types and vice versa, have a look to the documentation:
// http://docs.ros.org/melodic/api/eigen_conversions/html/namespacetf.html
#include <eigen_conversions/eigen_msg.h>



class controllerNode{
  ros::NodeHandle nh;


  ros::Subscriber desired_state, current_state;
  ros::Publisher prop_speeds;
  ros::Timer timer;


  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop


  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){


      
      desired_state = nh.subscribe("command/trajectory", 1, &controllerNode::onDesiredState, this);
      current_state = nh.subscribe("current_state", 1, &controllerNode::onCurrentState, this);
      prop_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

 

      kx = 12.7;
      kv = 5.8;
      kr = 8.8;
      komega = 1.15;

      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectory& des_state){

      // Position
      geometry_msgs::Vector3 t = des_state.points[0].transforms[0].translation;
      xd << t.x, t.y, t.z;
      // ROS_INFO_NAMED("onDesiredState", "POS: %f %f %f", t.x, t.y, t.z);

      // Velocities
      geometry_msgs::Vector3 v = des_state.points[0].velocities[0].linear;
      vd << v.x, v.y, v.z;
      // ROS_INFO_NAMED("onDesiredState", "VEL: %f %f %f", v.x, v.y, v.z);

      // Accelerations
      geometry_msgs::Vector3 a = des_state.points[0].accelerations[0].linear;
      ad << a.x, a.y, a.z;
      // ROS_INFO_NAMED("onDesiredState", "ACC: %f %f %f", a.x, a.y, a.z);

      tf::Quaternion q;
      tf::quaternionMsgToTF(des_state.points[0].transforms[0].rotation , q);
      yawd = tf::getYaw(q);
      if (v.x == 0) {
        yawd = 0;
      }
      else {
      yawd = atan2(v.y, v.x);
      }
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
    

      
      x << cur_state.pose.pose.position.x,cur_state.pose.pose.position.y,cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x,cur_state.twist.twist.linear.y,cur_state.twist.twist.linear.z;
      omega << cur_state.twist.twist.angular.x,cur_state.twist.twist.angular.y,cur_state.twist.twist.angular.z;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen (cur_state.pose.pose.orientation, q);
      R = q.toRotationMatrix();

      // Rotate omega
      omega = R.transpose()*omega;

   
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;

    
    ex = x - xd;
    ev = v - vd;



  
    
    Eigen::Vector3d b_3d = -kx*ex - kv*ev + m*g*e3 + m*ad;
    b_3d.normalize();

    Eigen::Vector3d b_1d(cos(yawd), sin(yawd), 0);
    Eigen::Vector3d b_2d = b_3d.cross(b_1d);
    b_2d.normalize();
    Eigen::Vector3d b_1d_true(b_2d.cross(b_3d));
    b_1d_true.normalize();

    Eigen::Matrix3d Rd;
    Rd << b_1d_true, b_2d, b_3d;



    
    er = 0.5 * Vee(Rd.transpose()*R - R.transpose()*Rd);
    eomega = omega;


    
    Eigen::Vector3d errs = -kx*ex - kv*ev + m*g*e3 + m*ad;
    double f = errs.dot(R*e3);
    Eigen::Vector3d torques = -kr*er -komega*eomega + omega.cross(J*omega);

    Eigen::Vector4d wrench(f, torques.x(), torques.y(), torques.z());



    

    
    double d_hat = d/sqrt(2);
    Eigen::Matrix4d F;

    F << cf, cf, cf, cf, 
        cf*d_hat, cf*d_hat, -cf*d_hat, -cf*d_hat, 
        -cf*d_hat, cf*d_hat, cf*d_hat, -cf*d_hat,
        cd, -cd, cd, -cd;

  
    
    Eigen::Vector4d props = F.inverse()*wrench;

    mav_msgs::Actuators msg;
    msg.angular_velocities.resize(4);
    msg.angular_velocities[0] = signed_sqrt(props[0]);
    msg.angular_velocities[1] = signed_sqrt(props[1]);
    msg.angular_velocities[2] = signed_sqrt(props[2]);
    msg.angular_velocities[3] = signed_sqrt(props[3]);

    // ROS_INFO_NAMED("controller", "rps: %f %f %f %f", msg.angular_velocities[0], msg.angular_velocities[1], msg.angular_velocities[2], msg.angular_velocities[3]);

    prop_speeds.publish(msg);
   
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode n;
  ros::spin();
}
