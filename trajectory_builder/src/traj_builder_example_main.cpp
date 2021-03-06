#include <trajectory_builder/traj_builder.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>


geometry_msgs::Twist g_halt_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;

void init(){
  g_halt_twist.linear.x = 0.0;
    g_halt_twist.linear.y = 0.0;
    g_halt_twist.linear.z = 0.0;
    g_halt_twist.angular.x = 0.0;
    g_halt_twist.angular.y = 0.0;
    g_halt_twist.angular.z = 0.0;

    //default values; can be overridden
    g_start_state.twist.twist = g_halt_twist;
    g_start_state.pose.pose.position.x = 0;
    g_start_state.pose.pose.position.y = 0;
    g_start_state.pose.pose.position.z = 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "des_state_publisher");ros::NodeHandle n;
    double dt = 0.02;

    ros::Publisher des_state_publisher = n.advertise<nav_msgs::Odometry>("/desState", 1);
    ros::Publisher des_psi_publisher = n.advertise<std_msgs::Float64>("/desPsi", 1);

    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate looprate(1 / dt);

    TrajBuilder trajBuilder;
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max_(1.0);

    double psi_start = 0.0;
    double psi_end = 0.0;

    g_start_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);
    g_end_state = g_start_state;
    g_end_state.pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);

    g_start_pose.pose.position.x = 0.0;
    g_start_pose.pose.position.y = 0.0;
    g_start_pose.pose.position.z = 0.0;
    g_start_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_start);

    g_end_pose.pose.orientation = trajBuilder.convertPlanarPsi2Quaternion(psi_end);

    g_end_pose.pose.position.x = 2.0;

    g_end_pose.pose.position.y = -4.0;

    double des_psi;
    std_msgs::Float64 psi_msg;
    std::vector<nav_msgs::Odometry> vec_states;

    nav_msgs::Odometry des_state;
    nav_msgs::Odometry last_state;
    geometry_msgs::PoseStamped last_pose;


     while (ros::ok()) {
       ROS_INFO("building traj from start to end");
       trajBuilder.build_point_and_go_traj(g_start_pose, g_end_pose, vec_states);
       ROS_INFO("publishing desired states and open-loop cmd_vel");

       for (int i = 0; i < vec_states.size(); i++){
         des_state = vec_states[i];
         des_state.header.stamp = ros::Time::now();
          des_state_publisher.publish(des_state);
          des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
          psi_msg.data = des_psi;
          des_psi_publisher.publish(psi_msg);
          twist_commander.publish(des_state.twist.twist);
          looprate.sleep();

       }
       ROS_INFO("building traj from end to start");
       last_state = vec_states.back();
       last_pose.header = last_state.header;
       last_pose.pose = last_state.pose.pose;
       trajBuilder.build_point_and_go_traj(last_pose, g_start_pose, vec_states);
       for (int i = 0; i < vec_states.size(); i++) {
            des_state = vec_states[i];
            des_state.header.stamp = ros::Time::now();
            des_state_publisher.publish(des_state);
            des_psi = trajBuilder.convertPlanarQuat2Psi(des_state.pose.pose.orientation);
            psi_msg.data = des_psi;
            des_psi_publisher.publish(psi_msg);
            twist_commander.publish(des_state.twist.twist); //FOR OPEN-LOOP CTL ONLY!
            looprate.sleep(); //sleep for defined sample period, then do loop again
        }
        last_state = vec_states.back();
        g_start_pose.header = last_state.header;
        g_start_pose.pose = last_state.pose.pose;

     }


}
