#include <trajectory_builder/traj_builder.h>

TrajBuilder::TrajBuilder(){
  dt_ = default_dt;
  accel_max_ = accel_max;
  alpha_max_ = alpha_max;
  speed_max_ = speed_max;
  omega_max_ = omega_max;
  path_move_tol_ = path_move_tol;

  // halt vec_states

  halt_twist_.linear.x = 0.0;
  halt_twist_.linear.y = 0.0;
  halt_twist_.linear.z = 0.0;
  halt_twist_.angular.x = 0.0;
  halt_twist_.angular.y = 0.0;
  halt_twist_.angular.z = 0.0;


}

double TrajBuilder::min_dang(double dang){
  while(dang > M_PI)
  dang -= 2.0*M_PI;

  while(dang>M_PI)
  dang+=2.0*M_PI;

  return dang;
}

double TrajBuilder::sat(double x){
  if (x>1.0)
  return 1.0;

  if (x < -1.0)
  return -1.0;
  return x;
}

double TrajBuilder::sgn(double x){
  if (x<0.0)
  return -1.0;

  if (x > 0.0)
  return 1.0;

  return 0.0;
}

double TrajBuilder::convertPlanarQuat2Psi(geometry_msgs::Quaternion quaternion){
  double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double psi = 2.0 * atan2(quat_z, quat_w);
}
geometry_msgs::Quaternion TrajBuilder::convertPlanarPsi2Quaternion(double psi){
  geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(psi / 2.0);
    quaternion.w = cos(psi / 2.0);
    return (quaternion);
}
geometry_msgs::PoseStamped TrajBuilder::xyPsi2PoseStamped(double x, double y, double psi){
  geometry_msgs::PoseStamped poseStamped; // a pose object to populate
    poseStamped.pose.orientation = convertPlanarPsi2Quaternion(psi); // convert from heading to corresponding quaternion
    poseStamped.pose.position.x = x;
    poseStamped.pose.position.y = y;
    poseStamped.pose.position.z = 0.0; // keep the robot on the ground!
    return poseStamped;

}

void TrajBuilder::build_trapezoidal_spin_traj(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped end_pose,
std::vector<nav_msgs::Odometry>& vec_states){
  double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    double t_ramp = omega_max_/ alpha_max_;
    double ramp_up_dist = 0.5 * alpha_max_ * t_ramp*t_ramp;
    double cruise_distance = fabs(dpsi) - 2.0 * ramp_up_dist; //delta-angle to spin at omega_max
    int npts_ramp = round(t_ramp / dt_);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header;
    des_state.pose.pose = start_pose.pose;
    des_state.twist.twist = halt_twist_;

    //ramp up
    double t = 0.0;
    double accel = sgn(dpsi)*alpha_max_;
    double omega_des = 0.0;
    double psi_des = psi_start;

    for(int i{0}; i<npts_ramp;i++){
      t+=dt_;
      omega_des=accel*t;
      des_state.twist.twist.angular.z = omega_des;
      psi_des = psi_start + 0.5*accel*t*t;
      des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
      vec_states.push_back(des_state);
    }

    //cruise mode - constant velocity

    omega_des = sgn(dpsi)*omega_max_;

    des_state.twist.twist.angular.z = omega_des;

    double t_cruise = cruise_distance/omega_max_;
    int npts_cruise = round(t_cruise/dt_;);

    for(int i{0}; i<npts_cruise; i++){
      psi_des += omega_des*dt_;
      des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
      vec_states.push_back(des_state;)
    }

    //ramp down

    for(int i{0}; i<npts_ramp; i++){
      omega_des -= accel*dt_;
      des_state.twist.twist.angular.z = omega_des;
      psi_des += omega_des*dt_;
      des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
      vec_states.push_back(des_state);
    }

    des_state.pose.pose = end_pose.pose;
    des_state.twist = halt_twist_;
    vec_states.push.back(des_state);


}


void TrajBuilder::build_triangular_spin_traj(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped end_pose,
std::vector<nav_msgs::Odometry>& vec_states){
  nav_msgs::Odometry des_state;
  des_state.header = start_pose.header;
  des_state.pose.pose = start_pose.pose;
  des_state.twist.twist = halt_twist_;
  vec_states.push_back(des_state);

  double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO("spin traj: psi_start = %f; psi_end = %f; dpsi= %f", psi_start, psi_end, dpsi);
     double t_ramp = sqrt(fabs(dpsi) / alpha_max_);
     int npts_ramp = round(t_ramp / dt_);
     double psi_des = psi_start;
     double omega_des = 0.0;
     double t=0.0;
     double accel = sgn(dpsi) * alpha_max_;

     for(int i=0; i<npts_ramp; i++){
       t+=dt_;
       omega_des = accel*t;
       des_state.twist.twist.angular.z = omega_des;
       psi_des = psi_start + 0.5*accel*t*t;
       des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
       vec_states.push_back(des_state);
     }

     //ramp down
     for(int i=0;i<npts_ramp;i++){
       omega_des-=accel*dt_;

       des_state.twist.twist.angular.z = omega_des;

       psi_des += omega_des*dt_;
       des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);

       vec_states.push_back(des_state);
     }

     des_state.pose.pose = end_pose.pose;
     des_state.twist.twist = halt_twist_;
     vec_states.push_back(des_state);
}

void TrajBuilder::build_spin_traj(geometry_msgs::PoseStamped start_pose,
          geometry_msgs::PoseStamped end_pose,
          std::vector<nav_msgs::Odometry> &vec_states){
            double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convertPlanarQuat2Psi(start_pose.pose.orientation);
    double psi_end = convertPlanarQuat2Psi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO("rotational spin distance = %f", dpsi);
    double ramp_up_time = omega_max_/ alpha_max_;
    double ramp_up_dist = 0.5 * alpha_max_ * ramp_up_time*ramp_up_time;

    if (fabs(dpsi) < 2.0 * ramp_up_dist) { //delta-angle is too short for trapezoid
        build_triangular_spin_traj(start_pose, end_pose, vec_of_states);
    } else {
        build_trapezoidal_spin_traj(start_pose, end_pose, vec_of_states);
    }
          }

  void TrajBuilder::build_travel_traj(geometry_msgs::PoseStamped start_pose,
                  geometry_msgs::PoseStamped end_pose,
                  std::vector<nav_msgs::Odometry> &vec_states) {
              //decide if triangular or trapezoidal profile:
              double x_start = start_pose.pose.position.x;
              double y_start = start_pose.pose.position.y;
              double x_end = end_pose.pose.position.x;
              double y_end = end_pose.pose.position.y;
              double dx = x_end - x_start;
              double dy = y_end - y_start;
              double trip_len = sqrt(dx * dx + dy * dy);
              double ramp_up_dist = 0.5 * speed_max_ * speed_max_ / alpha_max_;
              ROS_INFO("trip len = %f", trip_len);
              if (trip_len < 2.0 * ramp_up_dist) { //length is too short for trapezoid
                  build_triangular_travel_traj(start_pose, end_pose, vec_of_states);
              } else {
                  build_trapezoidal_travel_traj(start_pose, end_pose, vec_of_states);
              }
          }

          void TrajBuilder::build_trapezoidal_travel_traj(geometry_msgs::PoseStamped start_pose,
                geometry_msgs::PoseStamped end_pose,
                std::vector<nav_msgs::Odometry> &vec_states) {
            double x_start = start_pose.pose.position.x;
            double y_start = start_pose.pose.position.y;
            double x_end = end_pose.pose.position.x;
            double y_end = end_pose.pose.position.y;
            double dx = x_end - x_start;
            double dy = y_end - y_start;
            double psi_des = atan2(dy, dx);
            double trip_len = sqrt(dx * dx + dy * dy);
            double t_ramp = speed_max_ / accel_max_;
            double ramp_up_dist = 0.5 * accel_max_ * t_ramp*t_ramp;
            double cruise_distance = trip_len - 2.0 * ramp_up_dist; //distance to travel at v_max
            ROS_INFO("t_ramp =%f",t_ramp);
            ROS_INFO("ramp-up dist = %f",ramp_up_dist);
            ROS_INFO("cruise distance = %f",cruise_distance);
            //start ramping up:
            nav_msgs::Odometry des_state;
            des_state.header = start_pose.header; //really, want to copy the frame_id
            des_state.pose.pose = start_pose.pose; //start from here
            des_state.twist.twist = halt_twist_; // insist on starting from rest
            int npts_ramp = round(t_ramp / dt_);
            double x_des = x_start; //start from here
            double y_des = y_start;
            double speed_des = 0.0;
            des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
            des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
            // orientation of des_state will not change; only position and twist

            double t = 0.0;
            //ramp up;
            for (int i = 0; i < npts_ramp; i++) {
                t += dt_;
                speed_des = accel_max_*t;
                des_state.twist.twist.linear.x = speed_des; //update speed
                //update positions
                x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
                y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
                des_state.pose.pose.position.x = x_des;
                des_state.pose.pose.position.y = y_des;
                vec_states.push_back(des_state);
            }
            //now cruise for distance cruise_distance at const speed
            speed_des = speed_max_;
            des_state.twist.twist.linear.x = speed_des;
            double t_cruise = cruise_distance / speed_max_;
            int npts_cruise = round(t_cruise / dt_);
            ROS_INFO("t_cruise = %f; npts_cruise = %d",t_cruise,npts_cruise);
            for (int i = 0; i < npts_cruise; i++) {
                //Euler one-step integration
                x_des += speed_des * dt_ * cos(psi_des);
                y_des += speed_des * dt_ * sin(psi_des);
                des_state.pose.pose.position.x = x_des;
                des_state.pose.pose.position.y = y_des;
                vec_states.push_back(des_state);
            }
            //ramp down:
            for (int i = 0; i < npts_ramp; i++) {
                speed_des -= accel_max_*dt_; //Euler one-step integration
                des_state.twist.twist.linear.x = speed_des;
                x_des += speed_des * dt_ * cos(psi_des); //Euler one-step integration
                y_des += speed_des * dt_ * sin(psi_des); //Euler one-step integration
                des_state.pose.pose.position.x = x_des;
                des_state.pose.pose.position.y = y_des;
                vec_states.push_back(des_state);
            }
            //make sure the last state is precisely where requested, and at rest:
            des_state.pose.pose = end_pose.pose;
            //but final orientation will follow from point-and-go direction
            des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
            des_state.twist.twist = halt_twist_; // insist on starting from rest
            vec_states.push_back(des_state);
        }

        void TrajBuilder::build_triangular_travel_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_states) {
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    nav_msgs::Odometry des_state;
    des_state.header = start_pose.header; //really, want to copy the frame_id
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = sqrt(trip_len / accel_max_);
    int npts_ramp = round(t_ramp / dt_);
    double v_peak = accel_max_*t_ramp; // could consider special cases for reverse motion
    double d_vel = alpha_max_*dt_; // incremental velocity changes for ramp-up

    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist
    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += dt_;
        speed_des = accel_max_*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * accel_max_ * t * t * cos(psi_des);
        y_des = y_start + 0.5 * accel_max_ * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= accel_max_*dt_; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * dt_ * cos(psi_des); //Euler one-step integration
        y_des += speed_des * dt_ * sin(psi_des); //Euler one-step integration
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convertPlanarPsi2Quaternion(psi_des);
    des_state.twist.twist = halt_twist_; // insist on starting from rest
    vec_states.push_back(des_state);
}

void TrajBuilder::build_braking_traj(geometry_msgs::PoseStamped start_pose,
        std::vector<nav_msgs::Odometry> &vec_states) {

        double decel_smooth = 0.05;  // a small deceleration value for smooth halt
        nav_msgs::Odometry des_state;
        des_state.header = start_pose.header; //really, want to copy the frame_id
        des_state.pose.pose = start_pose.pose; //start from here
        des_state.twist.twist = vec_states.back().twist.twist;
        double x_start = start_pose.pose.position.x;
        double y_start = start_pose.pose.position.y;
        double psi_des = convertPlanarQuat2Psi(start_pose.pose.orientation);

        double cur_speed = fabs(des_state.twist.twist.linear.x);

        while(cur_speed > 0.001){
          cur_speed-=decel_smooth*dt_;
          des_state.twist.twist.linear.x = cur_speed;

          x_des += speed_des * dt_ * cos(psi_des);
          y_des += speed_des * dt_ * sin(psi_des);
          des_state.pose.pose.position.x = x_des;
          des_state.pose.pose.position.y = y_des;

          vec_states.push_back(des_state);
        }

        des_state.twist.twist = halt_twist_;
        vec_states.push_back(des_state);










    //FINISH ME!

}

void TrajBuilder::build_point_and_go_traj(geometry_msgs::PoseStamped start_pose,
        geometry_msgs::PoseStamped end_pose,
        std::vector<nav_msgs::Odometry> &vec_states){
          ROS_INFO("building point-and-go trajectory");
          nav_msgs::Odometry bridge_state;
          geometry_msgs::PoseStamped bridge_pose;
          vec_states.clear();
           ROS_INFO("building rotational trajectory");
           double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double des_psi = atan2(dy, dx);
    ROS_INFO("desired heading to subgoal = %f", des_psi);
    bridge_pose = start_pose;
    bridge_pose.pose.orientation = convertPlanarQuat2Psi(des_psi);
    build_spin_traj(start_pose, bridge_pose,vec_states);
    build_travel_traj(bridge_pose, end_pose, vec_states);
        }
