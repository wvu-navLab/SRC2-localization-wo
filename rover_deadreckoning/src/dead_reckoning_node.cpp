#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#define PI 3.14159265359

// Declare sensor data variables
double fl_wheel_turn_counts = 0.0;
double fl_wheel_turn_counts_prev = 0.0;
double fl_wheel_steer = 0.0;
double fr_wheel_steer = 0.0;
double bl_wheel_steer = 0.0;
double br_wheel_steer = 0.0;
double fl_wheel_vel = 0.0;
double bl_wheel_turn_counts = 0.0;
double bl_wheel_turn_counts_prev = 0.0;
double bl_wheel_vel = 0.0;
double fr_wheel_turn_counts = 0.0;
double fr_wheel_turn_counts_prev = 0.0;
double fr_delta_counts = 0.0;
double fr_wheel_vel = 0.0;
double br_wheel_turn_counts = 0.0;
double br_wheel_turn_counts_prev = 0.0;
double br_wheel_vel = 0.0;
double yaw_rate = 0.0;
double yaw_rate_prev = 0.0;
double delta_yaw = 0.0;
double imu_time;
double imu_time_prev;
double yaw = 0.0;
double heading = 0.0;
double delta_x = 0.0;
double delta_y = 0.0;
double velocity_x = 0.0;
double velocity_y = 0.0;
double linear_vel = 0.0;
double front_steering_angle = 0.0;
double rear_steering_angle = 0.0;
double front_tmp = 0.0;
double rear_tmp = 0.0;
double front_linear_speed = 0.0;
double rear_linear_speed = 0.0;
double angular = 0.0;
double front_left_tmp = 0.0;
double front_right_tmp = 0.0;
double fl_speed_tmp = 0.0;
double fr_speed_tmp = 0.0;
double rear_left_tmp = 0.0;
double rear_right_tmp = 0.0;
double bl_speed_tmp = 0.0;
double br_speed_tmp = 0.0;
double wheel_diameter;          // m
double wheel_base;              // m
double steering_track;          // m
double wheel_steering_y_offset; // m
double delta_time = 0.0;
bool first_callback = true;
uint32_t seq = 0;
std::string imu_topic_name;
std::string odometry_out_topic_name;
std::string joint_state_topic_name;
std::string odometry_frame_id;
std::string odometry_child_frame_id;
std::string joint_state_frame_id;

ros::Time current_time, last_time;
ros::Publisher odom_pub;

// Declare sensor callback functions
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void jointstateCallback(const sensor_msgs::JointState::ConstPtr &msg);

int main(int argc, char **argv) {

  std::string node_name = "rover_dead_reckoning_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;


  std::string robot_name;
  ros::Subscriber imu_sub, joint_state_sub;

  // Read params from yaml file
  if (ros::param::get(robot_name + "robot_name", robot_name) == false) {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/wheel_diameter", wheel_diameter) == false) {
    ROS_FATAL("No parameter 'wheel_diameter' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/wheel_base", wheel_base) == false) {
    ROS_FATAL("No parameter 'wheel_base' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/steering_track", steering_track) == false) {
    ROS_FATAL("No parameter 'steering_track' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/wheel_steering_y_offset", wheel_steering_y_offset) == false) {
    ROS_FATAL("No parameter 'wheel_steering_y_offset' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/imu_topic_name", imu_topic_name) == false) {
    ROS_FATAL("No parameter 'imu_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/odometry_out_topic_name", odometry_out_topic_name) == false) {
    ROS_FATAL("No parameter 'odometry_out_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/joint_state_topic_name", joint_state_topic_name) == false){
    ROS_FATAL("No parameter 'joint_state_topic_name' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name + "/odometry_frame_id", odometry_frame_id) == false){
    ROS_FATAL("No parameter 'odometry_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  odometry_frame_id = robot_name + odometry_frame_id;
  if (ros::param::get(node_name + "/odometry_child_frame_id", odometry_child_frame_id) == false){
    ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  odometry_child_frame_id = odometry_child_frame_id + odometry_frame_id;
  if (ros::param::get(node_name + "/joint_state_frame_id", joint_state_frame_id) == false){
    ROS_FATAL("No parameter 'joint_state_frame_id' specified");
    ros::shutdown();
    exit(1);
  }
  joint_state_frame_id = joint_state_frame_id + odometry_frame_id;
  nav_msgs::Odometry odom_msg;

  // Initialize publishers and subscribers
  imu_sub = nh.subscribe(imu_topic_name, 1, imuCallback);
  joint_state_sub = nh.subscribe(joint_state_topic_name, 1, jointstateCallback);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odometry_out_topic_name, 1);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  delta_yaw = 0.0;
  yaw = 0.0; // initialize yaw
  velocity_x = 0.0;
  velocity_y = 0.0;
  linear_vel = 0.0;
  angular = 0.0;
  front_steering_angle = 0.0;
  rear_steering_angle = 0.0;
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  ros::spin();
  return 0;
}

void jointstateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  current_time = ros::Time::now();
  bool highTurn = false;
  int fr_wheel_joint_idx;
  int br_wheel_joint_idx;
  int fl_wheel_joint_idx;
  int bl_wheel_joint_idx;
  int fr_steering_arm_joint_idx;
  int br_steering_arm_joint_idx;
  int fl_steering_arm_joint_idx;
  int bl_steering_arm_joint_idx;
  // loop joint states
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "fr_wheel_joint") {
      fr_wheel_joint_idx = i;
    }
    if (msg->name[i] == "br_wheel_joint") {
      br_wheel_joint_idx = i;
    }
    if (msg->name[i] == "fl_wheel_joint") {
      fl_wheel_joint_idx = i;
    }
    if (msg->name[i] == "bl_wheel_joint") {
      bl_wheel_joint_idx = i;
    }
    if (msg->name[i] == "fr_steering_arm_tibia_joint") {
      fr_steering_arm_joint_idx = i;
    }
    if (msg->name[i] == "br_steering_arm_tibia_joint") {
      br_steering_arm_joint_idx = i;
    }
    if (msg->name[i] == "fl_steering_arm_tibia_joint") {
      fl_steering_arm_joint_idx = i;
    }
    if (msg->name[i] == "bl_steering_arm_tibia_joint") {
      bl_steering_arm_joint_idx = i;
    }
  }

  fr_wheel_turn_counts_prev = fr_wheel_turn_counts;
  fr_wheel_turn_counts = msg->position[fr_wheel_joint_idx];
  fr_wheel_vel = msg->velocity[fr_wheel_joint_idx];
  fr_wheel_steer = msg->position[fr_steering_arm_joint_idx];

  br_wheel_turn_counts_prev = br_wheel_turn_counts;
  br_wheel_turn_counts = msg->position[br_wheel_joint_idx];
  br_wheel_vel = msg->velocity[br_wheel_joint_idx];
  br_wheel_steer = msg->position[br_steering_arm_joint_idx];

  fl_wheel_turn_counts_prev = fl_wheel_turn_counts;
  fl_wheel_turn_counts = msg->position[fl_wheel_joint_idx];
  fl_wheel_vel = msg->velocity[fl_wheel_joint_idx];
  fl_wheel_steer = msg->position[fl_steering_arm_joint_idx];

  bl_wheel_turn_counts_prev = bl_wheel_turn_counts;
  bl_wheel_turn_counts = msg->position[bl_wheel_joint_idx];
  bl_wheel_vel = msg->velocity[bl_wheel_joint_idx];
  bl_wheel_steer = msg->position[bl_steering_arm_joint_idx];

if (std::isnan(fl_wheel_vel) || std::isnan(fr_wheel_vel) ||
    std::isnan(bl_wheel_vel) || std::isnan(br_wheel_vel)) {
  return;
}
if (std::isnan(fl_wheel_steer) || std::isnan(fr_wheel_steer) ||
    std::isnan(bl_wheel_steer) || std::isnan(br_wheel_steer)) {
  return;
}


  front_steering_angle = atan(2*tan(fl_wheel_steer)*tan(fr_wheel_steer)/(tan(fl_wheel_steer) + tan(fr_wheel_steer)));
  if (fabs(tan(fl_wheel_steer) + tan(fr_wheel_steer)) < 0.001) {
    front_steering_angle = 0;
  }

  rear_steering_angle = atan(2*tan(bl_wheel_steer)*tan(br_wheel_steer)/(tan(bl_wheel_steer) + tan(br_wheel_steer)));
  if (fabs(tan(bl_wheel_steer) + tan(br_wheel_steer)) < 0.001) {
    rear_steering_angle = 0;
  }

  front_tmp = cos(front_steering_angle) * (tan(front_steering_angle) - tan(rear_steering_angle)) / wheel_base;
  front_left_tmp = front_tmp / sqrt(1 - steering_track * front_tmp * cos(front_steering_angle) + pow(steering_track * front_tmp / 2, 2));
  front_right_tmp = front_tmp / sqrt(1 + steering_track * front_tmp * cos(front_steering_angle) + pow(steering_track * front_tmp / 2, 2));
  fl_speed_tmp = fl_wheel_vel * (1 / (1 - wheel_steering_y_offset * front_left_tmp));
  fr_speed_tmp = fr_wheel_vel * (1 / (1 - wheel_steering_y_offset * front_right_tmp));
  front_linear_speed = (wheel_diameter / 2.0) * copysign(1.0, fl_speed_tmp + fr_speed_tmp) * sqrt((pow(fl_wheel_vel, 2) + pow(fr_wheel_vel, 2)) / (2 + pow(steering_track * front_tmp, 2) / 2.0));

  rear_tmp = cos(rear_steering_angle) * (tan(front_steering_angle) - tan(rear_steering_angle)) / wheel_base;
  rear_left_tmp = rear_tmp / sqrt(1 - steering_track * rear_tmp * cos(rear_steering_angle) + pow(steering_track * rear_tmp / 2, 2));
  rear_right_tmp = rear_tmp / sqrt(1 + steering_track * rear_tmp * cos(rear_steering_angle) + pow(steering_track * rear_tmp / 2, 2));
  bl_speed_tmp = bl_wheel_vel * (1 / (1 - wheel_steering_y_offset * rear_left_tmp));
  br_speed_tmp = br_wheel_vel * (1 / (1 - wheel_steering_y_offset * rear_right_tmp));
  rear_linear_speed = (wheel_diameter / 2.0) * copysign(1.0, bl_speed_tmp + br_speed_tmp) * sqrt((pow(bl_speed_tmp, 2) + pow(br_speed_tmp, 2)) / (2 + pow(steering_track * rear_tmp, 2) / 2.0));

  // Compute yaw and delta distances
  angular = (front_linear_speed * front_tmp + rear_linear_speed * rear_tmp) / 2.0; // wheel speed

  yaw += delta_yaw; // imu

  velocity_x = (front_linear_speed * cos(front_steering_angle) + rear_linear_speed * cos(rear_steering_angle)) / 2.0; // robot_body_vel*cos(yaw);
  velocity_y = (front_linear_speed * sin(front_steering_angle) + rear_linear_speed * sin(rear_steering_angle)) / 2.0;
  if ((fabs(fl_wheel_steer) + fabs(fr_wheel_steer)) > 85*PI/180 && fabs(fl_wheel_steer + fr_wheel_steer) < 5*PI/180) {
    velocity_x = 0;
    velocity_y = 0;
    highTurn = true;

  }
  linear_vel = copysign(1.0, rear_linear_speed) * sqrt(pow(velocity_x, 2) + pow(velocity_y, 2));


double delta_time = (current_time - last_time).toSec();
if (delta_time < 0.0001) {
  return;
}
delta_x += ((velocity_x * cos(yaw) - velocity_y * sin(yaw))) * delta_time;
delta_y += ((velocity_x * sin(yaw) + velocity_y * cos(yaw))) * delta_time;
heading += angular * delta_time;

delta_yaw = 0.0;
nav_msgs::Odometry odom_msg;

geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

// odom publisher
odom_msg.header.seq = seq;
odom_msg.header.stamp = current_time;
odom_msg.header.frame_id = odometry_frame_id;
odom_msg.pose.pose.position.x = delta_x;
odom_msg.pose.pose.position.y = delta_y;
odom_msg.pose.pose.position.z = 0.0;
odom_msg.pose.pose.orientation = odom_quat;
odom_msg.child_frame_id = odometry_child_frame_id;
odom_msg.twist.twist.linear.x = velocity_x;
odom_msg.twist.twist.linear.y = velocity_y;
odom_msg.twist.twist.linear.z = 0.0;
odom_msg.twist.twist.angular.x = 0.0;
odom_msg.twist.twist.angular.y = 0.0;
odom_msg.twist.twist.angular.z = yaw_rate;

if (std::isnan(delta_x) || std::isnan(delta_y) || std::isnan(yaw) ||
    std::isnan(velocity_x) || std::isnan(velocity_y) || std::isnan(yaw_rate) ||
    std::isnan(odom_quat.z) || std::isnan(odom_quat.w)) {
  ROS_FATAL("NaN parameters. Restarting WO");
  ros::shutdown();
} else {
  if(!highTurn){ // dont publish WO if 90 deg turn

    odom_pub.publish(odom_msg);
  }
}

last_time = current_time;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  imu_time_prev = imu_time;
  imu_time = ros::Time::now().toSec();
  double delta_imu_time = imu_time - imu_time_prev;
  if (delta_imu_time < 0.0001) {
    return;
  }
  yaw_rate_prev = yaw_rate;
  yaw_rate = msg->angular_velocity.z;

  if (first_callback) {
    delta_yaw = 0.0;
    first_callback = false;

  } else {
    delta_yaw = ((yaw_rate + yaw_rate_prev) / 2.0) * (delta_imu_time);
  }
}
