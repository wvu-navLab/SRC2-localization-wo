#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define PI 3.14159265359

// Declare sensor data variables
double fl_wheel_turn_counts = 0.0;
double fl_wheel_turn_counts_prev = 0.0;
double fl_delta_counts = 0.0;
double fl_wheel_steer=0.0;
double fr_wheel_steer=0.0;
double bl_wheel_steer=0.0;
double br_wheel_steer=0.0;
double fl_wheel_vel =0.0;
double bl_wheel_turn_counts = 0.0;
double bl_wheel_turn_counts_prev = 0.0;
double bl_delta_counts = 0.0;
double bl_wheel_vel =0.0;
double fr_wheel_turn_counts = 0.0;
double fr_wheel_turn_counts_prev = 0.0;
double fr_delta_counts = 0.0;
double fr_wheel_vel =0.0;
double br_wheel_turn_counts = 0.0;
double br_wheel_turn_counts_prev = 0.0;
double br_delta_counts = 0.0;
double br_wheel_vel =0.0;

double yaw_rate = 0.0;
double yaw_rate_prev = 0.0;
double pitch_rate = 0.0;
double pitch_rate_prev = 0.0;
double delta_yaw = 0.0;
// double delta_yaw2 = 0.0;
double delta_pitch =0.0;
double roll_rate=0.0;
double roll_rate_prev=0.0;
double delta_roll=0.0;


bool first_callback = true;
bool call_true_pose = true;
double imu_time;
double imu_time_prev;
double yaw = 0.0;
double pitch=0.0;
double roll=0.0;
double yawTruth = 0.0;
double pitchTruth=0.0;
double rollTruth=0.0;
double pitchAcc=0.0;
double rollAcc=0.0;
double pos_x=0.0;
double pos_y=0.0;
double pos_z=0.0;
double orient_x=0.0;
double orient_y=0.0;
double orient_z=0.0;
double orient_w=1.0;
// Declare sensor callback functions
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
// void posUpdateCallback(const nav_msgs::Odometry::ConstPtr& msg);

// void gazebostateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
int main(int argc, char** argv)
{
  // Declare variables that are read from params in 'rover_localization/config/dead_reckoning.yaml'
  float loop_rate; // Hz
  double wheel_diameter; // m
  double wheel_base; // m
  double steering_track; //m
  double wheel_steering_y_offset; //m
  double initPosx; //m
  double initPosy; //m
  double initPosz; //m
  double initYaw; //rad
  double initPitch; //rad
  double initRoll;//rad
  std::string imu_topic_name;
  std::string odometry_out_topic_name;
  std::string joint_state_topic_name;
  std::string odometry_frame_id;
  std::string odometry_child_frame_id;
  std::string joint_state_frame_id;
  // Initialize ROS
  std::string node_name = "rover_dead_reckoning_node";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Read params from yaml file
    if(ros::param::get(node_name+"/loop_rate",loop_rate)==false)
    {
      ROS_FATAL("No parameter 'loop_rate' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/wheel_diameter",wheel_diameter)==false)
    {
      ROS_FATAL("No parameter 'wheel_diameter' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/wheel_base",wheel_base)==false)
    {
      ROS_FATAL("No parameter 'wheel_base' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/steering_track",steering_track)==false)
    {
      ROS_FATAL("No parameter 'steering_track' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/wheel_steering_y_offset",wheel_steering_y_offset)==false)
    {
      ROS_FATAL("No parameter 'wheel_steering_y_offset' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/initPosx",initPosx)==false)
    {
      ROS_FATAL("No parameter 'initPosx' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/initPosy",initPosy )==false)
    {
      ROS_FATAL("No parameter 'initPosy' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/initPosz",initPosz)==false)
    {
      ROS_FATAL("No parameter 'initPosz' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/initYaw",initYaw)==false)
    {
      ROS_FATAL("No parameter 'initYaw' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/initPitch",initPitch)==false)
    {
      ROS_FATAL("No parameter 'initPitch' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/initRoll",initRoll)==false)
    {
      ROS_FATAL("No parameter 'initRoll' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/imu_topic_name",imu_topic_name)==false)
    {
      ROS_FATAL("No parameter 'imu_topic_name' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_out_topic_name",odometry_out_topic_name)==false)
    {
      ROS_FATAL("No parameter 'odometry_out_topic_name' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/joint_state_topic_name",joint_state_topic_name)==false)
    {
      ROS_FATAL("No parameter 'joint_state_topic_name' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_frame_id",odometry_frame_id)==false)
    {
      ROS_FATAL("No parameter 'odometry_frame_id' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/odometry_child_frame_id",odometry_child_frame_id)==false)
    {
      ROS_FATAL("No parameter 'odometry_child_frame_id' specified");
      ros::shutdown();
      exit(1);
    }
    if(ros::param::get(node_name+"/joint_state_frame_id",joint_state_frame_id)==false)
    {
      ROS_FATAL("No parameter 'joint_state_frame_id' specified");
      ros::shutdown();
      exit(1);
    }

  // Initialize publishers and subscribers
  ros::Subscriber imu_sub = nh.subscribe(imu_topic_name, 1, imuCallback);
  ros::Subscriber joint_state_sub = nh.subscribe(joint_state_topic_name, 1, jointstateCallback);
  // ros::Subscriber true_pose_sub = nh.subscribe("/posUpdate", 1, posUpdateCallback);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odometry_out_topic_name, 1);
  // ros::Publisher odomStatus=nh.advertise<std_msgs::Int64>("localization/status",100);
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener tf_listener_;

  // Initialize states
  nav_msgs::Odometry odom_msg; // Initializes to all zero, by default
  ros::Rate rate(loop_rate);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double delta_time = 0.0;
  double first_loop = true;
  uint32_t seq = 0;
  while(ros::ok())
  {
    double delta_x = 0.0;
    double delta_y = 0.0;
    double delta_z= 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double velocity_z= 0.0;
    double linear_vel=0.0;
    double front_steering_angle= 0.0;
    double rear_steering_angle=0.0;
    double front_tmp=0.0;
    double rear_tmp=0.0;
    double front_linear_speed=0.0;
    double rear_linear_speed=0.0;
    double angular=0.0;
    double front_left_tmp=0.0;
    double front_right_tmp=0.0;
    double fl_speed_tmp=0.0;
    double fr_speed_tmp=0.0;
    double rear_left_tmp=0.0;
    double rear_right_tmp=0.0;
    double bl_speed_tmp=0.0;
    double br_speed_tmp=0.0;

    current_time = ros::Time::now();
    double delta_time = (current_time - last_time).toSec();
    if(first_loop)
    {
      delta_x = 0.0;
      delta_y = 0.0;
      delta_z = 0.0;
      delta_yaw=0.0;
      // delta_yaw2=0.0;
      delta_pitch=0.0;
      delta_roll=0.0;
      yaw=initYaw; // initialize yaw
      pitch=initPitch;
      roll=initRoll;
      velocity_x = 0.0;
      velocity_y = 0.0;
      velocity_z=0.0;
      linear_vel=0.0;
      angular=0.0;
      front_steering_angle= 0.0;
      rear_steering_angle=0.0;
      first_loop = false;
      odom_msg.pose.pose.position.x = initPosx;
      odom_msg.pose.pose.position.y = initPosy;
      odom_msg.pose.pose.position.z = initPosz;
      odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(initRoll,initPitch,initYaw);
    }
    else
    {
      if (call_true_pose)
      {
          if (nh.hasParam("/true_pose_x"))
          {
                    nh.getParam("/true_pose_x", delta_x);
                    nh.getParam("/true_pose_y", delta_y);
                    nh.getParam("/true_pose_z", delta_z);
                    nh.getParam("/true_pose_orient_x", orient_x);
                    nh.getParam("/true_pose_orient_y", orient_y);
                    nh.getParam("/true_pose_orient_z", orient_z);
                    nh.getParam("/true_pose_orient_w", orient_w);

            tf::Quaternion q(orient_x,orient_y,orient_z,orient_w);
            tf::Matrix3x3 m(q.normalize());
            m.getRPY(rollTruth, pitchTruth, yawTruth);
            yaw=yawTruth;
            pitch=pitchTruth;
            roll=rollTruth;
            call_true_pose=false;
          }
        }
      else
      {

        // tf::StampedTransform T_fl2bl;
        // try
        // {
        //   tf_listener_.waitForTransform("/scout_1_tf/fl_wheel", "/scout_1_tf/bl_wheel",ros::Time(0),ros::Duration(1.0));
        //   tf_listener_.lookupTransform("/scout_1_tf/fl_wheel", "/scout_1_tf/bl_wheel",ros::Time(0),T_fl2bl);
        // }
        // catch(tf::TransformException &ex)
        // {
        //   ROS_WARN("%s", ex.what());
        // }
        // // tf::Vector3
        // tf::Vector3 t_fl2bl = T_fl2bl.getOrigin();
        // geometry_msgs::Point d_fl2bl;
        // d_fl2bl.x = t_fl2bl.getX(); //copy the components into geometry_msgs type
        // d_fl2bl.y = t_fl2bl.getY();
        // d_fl2bl.z = t_fl2bl.getZ();
        // //
        // double wheel_base_l = sqrt(d_fl2bl.x*d_fl2bl.x +d_fl2bl.y*d_fl2bl.y+ d_fl2bl.z*d_fl2bl.z);
        //
        // tf::StampedTransform T_fr2br;
        // try
        // {
        //   tf_listener_.waitForTransform("/scout_1_tf/fr_wheel", "/scout_1_tf/br_wheel",ros::Time(0),ros::Duration(1.0));
        //   tf_listener_.lookupTransform("/scout_1_tf/fr_wheel", "/scout_1_tf/br_wheel",ros::Time(0),T_fr2br);
        // }
        // catch(tf::TransformException &ex)
        // {
        //   ROS_WARN("%s", ex.what());
        // }
        // // tf::Vector3
        // tf::Vector3 t_fr2br = T_fr2br.getOrigin();
        // geometry_msgs::Point d_fr2br;
        // d_fr2br.x = t_fr2br.getX(); //copy the components into geometry_msgs type
        // d_fr2br.y = t_fr2br.getY();
        // d_fr2br.z = t_fr2br.getZ();
        // //
        // double wheel_base_r = sqrt(d_fr2br.x*d_fr2br.x +d_fr2br.y*d_fr2br.y+ d_fr2br.z*d_fr2br.z);
        //
        //
        //
        // tf::StampedTransform T_fl2fr;
        // try
        // {
        //   tf_listener_.waitForTransform("/scout_1_tf/fr_wheel", "/scout_1_tf/fl_wheel",ros::Time(0),ros::Duration(1.0));
        //   tf_listener_.lookupTransform("/scout_1_tf/fr_wheel", "/scout_1_tf/fl_wheel",ros::Time(0),T_fl2fr);
        // }
        // catch(tf::TransformException &ex)
        // {
        //   ROS_WARN("%s", ex.what());
        // }
        // // tf::Vector3
        // tf::Vector3 t_fl2fr = T_fl2fr.getOrigin();
        // geometry_msgs::Point d_fl2fr;
        // d_fl2fr.x = t_fl2fr.getX(); //copy the components into geometry_msgs type
        // d_fl2fr.y = t_fl2fr.getY();
        // d_fl2fr.z = t_fl2fr.getZ();
        // //
        // double steering_track_f = sqrt(d_fl2fr.x*d_fl2fr.x +d_fl2fr.y*d_fl2fr.y+ d_fl2fr.z*d_fl2fr.z);
        //
        // tf::StampedTransform T_bl2br;
        // try
        // {
        //   tf_listener_.waitForTransform("/scout_1_tf/bl_wheel", "/scout_1_tf/br_wheel",ros::Time(0),ros::Duration(1.0));
        //   tf_listener_.lookupTransform("/scout_1_tf/bl_wheel", "/scout_1_tf/br_wheel",ros::Time(0),T_bl2br);
        // }
        // catch(tf::TransformException &ex)
        // {
        //   ROS_WARN("%s", ex.what());
        // }
        // // tf::Vector3
        // tf::Vector3 t_bl2br = T_bl2br.getOrigin();
        // geometry_msgs::Point d_bl2br;
        // d_bl2br.x = t_bl2br.getX(); //copy the components into geometry_msgs type
        // d_bl2br.y = t_bl2br.getY();
        // d_bl2br.z = t_bl2br.getZ();
        // //
        // double steering_track_r = sqrt(d_bl2br.x*d_bl2br.x +d_bl2br.y*d_bl2br.y+ d_fl2fr.z*d_bl2br.z);
        //
        //
        //
        // wheel_base = (wheel_base_l+wheel_base_r)/2.0;
        // steering_track = (steering_track_f+steering_track_r)/2.0;



        //
        front_steering_angle = 2/((1/tan(fl_wheel_steer))+(1/tan(fr_wheel_steer)));
        rear_steering_angle = 2/((1/tan(bl_wheel_steer))+(1/tan(br_wheel_steer)));


        front_tmp = cos(front_steering_angle)*(tan(front_steering_angle)-tan(rear_steering_angle))/wheel_base;
        front_left_tmp = front_tmp/sqrt(1-steering_track*front_tmp*cos(front_steering_angle)+pow(steering_track*front_tmp/2,2));
        front_right_tmp = front_tmp/sqrt(1+steering_track*front_tmp*cos(front_steering_angle)+pow(steering_track*front_tmp/2,2));
        fl_speed_tmp = fl_wheel_vel * (1/(1-wheel_steering_y_offset*front_left_tmp));
        fr_speed_tmp = fr_wheel_vel * (1/(1-wheel_steering_y_offset*front_right_tmp));
        front_linear_speed = (wheel_diameter/2.0) * copysign(1.0, fl_speed_tmp+fr_speed_tmp)*sqrt((pow(fl_wheel_vel,2)+pow(fr_wheel_vel,2))/(2+pow(steering_track*front_tmp,2)/2.0));

        rear_tmp = cos(rear_steering_angle)*(tan(front_steering_angle)-tan(rear_steering_angle))/wheel_base;
        rear_left_tmp = rear_tmp/sqrt(1-steering_track*rear_tmp*cos(rear_steering_angle)+pow(steering_track*rear_tmp/2,2));
        rear_right_tmp = rear_tmp/sqrt(1+steering_track*rear_tmp*cos(rear_steering_angle)+pow(steering_track*rear_tmp/2,2));
        bl_speed_tmp = bl_wheel_vel * (1/(1-wheel_steering_y_offset*rear_left_tmp));
        br_speed_tmp = br_wheel_vel * (1/(1-wheel_steering_y_offset*rear_right_tmp));
        rear_linear_speed = (wheel_diameter/2.0)  * copysign(1.0, bl_speed_tmp+br_speed_tmp)*sqrt((pow(bl_speed_tmp,2)+pow(br_speed_tmp,2))/(2+pow(steering_track*rear_tmp,2)/2.0));




      // front_steering_angle = 2/((1/tan(fl_wheel_steer))+(1/tan(fr_wheel_steer)));
      // rear_steering_angle = 2/((1/tan(bl_wheel_steer))+(1/tan(br_wheel_steer)));
      // front_tmp = cos(front_steering_angle)*(tan(front_steering_angle)-tan(rear_steering_angle))/wheel_base;
      // front_linear_speed = (wheel_diameter/2.0) * copysign(1.0, fl_wheel_vel+fr_wheel_vel)*sqrt((pow(fl_wheel_vel,2)+pow(fr_wheel_vel,2))/(2+pow(steering_track*front_tmp,2)/2.0));
      // rear_tmp = cos(rear_steering_angle)*(tan(front_steering_angle)-tan(rear_steering_angle))/wheel_base;
      // rear_linear_speed = (wheel_diameter/2.0) * copysign(1.0, bl_wheel_vel+br_wheel_vel)*sqrt((pow(bl_wheel_vel,2)+pow(br_wheel_vel,2))/(2+pow(steering_track*rear_tmp,2)/2.0));



      // Compute yaw and delta distances
      angular= (front_linear_speed*front_tmp + rear_linear_speed*rear_tmp)/2.0; //wheel speed

      yaw += delta_yaw; //imu
      pitch=delta_pitch;
      roll=delta_roll;
      velocity_x = (front_linear_speed*cos(front_steering_angle) + rear_linear_speed*cos(rear_steering_angle))/2.0;//robot_body_vel*cos(yaw);
      velocity_y = (front_linear_speed*sin(front_steering_angle) - wheel_base*yaw/2.0+ rear_linear_speed*sin(rear_steering_angle) + wheel_base*yaw/2.0)/2.0;
      linear_vel =  copysign(1.0, rear_linear_speed)*sqrt(pow(velocity_x,2)+pow(velocity_y,2));
      // velocity_z=-linear_vel*sin(pitch);

      // delta_x=((velocity_x*cos(yaw)-velocity_y*sin(yaw))/cos(pitch))*delta_time;
      // delta_y=((velocity_x*sin(yaw)+velocity_y*cos(yaw))/cos(pitch))*delta_time;
      delta_x=((velocity_x*cos(yaw)-velocity_y*sin(yaw)))*delta_time;
      delta_y=((velocity_x*sin(yaw)+velocity_y*cos(yaw)))*delta_time;
      // delta_z=velocity_z*delta_time;
      // std::cout << "pitch" << pitchTruth <<'\n';

      delta_yaw = 0.0;
      }
      // delta_pitch=0.0;
      // delta_roll=0.0;
    }

    //tf broadcaster for odometry
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "scout_1_tf/chassis";
    odom_trans.transform.translation.x += delta_x;
    odom_trans.transform.translation.y += delta_y;
    odom_trans.transform.translation.z += delta_z;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    //odom publisher
    odom_msg.header.seq = seq;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.pose.pose.position.x += delta_x;
    odom_msg.pose.pose.position.y += delta_y;
    odom_msg.pose.pose.position.z += delta_z;
    odom_msg.pose.pose.orientation = odom_quat;
    odom_msg.child_frame_id = "scout_1_tf/chassis";
    odom_msg.twist.twist.linear.x = velocity_x;
    odom_msg.twist.twist.linear.y = velocity_y;
    odom_msg.twist.twist.linear.z = velocity_z;
    odom_msg.twist.twist.angular.x= roll_rate;
    odom_msg.twist.twist.angular.y= pitch_rate;
    odom_msg.twist.twist.angular.z = yaw_rate;

    odom_pub.publish(odom_msg);
    ros::spinOnce();
    last_time = current_time;
    rate.sleep();
  }

  return 0;
}


void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  bl_wheel_turn_counts_prev = bl_wheel_turn_counts;
  bl_wheel_turn_counts = msg->position.at(2);
  bl_wheel_vel= msg->velocity.at(2);
  bl_wheel_steer= msg->position.at(1);

  br_wheel_turn_counts_prev = br_wheel_turn_counts;
  br_wheel_turn_counts = msg->position.at(5);
  br_wheel_vel= msg->velocity.at(5);
  br_wheel_steer= msg->position.at(4);

  fl_wheel_turn_counts_prev = fl_wheel_turn_counts;
  fl_wheel_turn_counts = msg->position.at(8);
  fl_wheel_vel = msg->velocity.at(8);
  fl_wheel_steer= msg->position.at(7);

  fr_wheel_turn_counts_prev = fr_wheel_turn_counts;
  fr_wheel_turn_counts = msg->position.at(11);
  fr_wheel_vel= msg->velocity.at(11);
  fr_wheel_steer= msg->position.at(10);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_time_prev = imu_time;
  imu_time = ros::Time::now().toSec();


  yaw_rate_prev = yaw_rate;
  yaw_rate = msg->angular_velocity.z;


  double accX=msg->linear_acceleration.x;
  double accY=msg->linear_acceleration.y;
  double accZ=msg->linear_acceleration.z;

  double rollAcc = atan2(accY, accZ);
  double pitchAcc= atan2(-accX, sqrt(accY*accY + accZ*accZ));



  double rollOrient, pitchOrient,yawOrient;
  //
  tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  //
  tf::Matrix3x3 m(q);
  m.getRPY(rollOrient, pitchOrient, yawOrient);

  if(first_callback)
  {
    // delta_roll=0.0;
    // delta_pitch=0.0;
    delta_yaw = 0.0;
    roll_rate =0.0;
    pitch_rate=0.0;

    first_callback = false;
  }
  else
  {
    delta_yaw = ((yaw_rate + yaw_rate_prev)/2.0)*(imu_time - imu_time_prev);
    delta_pitch= pitchAcc;
    delta_roll=rollAcc;
    // roll_rate_prev = roll_rate;
    roll_rate = rollAcc/(imu_time - imu_time_prev);

    // pitch_rate_prev =pitch_rate;
    pitch_rate= pitchAcc/(imu_time - imu_time_prev);

  }
}
