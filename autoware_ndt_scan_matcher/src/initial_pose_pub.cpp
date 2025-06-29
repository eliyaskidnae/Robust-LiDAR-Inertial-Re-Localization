#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class InitialPosePublisher : public rclcpp::Node
{
  public:
  InitialPosePublisher()
    : Node("minimal_publisher")
    {
      pose_file_name = generateFilename();
    
      intial_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
      lio_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/Odometry", 10, std::bind(&InitialPosePublisher::lio_odom_callback, this, std::placeholders::_1));
      ekf_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/fused_odom", 10, std::bind(&InitialPosePublisher::ekf_odom_callback, this, std::placeholders::_1));
      ndt_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ndt_odom", 10, std::bind(&InitialPosePublisher::ndt_odom_callback, this, std::placeholders::_1));
      dlio_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ndt_odom_all", 10, std::bind(&InitialPosePublisher::dlio_odom_callback, this, std::placeholders::_1));
      
      // current time stamp based name 
      pose_publisher();

    }

  private:
    void read_tum_dataset()
    {
      // read tum dataset and save to log file 

      
    }

    void ndt_odom_callback(nav_msgs::msg::Odometry msg)
    {
      // read msg save to log file 
    
      if ( ndt_init &&  calc_distance(pre_ndt_pose, msg) < th_distance)
      {
        return;
      }
      std::ofstream pose_log;
      
      // Set fixed-point notation and high precision (9 digits after decimal)
      pose_log << std::fixed << std::setprecision(9);
      double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
      pose_log.open( pose_file_name + "_ndt_poses" , std::ios::app);
      pose_log << timestamp << " " << msg.pose.pose.position.x << " " <<msg.pose.pose.position.y << " " <<msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " <<msg.pose.pose.orientation.z<< " " <<msg.pose.pose.orientation.w << std::endl;
      pose_log.close();
      pre_ndt_pose = msg;
      ndt_init = true;

    }

    void dlio_odom_callback(nav_msgs::msg::Odometry msg)
    {
      
      if (lio_init && calc_distance(pre_lio_pose, msg) < 0.5)
      {
        return;
      }
      std::ofstream pose_log;   
      double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

      // Set fixed-point notation and high precision (9 digits after decimal)
      pose_log << std::fixed << std::setprecision(9);
      pose_log.open( pose_file_name + "_dlio_poses", std::ios::app);
      pose_log << timestamp <<  " " << msg.pose.pose.position.x << " " <<msg.pose.pose.position.y << " " <<msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " <<msg.pose.pose.orientation.z<< " " <<msg.pose.pose.orientation.w << std::endl;
      pose_log.close();
      pre_lio_pose = msg;
      lio_init = true;
    }
    void lio_odom_callback(nav_msgs::msg::Odometry msg)
    {
      
      if (lio_init && calc_distance(pre_lio_pose, msg) < th_distance)
      {
        return;
      }
      std::ofstream pose_log;   
      double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

      // Set fixed-point notation and high precision (9 digits after decimal)
      pose_log << std::fixed << std::setprecision(9);
      pose_log.open( pose_file_name + "_lio_poses", std::ios::app);
      pose_log << timestamp <<  " " << msg.pose.pose.position.x << " " <<msg.pose.pose.position.y << " " <<msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " <<msg.pose.pose.orientation.z<< " " <<msg.pose.pose.orientation.w << std::endl;
      pose_log.close();
      pre_lio_pose = msg;
      lio_init = true;
    }
    void ekf_odom_callback(nav_msgs::msg::Odometry msg)
    { 

      if (ekf_init && calc_distance(prev_ekf_pose, msg) < th_distance)
      {
        return;
      }

      std::ofstream pose_log; 
      double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
      // Set fixed-point notation and high precision (9 digits after decimal)
      pose_log << std::fixed << std::setprecision(9);
      pose_log.open( pose_file_name + "_fused_poses" , std::ios::app);
      pose_log << timestamp << " " << msg.pose.pose.position.x << " " <<msg.pose.pose.position.y << " " <<msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " <<msg.pose.pose.orientation.z<< " " <<msg.pose.pose.orientation.w << std::endl;
      pose_log.close(); 
      prev_ekf_pose = msg;
      ekf_init = true;

    }
    void pose_publisher()
    {
      auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
      message.header.stamp = this->now();
      message.header.frame_id = "map";
      message.pose.pose.position.x = 0.0;
      message.pose.pose.position.y = 0.0;
      message.pose.pose.position.z = 0.0;
      message.pose.pose.orientation.x = 0.0;
      message.pose.pose.orientation.y = 0.0;
      message.pose.pose.orientation.z = 0.0;
      message.pose.pose.orientation.w = 1.0;
      intial_pose_pub->publish(message);
    }

    std::string generateFilename()
    {
      auto now = std::chrono::system_clock::now();
      auto now_c = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << "data/path_log/";
      ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
      return ss.str();
    }

    double calc_distance(nav_msgs::msg::Odometry pose1, nav_msgs::msg::Odometry pose2)
    {
      double x1 = pose1.pose.pose.position.x;
      double y1 = pose1.pose.pose.position.y;
      double z1 = pose1.pose.pose.position.z;
      double x2 = pose2.pose.pose.position.x;
      double y2 = pose2.pose.pose.position.y;
      double z2 = pose2.pose.pose.position.z;
      return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr intial_pose_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ndt_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dlio_odom_sub;
    std::string pose_file_name;

    // previous pose msg  and nitilize null
    nav_msgs::msg::Odometry pre_ndt_pose;
    nav_msgs::msg::Odometry pre_lio_pose;
    nav_msgs::msg::Odometry prev_ekf_pose;

    bool ndt_init = false;
    bool lio_init = false;
    bool ekf_init = false;
    
    // double th_distance 
    double th_distance = 0.5;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}