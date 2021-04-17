#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <string>

#include "ekfNavINS.h"

class GpsImuVelFilter {
public:
    GpsImuVelFilter(ros::NodeHandle& nh);
    ~GpsImuVelFilter();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    void MagCallback(const sensor_msgs::MagneticFieldPtr& mag_msg_ptr);
    void GpsPosCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);
    void GpsVelCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& vel_msg_ptr);

private:
    void PrepareRosTopics(const ekfState& ekf_state);
    constexpr std::tuple<float, float, float> FromNedToEnu(float n, float e, float d);
    constexpr std::tuple<float, float, float> FromEnuToNed(float e, float n, float u);

    // Transform conversion flags from params
    bool vel_tf_enu_to_ned;
    bool imu_tf_enu_to_ned;
    bool vel_output_to_ned;
    bool imu_output_to_ned;
    std::string gps_link_name;
    std::string imu_link_name;
    std::string mag_link_name;
    
    //
    // Subscribers
    // 
    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;
    ros::Subscriber gps_pos_sub_;
    ros::Subscriber gps_vel_sub_;
    //
    // Publishers
    // 
    //ros::Publisher ekf_pose_pub_;
    ros::Publisher ekf_pos_pub_;
    ros::Publisher ekf_vel_pub_;
    ros::Publisher ekf_imu_pub_;
    ros::Publisher ekf_mag_pub_;

    //geometry_msgs::PoseStamped  ros_pose_;
    sensor_msgs::NavSatFix                      ros_pos_;
    geometry_msgs::TwistWithCovarianceStamped   ros_vel_;
    sensor_msgs::Imu            ros_imu_;
    sensor_msgs::MagneticField  ros_mag_;

    std::unique_ptr<ekfNavINS> ekfNavINS_ptr_;

    //NavSatFix restore
    sensor_msgs::NavSatFix restore_;
};