#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

#include "gps_imu_vel_filter.h"

int main (int argc, char** argv) {
    // Initialize ros.
    ros::init(argc, argv, "gps_imu_vel_filter");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    GpsImuVelFilter ekf_localizer(nh);

    ros::spin();
    return 1;
}