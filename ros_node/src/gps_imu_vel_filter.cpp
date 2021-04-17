#include <tf2/LinearMath/Quaternion.h>
#include <angles/angles.h>
#include <algorithm>
#include "gps_imu_vel_filter.h"

GpsImuVelFilter::GpsImuVelFilter(ros::NodeHandle& nh) {
    // Load configs.
    double accl_noise = 0.0, gyro_noise = 0.0, accl_bias_noise = 0.0, gyro_bias_noise = 0.0;
    nh.param("accl_noise",      accl_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("accl_bias_noise", accl_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double sigma_gps_pos_ne = 0.0, sigma_gps_pos_d = 0.0, sigma_gps_vel_ne = 0.0, sigma_gps_vel_d= 0.0;
    nh.param("sigma_gps_pos_ne", sigma_gps_pos_ne, 0.);
    nh.param("sigma_gps_pos_d", sigma_gps_pos_d, 0.);
    nh.param("sigma_gps_vel_ne", sigma_gps_vel_ne, 0.);
    nh.param("sigma_gps_vel_d", sigma_gps_vel_d, 0.);

    nh.param<bool>("vel_tf_enu_to_ned", vel_tf_enu_to_ned, "true");
    nh.param<bool>("imu_tf_enu_to_ned", imu_tf_enu_to_ned, "true");

    std::string gps_fix_topic, gps_vel_topic, imu_topic, mag_topic;
    nh.param<std::string>("gps_fix_topic", gps_fix_topic, "/fix");
    nh.param<std::string>("gps_vel_topic", gps_vel_topic, "/vel");
    nh.param<std::string>("imu_topic", imu_topic, "/imu/data");
    nh.param<std::string>("mag_topic", mag_topic, "/mag");

    int gps_publish_rate, imu_publish_rate, mag_publish_rate;
    nh.param<int>("gps_publish_rate", gps_publish_rate, 10);
    nh.param<int>("imu_publish_rate", imu_publish_rate, 200);
    nh.param<int>("mag_publish_rate", mag_publish_rate, 50);

    // Initialization ekf imu gps vel localizer.
    ekfNavINS_ptr_ = std::make_unique<ekfNavINS>();

    // Initialize error charasteristics
    ekfNavINS_ptr_->setAcclNoise(accl_noise);
    ekfNavINS_ptr_->setAcclBias(accl_bias_noise);
    ekfNavINS_ptr_->setGyroNoise(gyro_noise);
    ekfNavINS_ptr_->setGyroBias(gyro_bias_noise);
    ekfNavINS_ptr_->setStdDevGpsPosNE(sigma_gps_pos_ne);
    ekfNavINS_ptr_->setStdDevGpsPosD(sigma_gps_pos_d);
    ekfNavINS_ptr_->setStdDevGpsVelNE(sigma_gps_vel_ne);
    ekfNavINS_ptr_->setStdDevGpsVelD(sigma_gps_vel_d);

    // Subscribe topics.
    imu_sub_ = nh.subscribe(imu_topic, imu_publish_rate,  &GpsImuVelFilter::ImuCallback, this);
    mag_sub_ = nh.subscribe(mag_topic, mag_publish_rate, &GpsImuVelFilter::MagCallback, this);
    gps_pos_sub_ = nh.subscribe(gps_fix_topic, gps_publish_rate,  &GpsImuVelFilter::GpsPosCallback, this);
    gps_vel_sub_ = nh.subscribe(gps_vel_topic, gps_publish_rate,  &GpsImuVelFilter::GpsVelCallback, this);

    // Initialize Publishers
    //ekf_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ekf_fused_pose", 10);
    ekf_pos_pub_  = nh.advertise<sensor_msgs::NavSatFix>("ekf_fix_filtered", 10);
    ekf_vel_pub_  = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("ekf_vel_filtered", 10);
    ekf_imu_pub_  = nh.advertise<sensor_msgs::Imu>("ekf_imu_filtered",  10);
    //ekf_mag_pub_  = nh.advertise<sensor_msgs::MagneticField>("ekf_mag_filtered",  10);
}

GpsImuVelFilter::~GpsImuVelFilter() {

}

void GpsImuVelFilter::MagCallback(const sensor_msgs::MagneticFieldPtr& mag_msg_ptr) {
    magDataPtr mag_data_ptr = std::make_shared<magData>();
    mag_link_name = mag_msg_ptr->header.frame_id;
    mag_data_ptr->mag_time = mag_msg_ptr->header.stamp.toSec();
    mag_data_ptr->hX = mag_msg_ptr->magnetic_field.x;
    mag_data_ptr->hY = mag_msg_ptr->magnetic_field.y;
    mag_data_ptr->hZ = mag_msg_ptr->magnetic_field.z;
    ekfNavINS_ptr_->magDataUpdateEKF(mag_data_ptr);
}

void GpsImuVelFilter::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    imuDataPtr imu_data_ptr = std::make_shared<imuData>();
    imu_link_name = imu_msg_ptr->header.frame_id;
    imu_data_ptr->imu_time = imu_msg_ptr->header.stamp.toSec();
    if (imu_tf_enu_to_ned == true) {
        std::tie(imu_data_ptr->acclX, imu_data_ptr->acclY, imu_data_ptr->acclZ) = 
                FromEnuToNed(imu_msg_ptr->linear_acceleration.x, imu_msg_ptr->linear_acceleration.y, imu_msg_ptr->linear_acceleration.z);
        std::tie(imu_data_ptr->gyroX, imu_data_ptr->gyroY, imu_data_ptr->gyroZ) = 
                FromEnuToNed(imu_msg_ptr->angular_velocity.x, imu_msg_ptr->angular_velocity.y, imu_msg_ptr->angular_velocity.z);
    } else {
        imu_data_ptr->acclX = imu_msg_ptr->linear_acceleration.x; 
        imu_data_ptr->acclY = imu_msg_ptr->linear_acceleration.y;
        imu_data_ptr->acclZ = imu_msg_ptr->linear_acceleration.z;
        imu_data_ptr->gyroX = imu_msg_ptr->angular_velocity.x;
        imu_data_ptr->gyroY = imu_msg_ptr->angular_velocity.y;
        imu_data_ptr->gyroZ = imu_msg_ptr->angular_velocity.z;
    }
    ekfState fused_state;
    if (ekfNavINS_ptr_->imuDataUpdateEKF(imu_data_ptr, &fused_state) != true) {
        return;
    }
    // Publish fused state.
    PrepareRosTopics(fused_state);
    //ekf_pose_pub_.publish(ros_pose_);
    ekf_pos_pub_.publish(ros_pos_);
    ekf_vel_pub_.publish(ros_vel_);
    ekf_imu_pub_.publish(ros_imu_);
}

void GpsImuVelFilter::GpsPosCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    if (gps_msg_ptr->status.status != 0) {
        ROS_WARN("Bad GPS data!!! Waiting to receive the GPS fix...");
        return;
    }
    restore_.status = gps_msg_ptr->status;
    restore_.position_covariance_type = gps_msg_ptr->position_covariance_type;
    gps_link_name = gps_msg_ptr->header.frame_id;

    gpsPosDataPtr gps_data_ptr = std::make_shared<gpsPosData>();
    gps_data_ptr->pos_time = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lat = angles::from_degrees(gps_msg_ptr->latitude);
    gps_data_ptr->lon = angles::from_degrees(gps_msg_ptr->longitude);
    gps_data_ptr->alt = gps_msg_ptr->altitude;
    ekfNavINS_ptr_->gpsPosDataUpdateEKF(gps_data_ptr);
}

void GpsImuVelFilter::GpsVelCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& vel_msg_ptr) {
    gpsVelDataPtr gps_vel_data_ptr = std::make_shared<gpsVelData>();
    gps_vel_data_ptr->vel_time = vel_msg_ptr->header.stamp.toSec();
    if (vel_tf_enu_to_ned == true) {
        std::tie(gps_vel_data_ptr->vN, gps_vel_data_ptr->vE, gps_vel_data_ptr->vD) = 
                    FromEnuToNed(vel_msg_ptr->twist.twist.linear.x, vel_msg_ptr->twist.twist.linear.y, vel_msg_ptr->twist.twist.linear.z);
    } else {
        gps_vel_data_ptr->vN = vel_msg_ptr->twist.twist.linear.x;
        gps_vel_data_ptr->vE = vel_msg_ptr->twist.twist.linear.y;
        gps_vel_data_ptr->vD = vel_msg_ptr->twist.twist.linear.z;
    }
    ekfNavINS_ptr_->gpsVelDataUpdateEKF(gps_vel_data_ptr);
}

void GpsImuVelFilter::PrepareRosTopics(const ekfState& ekf_state) {
    /*
    ros_pose_.header.frame_id = "map";
    ros_pose_.header.stamp = ros::Time::now();
    ros_pose_.pose.position.x = ekf_state.lla(0);
    ros_pose_.pose.position.y = ekf_state.lla(1);
    ros_pose_.pose.position.z = ekf_state.lla(2);
    tf2::Quaternion quat;
    quat.setRPY(ekf_state.rpy(0), ekf_state.rpy(1), ekf_state.rpy(2));
    quat=quat.normalize();
    ros_pose_.pose.orientation.x = quat.x();
    ros_pose_.pose.orientation.y = quat.y();
    ros_pose_.pose.orientation.z = quat.z();
    ros_pose_.pose.orientation.w = quat.w();
    */

    // NavSatFix
    ros_pos_.header.frame_id = gps_link_name;
    ros_pos_.header.stamp = ros::Time::now();
    ros_pos_.status = restore_.status;
    ros_pos_.position_covariance_type = restore_.position_covariance_type;
    ros_pos_.latitude = angles::to_degrees(ekf_state.lla(0));
    ros_pos_.longitude = angles::to_degrees(ekf_state.lla(1));
    ros_pos_.altitude = ekf_state.lla(2);
    auto resultEigen = ekf_state.cov.block(0,0,3,3);
    float *tmp_ptr = reinterpret_cast<float*>(&(ros_pos_.position_covariance[0]));
    Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor> >(tmp_ptr,3,3) = resultEigen;

    // IMU
    ros_imu_.header.frame_id = imu_link_name;
    ros_imu_.header.stamp = ros::Time::now();
    ros_imu_.orientation.w = ekf_state.quat(0);
    if (imu_tf_enu_to_ned == true) {
        std::tie(ros_imu_.orientation.x, ros_imu_.orientation.y, ros_imu_.orientation.z) = 
            FromNedToEnu(ekf_state.quat(1), ekf_state.quat(2), ekf_state.quat(3));
        std::tie(ros_imu_.angular_velocity.x, ros_imu_.angular_velocity.y, ros_imu_.angular_velocity.z) = 
            FromNedToEnu(ekf_state.angular(0), ekf_state.angular(1), ekf_state.angular(2));
        std::tie(ros_imu_.linear_acceleration.x, ros_imu_.linear_acceleration.y, ros_imu_.linear_acceleration.z) = 
            FromNedToEnu(ekf_state.linear(0), ekf_state.linear(1), ekf_state.linear(2));
    } else {
        ros_imu_.orientation.x = ekf_state.quat(1);
        ros_imu_.orientation.y = ekf_state.quat(2);
        ros_imu_.orientation.z = ekf_state.quat(3);
        ros_imu_.angular_velocity.x = ekf_state.angular(0);
        ros_imu_.angular_velocity.y = ekf_state.angular(1);
        ros_imu_.angular_velocity.z = ekf_state.angular(2);
        ros_imu_.linear_acceleration.x = ekf_state.linear(0);
        ros_imu_.linear_acceleration.y = ekf_state.linear(1);
        ros_imu_.linear_acceleration.z = ekf_state.linear(2);
    }

    // vel
    ros_vel_.header.frame_id = gps_link_name;
    ros_vel_.header.stamp = ros::Time::now();
    if (vel_tf_enu_to_ned == true) {
        std::tie(ros_vel_.twist.twist.linear.x, ros_vel_.twist.twist.linear.y, ros_vel_.twist.twist.linear.z) = 
            FromNedToEnu(ekf_state.velNED(0), ekf_state.velNED(1), ekf_state.velNED(2));
    } else {
        ros_vel_.twist.twist.linear.x = ekf_state.velNED(0);
        ros_vel_.twist.twist.linear.y = ekf_state.velNED(1);
        ros_vel_.twist.twist.linear.z = ekf_state.velNED(2);
    }
    auto resultEigenVel = ekf_state.cov.block(3,3,3,3);
    float *tmp_ptr_imu_cov = reinterpret_cast<float*>(&(ros_vel_.twist.covariance[0]));
    Eigen::Map<Eigen::Matrix<float,3,3,Eigen::RowMajor> >(tmp_ptr_imu_cov,3,3) = resultEigen;
}

constexpr std::tuple<float, float, float> GpsImuVelFilter::FromNedToEnu(float n, float e, float d) {
    return (std::make_tuple(e, n, -d));
}

constexpr std::tuple<float, float, float> GpsImuVelFilter::FromEnuToNed(float e, float n, float u) {
    return (std::make_tuple(n, e, -u));
}