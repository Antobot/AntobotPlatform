/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description:  The primary purpose of this code is to define the functions and variables used in heading.cpp
# Contacts: 	soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <std_srvs/Trigger.h>
#include <antobot_devices_msgs/progressUpdate.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>

#include <geonav_transform/navsat_conversions.h>

using namespace GeonavTransform::NavsatConversions;

namespace heading
{
    class heading{
        public:
            heading(ros::NodeHandle& nh);
            ~heading();
            void initialCalibration(void);
        protected:
			ros::NodeHandle nh_;
            tf2::Matrix3x3 mat;

            bool dual_gps;
            bool heading_received;
            std_msgs::Float64 dual_gps_heading;

            // configuration parameters
            double calib_distance, angular_zero_tol, lin_tol, calib_deg;

            // GPS
            std::vector<double> gps_start, gps_end;
            std::string gps_frame;
            bool gps_received;
            double utm_y, utm_x, rtk_status;
            std::string utm_zone;
            int rtk_target_status; 

            // imu
            geometry_msgs::Quaternion q_imu;
            std::string imu_frame;
            bool imu_received;
            float imu_ang_vel_z;
            double imu_offset;
            int imu_calibration_status;

            // odometry
            geometry_msgs::Quaternion q_odom;
            bool odometry_received;

            // wheel odometry
            double wheel_odom_v;

            // actual robot cmd_vel
            geometry_msgs::Twist robot_cmd_vel;

            // costmap
            bool costmap_received;
            std::vector<std::vector<int>> costmap_msg;
            int width,height;
            float resolution;
            float footprint_x, footprint_y;
            bool footprint_received;
            
            // Other parameters
            bool direction; // true if the robot is moving forward, false if the robot is moving backward
            double gps_yaw;
            bool hmi_auto_button_pressed;
            ros::Time hmi_auto_button_time, heading_received_time;

            // Read parameters to set the topic names
            std::string gps_topic, imu_topic, odometry_topic, wheelodometry_topic;
            bool sim;
            
            // Subscriber
            ros::Subscriber sub_gps, sub_imu, sub_odometry, sub_wheel_odom, sub_costmap;
            ros::Subscriber sub_footprint, sub_cmd_vel, sub_heading;

            // Publisher
            ros::Publisher pub_imu, pub_imu_offset, pub_calib, pub_imu_z, pub_GPS_origin; 
            ros::Publisher pub_cmd_vel,pub_switch;

            // Timer
            ros::Timer auto_calibration_timer;
            ros::Timer imu_pub_timer;

            // ServiceClient
            ros::ServiceClient ekf_srv;
            ros::ServiceClient hmi_srv;

            // ServiceServer
            ros::ServiceServer hmi_calibration_srv;

            // Functions
            void initialise(void);
            std::vector<double> eulerFromQuat(tf2::Quaternion);
            tf2::Quaternion quatFromEuler(double a, double b, double c);
            void autoCalibrate(const ros::TimerEvent& event);
            void publishNewIMU(const ros::TimerEvent& event);
            bool saveStartGPS(void);
            int checkCondition(void);
            void checkInputs(void);
            double calculateDifference(double, double);
            double normalize_angle(double);
            double autoCalibration_hmi(void);
            bool hmiService(antobot_devices_msgs::progressUpdate::Request &req, antobot_devices_msgs::progressUpdate::Response &res );
            bool checkObstacle(void);
            bool dualGPSHeadingCalibration(void);

            // Callback functions 
            void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
            void headingCallback(const std_msgs::Float64::ConstPtr& msg); // dual GPS
            void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
            void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
            void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    };
}