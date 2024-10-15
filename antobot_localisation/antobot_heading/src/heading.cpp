/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: The primary purpose of this code is to calculate yaw offset from the robot motion with single GPS. 
#             This script subscribes to the IMU and GPS topics and publishes Imu messages over 
# 		    imu/data_corrected topic. heading node performs the same functions as the heading_node.py but is written in C++.
# Subscribes to: GPS topic (/am_gps_uruc)
#                imu topic (/imu/data)
#                EKF odometry topic (/odometry/filtered)
#                wheel odometry topic (/antobot_robot/odom)
# Publishes : Calibrated imu topic (/imu/data_corrected) - which is then used in EKF
# Contacts: 	soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <antobot_heading/heading.h>

namespace heading
{
    heading::heading(ros::NodeHandle& nh)
    : nh_(nh)
    {   
        // Description: Constructor for the heading class
        ROS_INFO("Initialise");
        initialise();
    }

    heading::~heading(){
        // Description: Deconstructor for the heading class
    }

    void heading::initialise(){
        // Description: Initialises autoCalibration class
        if (nh_.hasParam("/gps/ublox_rover")){
            ROS_INFO("MV2100: Heading IMU calibration - using DUAL GPS");
            dual_gps = true;
        }
        else{
            ROS_INFO("MV2100: Heading IMU calibration - using One GPS & wheel odometry");
            dual_gps = false;
        }


        // configuration parameters
        calib_distance = 1.0; // calibration distance 
        angular_zero_tol = 0.1; // Angular velocity (rad/sec)
        lin_tol = 0.01; // linear velocity x (m/s)
        calib_deg = 3; // deg

        // GPS
        gps_frame = "gps_frame";
        gps_start.resize(2);
        gps_end.resize(2);
        gps_received = false;
        utm_y = 0;
        utm_x = 0;
        utm_zone = "";
        rtk_status = -1; // default value

        // dual GPS
        heading_received = false;
        heading_received_time = ros::Time::now();

        // imu
        imu_frame = "imu";
        imu_received = false;
        imu_ang_vel_z = 0;
        imu_offset = 0;
        imu_calibration_status = -1;

        // odometry
        odometry_received = false;

        // wheel odometry
        wheel_odom_v = 0;

        robot_cmd_vel = geometry_msgs::Twist();



        //costmap
        costmap_received = false;
        
        // Other parameters
        direction = true; // true if the robot is moving forward, false if the robot is moving backward
        gps_yaw = 0;
        hmi_auto_button_pressed = false;
        hmi_auto_button_time =ros::Time::now();

        // Read parameters to set the topic names
        nh_.getParam("/heading_node/gps_topic", gps_topic);
        nh_.getParam("/heading_node/imu_topic", imu_topic);
        nh_.getParam("/heading_node/odometry_topic", odometry_topic);
        nh_.getParam("/heading_node/wheel_odometry_topic", wheelodometry_topic);

        nh_.param<bool>("/simulation", sim, false);

        rtk_target_status = 3; // rtk status is 3 in 3D fixed mode
        if (sim){
            rtk_target_status = 0; // in simulation, gps status is always 0
        }

        // Subscribers
        sub_gps = nh_.subscribe<sensor_msgs::NavSatFix>(gps_topic, 100, &heading::gpsCallback,this);
        sub_imu = nh_.subscribe<sensor_msgs::Imu>(imu_topic, 100, &heading::imuCallback,this);
        sub_odometry = nh_.subscribe<nav_msgs::Odometry>(odometry_topic, 100, &heading::odometryCallback,this);
        sub_wheel_odom = nh_.subscribe<nav_msgs::Odometry>(wheelodometry_topic, 100, &heading::wheelOdomCallback,this);
        sub_costmap = nh_.subscribe<nav_msgs::OccupancyGrid>("/costmap_node/costmap/costmap", 100, &heading::costmapCallback,this);
        sub_footprint = nh_.subscribe<geometry_msgs::PolygonStamped>("/costmap_node/costmap/footprint", 100, &heading::footprintCallback,this);
        sub_cmd_vel = nh_.subscribe<geometry_msgs::Twist>("/antobot_robot/cmd_vel", 100, &heading::cmdVelCallback,this);
        sub_heading = nh_.subscribe<std_msgs::Float64>("am_heading_robot", 100, &heading::headingCallback,this);
        
        // Publisher
        pub_imu = nh_.advertise<sensor_msgs::Imu>("/imu/data_corrected", 1);
        pub_imu_z = nh_.advertise<std_msgs::Float32>("/imu/heading_z", 1); // For opcua
        pub_imu_offset = nh_.advertise<std_msgs::Float32>("/imu/data_offset", 1); // For debug
        pub_calib = nh_.advertise<std_msgs::UInt8>("/imu_calibration_status", 1); // For heading_launcher. Notify when the initial calibraion is done.
        pub_GPS_origin = nh_.advertise<sensor_msgs::NavSatFix>("/gps_map_origin",1);
        pub_cmd_vel = nh_.advertise<geometry_msgs::Twist>("/am_nav/cmd_vel",1); // cmd_vel for HMI auto calibration
        pub_switch = nh_.advertise<std_msgs::UInt8>("/antobot/teleop/switch_mode", 1);
        // Service
        ekf_srv = nh_.serviceClient<std_srvs::Trigger>("launch_ekf"); // launch ekf nodes once the initial calibration is done
        hmi_srv = nh_.serviceClient<antobot_devices_msgs::progressUpdate>("/calibration_HMI/progressUpdate"); // update progress in HMI bridge
        hmi_calibration_srv = nh_.advertiseService("/calibration/progressUpdate", &heading::hmiService,this);

        // Timer
        auto_calibration_timer = nh_.createTimer(ros::Duration(15), &heading::autoCalibrate, this); // every 15 secs # was 30 sec
        imu_pub_timer = nh_.createTimer(ros::Duration(1.0/20.0), &heading::publishNewIMU, this); // 10Hz 
    } 

    bool heading::hmiService(antobot_devices_msgs::progressUpdate::Request &req, antobot_devices_msgs::progressUpdate::Response &res ){
        ROS_INFO("hmi auto service called %d",req.progressCode);
        hmi_auto_button_pressed = true;
        hmi_auto_button_time =ros::Time::now();
        return true;
    }  

    // class methods

    std::vector<double> heading::eulerFromQuat(tf2::Quaternion q){
        // Description: Convert tf2::Quaternion to euler and return vector<double> that is filled with euler values. 
        // Input: tf2::Quaternion q
        // Output: vector with euler value - roll, pitch, yaw 

        tf2::Matrix3x3 m(q);
        double r,p,y;
        m.getRPY(r,p,y);
        std::vector<double> v;
        v.push_back(r);
        v.push_back(p);
        v.push_back(y);
        return v; 
    }

    tf2::Quaternion heading::quatFromEuler(double a, double b, double c){
        // Description: Convert euler values to tf2::Quaternion 
        // Input: euler value - roll, pitch, yaw 
        // Returns: tf2::Quaternion

        tf2::Quaternion q;
        q.setRPY(a,b,c);
        q =q.normalize();
        return q;
    }

    bool heading::saveStartGPS(void){
        // Description: Save gps point as the starting point if the robot is moving. Set direction depending on the linear x velocity.
        // Robot wheel odometry's linear x should be larger than lin_tol and the imu angular z velocity should be smaller than angular_zero_tol.
        // direction is set to true when the robot is moving forward.
        // Returns: True if the start GPS point is saved, False if the robot velcoity didn't meet the conditons and failed to save the start GPS point.

        double vel = wheel_odom_v; 
        if ((abs(imu_ang_vel_z) < angular_zero_tol) && (abs(vel) > lin_tol)&& (abs(robot_cmd_vel.linear.x)>lin_tol)){
            gps_start.clear();
            gps_start.push_back(utm_x);
            gps_start.push_back(utm_y); // utm zone is not used for calculation - check needed
            if (vel > 0){
                direction = true; // forward
            }
            else{
                direction = false; // backward
            }
            return true;
        }   
        else{
            return false;
        }
    }

    int heading::checkCondition(void){
        // Description: Check several conditions in order to start calibration and returns the state that will be used in the while loop that called this function.
        // Condition: 1. imu angular z velocity should be smaller than angular_zero_tol.
        //            2. Robot's direction (forward/backward) should be consistent and hasn't change since the saving of the staring GPS point.
        //            3. Robot has moved more than calib_distance (calculated based on gps lat/long data)
        // Returns: state that will be used in the while loop that called this function
        //          state 1: Reset and save another GPS start point
        //          state 2: Distance moved is not enough, continue to call checkCondition
        //          state 3: Distance moved is larger than calib_distance, now exit the while loop and use the gps_yaw in the next step of the calibration.

        // Reset if angular vel is large
        if (abs(imu_ang_vel_z) > angular_zero_tol){
            ROS_DEBUG("Reset due to angular velocity");
            return 1;
        }

        bool current_dir; 
        // Reset if direction is changed
        if (wheel_odom_v > lin_tol){
            current_dir = true; // forward
        }
        else if (abs(wheel_odom_v) <= lin_tol){
            current_dir = direction; // very small velocity - hasn't changed the direction
        } 
        else{
            current_dir = false; // backward
        }
            
        if (current_dir != direction){
            ROS_DEBUG("Reset due to change in direction");
            return 1;
        }

        // Check the distance
        gps_end.clear();
        gps_end.push_back(utm_x);
        gps_end.push_back(utm_y); 

        double dx, dy, d;
        if (direction){ // forward
            dy = gps_end[1] - gps_start[1];
            dx = gps_end[0] - gps_start[0];
        } 
        else{ // backward
            dy = gps_start[1] - gps_end[1];
            dx = gps_start[0] - gps_end[0];
        }     

        d = sqrt(dx*dx + dy*dy);   // distance between start and end points of calibration
        gps_yaw = atan2(dy, dx);   // calculate yaw from gps start and end points

        if (d >= calib_distance){
            ROS_DEBUG("calibration distance satisfied");
            return 3;
        }
        else{
            return 2; // not satisfied
        }
    }

    void heading::checkInputs(void){
        // Description: Check if the imu, gps data are recieved and check if the RTK status is fixed mode value

        while ((!imu_received) || (!gps_received)){
            ROS_INFO("Waiting for initial imu and gps data");
            ros::Duration(0.1).sleep();
            ros::spinOnce(); // Initial Calibration runs before spin() in main loop
        }
            
        ROS_INFO("IMU and gps value received, check RTK status");

        while (rtk_status != rtk_target_status){
            ROS_INFO("Waiting for RTK status to become %d",rtk_target_status);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
            
        // ROS_INFO("RTK status is %d",rtk_target_status);

        while (dual_gps && !heading_received){
            ROS_INFO("Waiting for dual GPS heading to be received");
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
            
        ROS_INFO("GPS, IMU, RTK all ready - start calibration");
    }

    bool heading::dualGPSHeadingCalibration(void){
        // only if the recevied heading is up to date
        if ((ros::Time::now() - heading_received_time).toSec() < 1){ 
                // gps_yaw is dual gps heading in -pi and +pi 
            float gps_yaw_tmp = double(dual_gps_heading.data)*M_PI/180.0;

            // Adjust the angle to be within -PI to +PI
            if (gps_yaw_tmp > M_PI) {
                gps_yaw_tmp -= 2.0 * M_PI;
            }
            gps_yaw = gps_yaw_tmp;
            
            if (imu_calibration_status < 0){ // gps_end used only for the initial calibration 
                // gps_end save
                gps_end.clear();
                gps_end.push_back(utm_x);
                gps_end.push_back(utm_y); 
            }
            return true;
            
        }
        else{
            ROS_INFO("dual GPS heading value not updated");
            return false;
        }

    }

    void heading::initialCalibration(void){
        // Description: Function that is called once in the main function. When all the required topics are recieved (self.checkInputs)
        // this function performs the initial calibration. In the while loop, the starting gps postion is saved and when the 
        // Calibration conditions are met, the imu_offset is calculated. 

        // Check if all input toics are being published 
        checkInputs();

        int state = 1;
        
        // update HMI progress : state 1 - ready for calibration
        antobot_devices_msgs::progressUpdate hmi_req;
        hmi_req.request.progressCode = 1;
        hmi_srv.call(hmi_req);
        ROS_INFO("HMI progress updated - state 1 (ready for calibration)");

        if (dual_gps){
            ros::Duration(2).sleep(); // prevent instant HMI screen change
            ROS_INFO("Heading calibration using dual gps");
            while (dualGPSHeadingCalibration()== false){
                ros::Duration(0.5).sleep(); 
                ros::spinOnce(); 
            };
            state = 3; 
        }
        
        while (true){
            ros::spinOnce();
            if (! dual_gps && hmi_auto_button_pressed){
                hmi_auto_button_pressed = false; 
                // update HMI progress : state 2 - In auto calibration
                antobot_devices_msgs::progressUpdate hmi_req;
                hmi_req.request.progressCode = 2;
                hmi_srv.call(hmi_req);
                ROS_INFO("HMI progress updated - state 2 (In auto calibration) - Robot will move forward");
                state = 4;
            } // If moving for set distance fail - send fail progress 4

            if (state == 1){
                // moving forward/backward, no angular : set start gps point
                if (saveStartGPS()){
                    state = 2;
                }
            }
            else if (state == 2){
                state = checkCondition(); // returns the state
            }
            else if (state == 3){ // calibration satisfied
                sensor_msgs::NavSatFix gpsMsg;
                gpsMsg.header.stamp = ros::Time::now();
                gpsMsg.header.frame_id = gps_frame;
                UTMtoLL(gps_end[1],gps_end[0],utm_zone,gpsMsg.latitude,gpsMsg.longitude);
                
                nh_.setParam("/GPS_origin/latitude", gpsMsg.latitude);
                nh_.setParam("/GPS_origin/longitude", gpsMsg.longitude);
                break;
            }
            else if (state ==4){ // auto calibration
                state = autoCalibration_hmi(); // return 3 when success, return to 1 when failed
            }
            ros::Duration(0.1).sleep(); 
        }

        ROS_DEBUG("gps_yaw is %f", gps_yaw); 
        ros::spinOnce();
        tf2::Quaternion quat_tf;
        // Convert  ROS Quaternion to tf2 Quaternion msg
        tf2::convert(q_imu , quat_tf);
        // Convert from Quaternion to euler
        std::vector<double> result = eulerFromQuat(quat_tf); // result[2] is yaw angle
        // imu_yaw + imu_offset = gps_yaw
        imu_offset = calculateDifference(gps_yaw, result[2]); // difference between orientations from imu and gps
        // ROS_DEBUG("calibration successful (offset = %f degs)", imu_offset/M_PI*180.0); // wrt map x frame

        // Let heading launcher know the inital calibration is finished 
        imu_calibration_status = 1;
        std_msgs::UInt8 msg;
        msg.data = imu_calibration_status;
        pub_calib.publish(msg);
        ROS_INFO("MV2100: Initial calibration finished, (offset = %f degs)", imu_offset/M_PI*180.0);

        // update HMI progress : state 3 - calibration success

        hmi_req.request.progressCode = 3;
        hmi_srv.call(hmi_req);
        ROS_INFO("HMI progress updated - state 3 (ready for job)");

        std_srvs::Trigger srv;
        ekf_srv.call(srv);

    }

    double heading::autoCalibration_hmi(void){
        // auto calibration triggered by HMI button
        
        // save start GPS
        gps_start.clear();
        gps_start.push_back(utm_x);
        gps_start.push_back(utm_y); 
        ROS_INFO("HMI auto-calibration: start GPS saved");
        geometry_msgs::Twist tmp;

        std_msgs::UInt8 switch_tmp;
        switch_tmp.data = 3;
        pub_switch.publish(switch_tmp);
        ROS_INFO("HMI auto-calibration: Change switch mode to autonomous mode");

        

        bool success = false;
        ROS_INFO("HMI auto-calibration: Enter while loop (Try auto calibration for 15secs)");
        while (true){
        
            double duration = (ros::Time::now()- hmi_auto_button_time).toSec();
            if (duration > 15){
                ROS_INFO("HMI auto-calibration: Time out (15s)- Fail");

                // update HMI progress : state 4 - calibration fail
                antobot_devices_msgs::progressUpdate hmi_req;
                hmi_req.request.progressCode = 4;
                hmi_srv.call(hmi_req);
                ROS_INFO("HMI progress updated - state 4 (auto calibration fail)");

                break;
            }
            // TODO: Fail if joystick/teleop is triggered - back to state 1 & pub 4

            ros::spinOnce(); 

            // check obstacle - if True, don't publish cmd_vel 
            if (checkObstacle()){
                ROS_INFO("HMI auto-calibration: Obstacle found - Cmd vel 0.0");
                tmp.linear.x = 0.0; 
            }
            else{
                if (!costmap_received){
                    ROS_INFO("HMI auto-calibration: costmap not received! - Cmd vel 0.0");
                    tmp.linear.x = 0.0;
                }
                else{
                    ROS_INFO("HMI auto-calibration: Obstacle Not found - Cmd vel 0.3");
                    tmp.linear.x = 0.3;
                }
                
            }
            pub_cmd_vel.publish(tmp);   
            ros::Duration(0.5).sleep();

            ros::spinOnce();

            // Check the distance
            gps_end.clear();
            gps_end.push_back(utm_x);
            gps_end.push_back(utm_y); 

            
            double dx, dy, d;
            dy = gps_end[1] - gps_start[1];
            dx = gps_end[0] - gps_start[0];
            d = sqrt(dx*dx + dy*dy);   // distance between start and end points of calibration
            
            if (d >= calib_distance){
                ROS_INFO("HMI auto-calibration: calibration distance satisfied (1m)");
                gps_yaw = atan2(dy, dx);   // calculate yaw from gps start and end points
                success = true;
                tmp.linear.x = 0.0;
                pub_cmd_vel.publish(tmp);
                break;
            }
            else{
                ROS_INFO("HMI auto-calibration: calibration distance(1m) Not satisfied %f",d);
            }

        }

        if (success){
            return 3; // success
        }
        else{
            return 1; // back to default mode
        }


    }

    bool heading::checkObstacle(void){
        // if obstacle is detected - will update after looking at Graph obstacle detection
        // return true
        if (footprint_received && costmap_received){ // Assuming the costmap is bigger than the footprint
            // w (x) to check - only front of the robot for 1m
            int tmp_x = int(width/2 + footprint_x/resolution);
            int tmp_check = std::min(tmp_x + int(1.0/resolution),width);
            // h (y) to check - width of the robot 
            int tmp_y_low = int(height/2 - footprint_y/resolution);
            int tmp_y_high = int(height/2 + footprint_y/resolution);

            bool obstacle_found = false;

            //ROS_INFO("%d %d %d %d", tmp_x, tmp_check, tmp_y_low, tmp_y_high); // this is in cell unit
            for(int x = tmp_x; x <tmp_check; x++){
                for(int y = tmp_y_low; y <tmp_y_high; y++){
                    //ROS_INFO("x %d y %d value %d", x, y, costmap_msg[x][y]);
                    if (costmap_msg[x][y] != 0){
                        ROS_INFO("obstacle found at x %d y %d",x, y );
                        obstacle_found = true;
                        break;
                    }
                }
                if(obstacle_found){
                    break;
                }
            }

            if(obstacle_found){
                return true;
            }
            else{
                return false;
            }


        }
        else{
            ROS_INFO("footprint and costmap not recieved - obstacle not checked");
            return false; 
        }

        
    }

    // Normalize an angle to the range [-pi, pi]
    double heading::normalize_angle(double angle) {
        angle = fmod(angle + M_PI, 2 * M_PI);  // Wrap within the range [0, 2*pi)
        if (angle < 0) {
            angle += 2 * M_PI; // Make sure it’s in [0, 2*pi]
        }
        return angle - M_PI;  // Shift to the range [-pi, pi]
    }

    double heading::calculateDifference(double input_a, double input_b){
        // Description: Compare two values in [-pi,pi] and return signed value in the same range
        // Input: Two double values in [-pi,pi]
        // Returns: Difference between two input values in [-pi,pi] 
        double diff = input_a - input_b;
        return normalize_angle(diff);
    }

    void heading::autoCalibrate(const ros::TimerEvent& event){
        // Description: Function that is called every 30 seconds to check if the calibration is needed. 
        // In the while loop, several conditions are checked and if theses conditions are met within 20 seconds, the new imu_offset is calculated (calibration)
        
        ROS_DEBUG("AutoCalibration called");
        ros::Time started_time =ros::Time::now();

        if (imu_calibration_status == 1 && odometry_received){
            ROS_INFO("auto calibration called but skip one time"); // Since the initial calibration was done recently
            imu_calibration_status = 2;
        }  
        else if (imu_calibration_status == 2){
            ROS_INFO("auto calibration checking started");
            int state = 0;

            while(true){
                ros::spinOnce();
                if (state == 0){ // Check rtk status and ekf odometry
                    //if (rtk_status == rtk_target_status && odometry_received){
                    if (rtk_status == rtk_target_status && odometry_received){   //1 = float; 3 = fix
                        if (dual_gps){ // dual GPS doesn't require calculating the heading based on the GPS position
                            if (dualGPSHeadingCalibration()){
                                state = 3;
                            } // if dual gps heading is not updated - state remains 0
                            else{
                                ros::Duration(1).sleep(); 
                            }
                            
                        }
                        else{
                            state = 1;
                        }
                    }
                }
                else if (state == 1){ // Save start gps position
                    if (saveStartGPS()){ // If the first gps position is saved
                        state = 2;
                    }
                }
                else if (state == 2){ // Check if the auto calibration conditions are met
                    state= checkCondition(); // returns the state
                }
                else if (state == 3){ // Conditions met, start the calibration
                    // gps angle calculated 
                    ROS_DEBUG("gps angles = %f", gps_yaw);

                    tf2::Quaternion quat_tf_imu;
                    tf2::convert(q_imu , quat_tf_imu);
                    std::vector<double> result_imu = eulerFromQuat(quat_tf_imu); // returns r,p,y

                    tf2::Quaternion quat_tf_odom;
                    tf2::convert(q_odom , quat_tf_odom);
                    std::vector<double> result_odom = eulerFromQuat(quat_tf_odom);

                    // imu_yaw + imu_offset = gps_yaw
                    // Compare two angles in [-pi,pi] and returns signed value in radian
                    double diff = calculateDifference(gps_yaw,result_odom[2]);
                    ROS_DEBUG("diff = %f", diff);
                    double diff_deg = abs(diff*180.0/M_PI);
                    ROS_DEBUG("angle diff = %f",diff_deg);

                    if (diff_deg > calib_deg){
                        double imu_offset_tmp = calculateDifference(gps_yaw, result_imu[2]); // difference between orientations from imu and gps
                        imu_offset = imu_offset_tmp;
                        ROS_INFO("MV2200: Auto-calibration complete (imu offset = %f degs)", imu_offset / M_PI * 180.0);
                    }
                    else{
                        ROS_INFO("auto-calibration not required %f deg",diff_deg);
                    }
                    break; // Auto calibration finished
                }

                double duration = (ros::Time::now()- started_time).toSec();
                if (duration > 10.0){ // Every time Auto calibration is called, it will check for 10 seconds. #was 20 sec
                    ROS_INFO("auto calibration conditions not met - cancelled");
                    break;
                }
                ros::Duration(0.1).sleep();
            }
        }
        else{
            ROS_DEBUG("auto calibration called but ignored - Do initial calibration first!");
        }

    }

    void heading::publishNewIMU(const ros::TimerEvent& event){
        // Description: Publish calibrated imu data (/imu/dadta_coreccted) and a topic for debugging purpose (/imu/data_offset)
        // Input: imu raw data and calculated imu_offset. imu_offset is added to the yaw value of the imu data. 

        if (imu_calibration_status > 0){
            tf2::Quaternion quat_tf;
            geometry_msgs::Quaternion quat_msg;

            // Convert  ROS Quaternion to tf2 Quaternion msg
            tf2::convert(q_imu , quat_tf);
            // Convert from Quaternion to euler
            std::vector<double> result = eulerFromQuat(quat_tf);
            // Add imu offset and convert back to Quaternion  
            tf2::Quaternion result_tf =  quatFromEuler(result[0],result[1],result[2]+imu_offset);

            std_msgs::Float32 heading_z_msg;
            heading_z_msg.data = result[2]+imu_offset;
            pub_imu_z.publish(heading_z_msg);

            // Convert tf2 Quaternion to ROS Quaternion
            tf2::convert(result_tf ,quat_msg);
            
            // create imu message
            sensor_msgs::Imu imuMsg;
            imuMsg.header.stamp = ros::Time::now();
            imuMsg.header.frame_id = imu_frame;
            imuMsg.orientation.x = quat_msg.x;
            imuMsg.orientation.y = quat_msg.y;
            imuMsg.orientation.z = quat_msg.z;
            imuMsg.orientation.w = quat_msg.w;
            
            imuMsg.angular_velocity.z = imu_ang_vel_z;  // for EKF inupt

            // publish /imu/data_corrected
            pub_imu.publish(imuMsg);

            // Publsih /imu/data_offset
            std_msgs::Float32 imu_offset_msg;
            imu_offset_msg.data = (float)imu_offset;
            pub_imu_offset.publish(imu_offset_msg);
        }       
    }

    // Callback functions

    void heading::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
        // Description: GPS callback function that gets utm coordinates(from lat, long values) and rts status

        // UTMNorthing, UTMEasting, UTMZone
        LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone); //convert from gps to utm coordinates  
        rtk_status = msg->status.status;
        if (!gps_received){
            gps_received = true;
        }
    }

    void heading::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
        // Description: IMU callback function (IMU sensor raw data)

        q_imu = msg->orientation;
        imu_ang_vel_z = msg->angular_velocity.z; //value used for checking angular vel
        if (!imu_received){
            imu_received = true;
        }
    }

    void heading::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // Description: EKF odometry topic callback function.

        //EKF odometry orientation is the same as calibrated imu's orientation
        q_odom = msg->pose.pose.orientation; // value to compare with gps yaw degree
        if (!odometry_received){
            odometry_received = true;
        }
    }

    void heading::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        // Description: Wheel odometry callback function that gets linear x velocity 

        wheel_odom_v = msg->twist.twist.linear.x; // value used for checking linear vel
    }

    void heading::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        // Description: costmap callback function  
        // TODO: check if the costmap is empty due to no lidar or no lidar frame

        if (!costmap_received){
            costmap_received = true;
            width = msg->info.width;
            height = msg->info.height;
            resolution = msg->info.resolution;
            costmap_msg.resize(width,std::vector<int>(height));
        }

        if (imu_calibration_status == -1){ // update costmap only during the initial calibration
            for (unsigned int x = 0; x < width; x++){
                for (unsigned int y = 0; y < height; y++){
                    //ROS_INFO("%d",msg->data[x+ width * y]);
                    costmap_msg[x][y]=(msg->data[x+ width * y]);
                }
            }
        }
           
    }

    void heading::footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg){
        if (!footprint_received){
            footprint_received = true;
            //Assumming footprint points are 
            footprint_x = abs(msg->polygon.points[0].x);
            footprint_y = abs(msg->polygon.points[0].y);
            //ROS_INFO("%f %f",footprint_x, footprint_y);
        }
        
    }

    void heading::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
        robot_cmd_vel = *msg;
        //ROS_INFO("%f %f",robot_cmd_vel.linear.x, robot_cmd_vel.angular.z);
    }

    void heading::headingCallback(const std_msgs::Float64::ConstPtr& msg){
        if (!heading_received){
            heading_received = true;
        }
        dual_gps_heading = *msg; // ENU 0 to 360 deg 
        heading_received_time = ros::Time::now();
    }
    
}
