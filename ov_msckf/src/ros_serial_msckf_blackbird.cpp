/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2020 Yulin Yang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"

#include <quadrotor_msgs/MotorRPM.h>


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz;


// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "run_serial_msckf_blackbird");
    ros::NodeHandle nh("~");

    // Create our VIO system
    VioManagerOptions params = parse_ros_nodehandler(nh);
    sys = new VioManager(params);
    viz = new RosVisualizer(nh, sys);


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our camera topics (left and right stereo)
    std::string topic_imu;
    std::string topic_camera0, topic_camera1;
    nh.param<std::string>("topic_imu", topic_imu, "/blackbird/imu");

    std::string topic_bb_gt_pose;
    std::string topic_bb_rpm;
    nh.param<std::string>("topic_bb_gt_pose", topic_bb_gt_pose, "/blackbird/state");
    nh.param<std::string>("topic_bb_rpm", topic_bb_rpm, "/blackbird/rotor_rpm");

    // Location of the ROS bag we want to read in
    std::string path_to_bag;
    nh.param<std::string>("path_bag", path_to_bag, "/home/lin/BlackbirdDatasetData/bentDice/yawForward/maxSpeed3p0/rosbag.bag");

    // Load groundtruth if we have it
    std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
    if (nh.hasParam("path_gt")) {
        std::string path_to_gt;
        nh.param<std::string>("path_gt", path_to_gt, "");
        DatasetReader::load_gt_file(path_to_gt, gt_states);
        ROS_INFO("gt file path is: %s", path_to_gt.c_str());
    }
    std::map<double, Eigen::Matrix<double, 7, 1>> gt_poses;


    // Get our start location and how much of the bag we want to play
    // Make the bag duration < 0 to just process to the end of the bag
    double bag_start, bag_durr;
    nh.param<double>("bag_start", bag_start, 0);
    nh.param<double>("bag_durr", bag_durr, -1);
    ROS_INFO("bag start: %.1f",bag_start);
    ROS_INFO("bag duration: %.1f",bag_durr);

    // IMU data vec and rotors time vec
    std::vector<double> times_imu;
    std::vector<Eigen::Vector3d> wms, ams;
    std::vector<double> times_rotor;
    std::vector<Eigen::VectorXd> rotors;


    // Read in what mode we should be processing in (1=mono, 2=stereo)
    int max_cameras;
    nh.param<int>("max_cameras", max_cameras, 1);
    std::string path_to_img_foler;
    std::string path_to_left_img;
    std::string path_to_left_img_time;
    std::string path_to_right_img;
    std::string path_to_right_img_time;
    nh.param<std::string>("path_img", path_to_img_foler, "/home/lin/BlackbirdDatasetData/bentDice/yawForward/maxSpeed3p0/Ancient_Asia_Museum_Room/");
    path_to_left_img = path_to_img_foler + "Camera_Left_Gray/lossless.mov";
    path_to_left_img_time = path_to_img_foler + "Camera_Left_Gray/nSecTimestamps.txt";
    path_to_right_img = path_to_img_foler + "Camera_Right_Gray/lossless.mov";
    path_to_right_img_time = path_to_img_foler + "Camera_Right_Gray/nSecTimestamps.txt";
    // Create an input filestream
    std::ifstream left_img_time_file(path_to_left_img_time.c_str());
    std::ifstream right_img_time_file(path_to_right_img_time.c_str());
    // Make sure the file is open
    if(!left_img_time_file.is_open())  ROS_ERROR("Left image time file is not found!");
    if(!right_img_time_file.is_open())  ROS_ERROR("Right image time file is not found!");

    // camera time vec
    std::vector<double> times_left_img;
    std::vector<double> times_right_img;

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(path_to_bag, rosbag::bagmode::Read);


    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view_full;
    rosbag::View view;

    // Start a few seconds in from the full view time
    // If we have a negative duration then use the full bag length
    view_full.addQuery(bag);
    ros::Time time_init = view_full.getBeginTime();
    time_init += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
    ROS_INFO("time start = %.6f", time_init.toSec());
    ROS_INFO("time end   = %.6f", time_finish.toSec());
    view.addQuery(bag, time_init, time_finish);

    // Check to make sure we have data to play
    if (view.size() == 0) {
        ROS_ERROR("No messages to play on specified topics.  Exiting.");
        ros::shutdown();
        return EXIT_FAILURE;
    }


    // Buffer variables for our system (so we always have imu to use)
    bool has_left = false;
    bool has_right = false;
    cv::Mat img0, img1;
    cv::Mat img0_buffer, img1_buffer;
    double time = time_init.toSec();
    double time_buffer = time_init.toSec();


    //===================================================================================
    //===================================================================================
    //===================================================================================


    // Step through the rosbag
    for (const rosbag::MessageInstance& m : view) {

        // If ros is wants us to stop, break out
        if (!ros::ok())
            break;

        // Handle IMU measurement
        sensor_msgs::Imu::ConstPtr s2 = m.instantiate<sensor_msgs::Imu>();
        if (s2 != nullptr && m.getTopic() == topic_imu) {
            // convert into correct format
            double timem = (*s2).header.stamp.toSec();
            Eigen::Matrix<double, 3, 1> wm, am;
            wm << (*s2).angular_velocity.x, (*s2).angular_velocity.y, (*s2).angular_velocity.z;
            am << (*s2).linear_acceleration.x, (*s2).linear_acceleration.y, (*s2).linear_acceleration.z;
            // send it to our VIO system
            times_imu.push_back(timem); wms.push_back(wm); ams.push_back(am);
            // sys->feed_measurement_imu(timem, wm, am);
            // std::cout << std::setprecision(12) << timem << std::endl;
            // viz->visualize_odometry(timem);
        }


        // Handle MAV rpm measurements
        // Handle quadrotor pose message
        boost::shared_ptr<quadrotor_msgs::MotorRPM> s3 = m.instantiate<quadrotor_msgs::MotorRPM>();
        if(s3 != nullptr && m.getTopic() == topic_bb_rpm){
            // convert to the right format
            double timem = s3->header.stamp.toSec();
            Eigen::VectorXd rpm = Eigen::MatrixXd::Zero(s3->rpm.size(),1);
            for(int i=0; i<s3->rpm.size(); i++){
                rpm(i) = s3->rpm[i] * 2*3.1415926/60;
            }
            times_rotor.push_back(timem);
            rotors.push_back(rpm);
        }


        // Handle quadrotor gt pose message
        geometry_msgs::PoseStampedConstPtr ptr2 = m.instantiate<geometry_msgs::PoseStamped>();
        if(ptr2 != nullptr && m.getTopic() == topic_bb_gt_pose){
            // convert to the right format
            double timem = ptr2->header.stamp.toSec();
            Eigen::Matrix<double, 4, 1> qm; // q_M_to_G
            Eigen::Matrix<double, 3, 1> pm; // p_M_in_G
            qm << ptr2->pose.orientation.x, ptr2->pose.orientation.y, ptr2->pose.orientation.z, ptr2->pose.orientation.w;
            pm << ptr2->pose.position.x, ptr2->pose.position.y, ptr2->pose.position.z;
            // wrap the poses
            Eigen::Matrix<double,7,1> posem; posem << pm, qm;
            // store gt poses
            gt_poses.insert({timem, posem});
        }
    }

    // load the left camera time vector
    double temp_time;
    string line;
    while(left_img_time_file.good()){
        std::getline(left_img_time_file, line);
        std::stringstream ss(line);
        ss >> temp_time;
        times_left_img.push_back(temp_time/1.0e9);
    }
    // load the right camera time vector
    if(max_cameras == 2 ){
        while(right_img_time_file.good()){
            std::getline(right_img_time_file, line);
            std::stringstream ss(line);
            ss >> temp_time;
            times_right_img.push_back(temp_time/1.0e9);
        }
    }

    // setup the video file reading
    cv::VideoCapture cap_left(path_to_left_img.c_str());
    if(!cap_left.isOpened()) ROS_ERROR("Left image file is not found!");
    std::cout << cap_left.get(CV_CAP_PROP_FRAME_COUNT) << std::endl;
    std::cout << times_left_img.size() << std::endl;
    assert (cap_left.get(CV_CAP_PROP_FRAME_COUNT) == times_left_img.size()-1);
    cv::VideoCapture cap_right(path_to_right_img.c_str());
    if(max_cameras == 2) {
        if(!cap_right.isOpened()) ROS_ERROR("Right image file is not found!");
        assert(times_left_img.size() == times_right_img.size());
        assert(cap_right.get(CV_CAP_PROP_FRAME_COUNT) == times_right_img.size());
    }

    int skip = 3;
    int ct_img = 0;
    // now begin to process the measurements
    for(size_t i=0; i<times_left_img.size()-1; i++){

        if(ct_img == skip) {
            std::cout << "processing the " << i << "-th image!" << std::endl;
            time = times_left_img.at(i+1);

            // process the imu first
            double time_imu = times_imu.front();
            std::cout << time_imu << std::endl;
            std::cout << time << std::endl;
            while (time_imu <= time){
                 sys->feed_measurement_imu(time_imu, wms.front(), ams.front());
                 times_imu.erase(times_imu.begin()); wms.erase(wms.begin()); ams.erase(ams.begin());
                 time_imu = times_imu.front();
            }

            // process the rotors meas
            double time_rotor = times_rotor.front();
//            while(time_rotor <= time){
//
//            }


            // read left image
            cap_left.set ( CV_CAP_PROP_POS_FRAMES , i );
            cv::Mat img_temp;
            has_left = cap_left.read(img_temp);
            cvtColor(img_temp, img0, CV_BGR2GRAY);
            // read right image
            if(max_cameras == 2){
                cap_right.set(CV_CAP_PROP_POS_FRAMES, i);
                has_right = cap_right.read(img1);
            }

            // Fill our buffer if we have not
            if(has_left && img0_buffer.rows == 0) {
                has_left = false;
                time_buffer = time;
                img0_buffer = img0.clone();
            }

            // Fill our buffer if we have not
            if(has_right && img1_buffer.rows == 0) {
                has_right = false;
                img1_buffer = img1.clone();
            }


            // If we are in monocular mode, then we should process the left if we have it
            if(max_cameras==1 && has_left) {
                // process once we have initialized with the GT
                Eigen::Matrix<double, 17, 1> imustate;
                if(!gt_states.empty() && !sys->initialized() && DatasetReader::get_gt_state(time_buffer, imustate, gt_states)) {
                    //biases are pretty bad normally, so zero them
                    //imustate.block(11,0,6,1).setZero();
                    sys->initialize_with_gt(imustate);
                } else if(gt_states.empty() || sys->initialized()) {
                    std::cout << img0_buffer.cols << ", " << img0_buffer.rows << std::endl;
                    std::cout << std::setprecision(12) << time_buffer << std::endl;
                    sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
                    // visualize
                    viz->visualize();
                    // reset bools
                    has_left = false;
                    // move buffer forward
                    time_buffer = time;
                    img0_buffer = img0.clone();

                }

            }


            // If we are in stereo mode and have both left and right, then process
            if(max_cameras==2 && has_left && has_right) {
                // process once we have initialized with the GT
                Eigen::Matrix<double, 17, 1> imustate;
                if(!gt_states.empty() && !sys->initialized() && DatasetReader::get_gt_state(time_buffer, imustate, gt_states)) {
                    //biases are pretty bad normally, so zero them
                    //imustate.block(11,0,6,1).setZero();
                    sys->initialize_with_gt(imustate);
                } else if(gt_states.empty() || sys->initialized()) {
                    sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
                    ct_img = 0;
                    // visualize
                    viz->visualize();
                    // reset bools
                    has_left = false;
                    has_right = false;
                    // move buffer forward
                    time_buffer = time;
                    img0_buffer = img0.clone();
                    img1_buffer = img1.clone();

                }

            }
            ct_img = 0;
        }else{
            ct_img ++;
        }

    }





    // Final visualization
    viz->visualize_final();

    // Finally delete our system
    delete sys;
    delete viz;


    // Done!
    return EXIT_SUCCESS;

}


















