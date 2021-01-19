/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/MagneticField.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
//#include "parameters.h"

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;

int compass_counter = 0;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("GPS_callback! \n");
    double t = GPS_msg->header.stamp.toSec();
    //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    //int numSats = GPS_msg->status.service;
    double pos_accuracy[] = {GPS_msg->position_covariance[0]/100.0,GPS_msg->position_covariance[4]/100.0,GPS_msg->position_covariance[8]/100.0};
    //printf("receive covariance %lf \n", pos_accuracy);
    if(!isnan(GPS_msg->latitude)){
		globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
	}
}

void pressure_callback(const sensor_msgs::FluidPressureConstPtr &pressure_msg)
{
    //printf("pressure callback! \n");
	double t = pressure_msg->header.stamp.toSec();
	double pressure = pressure_msg->fluid_pressure;
	double pressure_var = pressure_msg->variance;
    if(pressure_var < 0.1){
        pressure_var = 1.0;
	globalEstimator.inputPressure(t, pressure, pressure_var);
    }
}

void compass_callback(const geometry_msgs::Vector3StampedConstPtr &compass_msg)
{
	compass_counter++;
	double t = compass_msg->header.stamp.toSec();
	//geometry_msgs::Vector3 mag_field = compass_msg->magnetic_field;
	Eigen::Vector3d mag_field(compass_msg->vector.x, compass_msg->vector.y, compass_msg->vector.z);
	double mag_var[9]; 
	mag_var[0] = 0.1;
	//copy(begin(compass_msg->magnetic_field_covariance),end(compass_msg->magnetic_field_covariance), begin(mag_var));
	globalEstimator.inputCompass(t, mag_field, mag_var);
	compass_counter = 0;
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);
	

    //add local odometry to global pose and publish
    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    printf("starting global fusion node \n");
    if(argc != 2)
    {
        printf("please intput: rosrun global_fusion global_fusion_node [config file] \n"
               "for example: rosrun loop_fusion loop_fusion_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }
    int USE_GPS;
    int USE_COMPASS;
    int USE_PRESSURE;
    float density;
    Eigen::Vector3f mag_world;

    std::string GPS_TOPIC;
    std::string PRESSURE_TOPIC;
    std::string COMPASS_TOPIC;
    
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    USE_GPS = fsSettings["use_gps"];
    USE_COMPASS = fsSettings["use_compass"];
    USE_PRESSURE = fsSettings["use_pressure"];
    fsSettings["gps_topic"] >> GPS_TOPIC;
    
    fsSettings["pressure_topic"] >> PRESSURE_TOPIC;
    fsSettings["density"] >> density;
    globalEstimator.density = density;
    cv::Mat body_t_depth;
    fsSettings["body_t_depth"] >> body_t_depth;
    cv::cv2eigen(body_t_depth, globalEstimator.body_t_depth);


    fsSettings["compass_topic"] >> COMPASS_TOPIC;
    cv::Mat mag_field_vector;
    fsSettings["magnetic_field_vector"] >> mag_field_vector;
    cv::Mat body_R_compass;
    Eigen::Matrix3d imu_R_compass;
    fsSettings["body_R_compass"] >> body_R_compass;
    cv::cv2eigen(body_R_compass, imu_R_compass);
    globalEstimator.imu_q_compass = imu_R_compass;

    cv::cv2eigen(mag_field_vector, mag_world);
    globalEstimator.mag_world_size = mag_world.norm();
    globalEstimator.mag_world_norm = mag_world.normalized();

    fsSettings.release();

    global_path = &globalEstimator.global_path;
    
    ros::Subscriber sub_GPS;
    ros::Subscriber sub_depth;
    ros::Subscriber sub_compass;

    if(USE_GPS)
    {
        std::cout << "subscribing to: " << GPS_TOPIC;
        printf("\n");
    	sub_GPS = n.subscribe(GPS_TOPIC, 100, GPS_callback);
    } else {
        printf("no gps \n");
    }
    if(USE_PRESSURE){
        std::cout << "subscribing to: " << PRESSURE_TOPIC;
        printf("\n");
	printf("density: %f\n", globalEstimator.density);
    	sub_depth = n.subscribe(PRESSURE_TOPIC, 1000, pressure_callback);
    } else {
        printf("no depth\n");
    }
    if(USE_COMPASS){
        std::cout << "subscribing to: " << COMPASS_TOPIC;
        printf("\n");
	printf("mag field: %f %f %f \n", mag_world[0], mag_world[1], mag_world[2]);
	    sub_compass = n.subscribe(COMPASS_TOPIC, 1000, compass_callback);
    } else {
        printf("no compass\n");
    }
    printf("subscribing to /vins_estimator/odometry \n");
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    //ros::Subscriber sub_depth = n.subscribe("/bluerov2/pressure", 1000, pressure_callback);
    compass_counter = 0;
    //printf("sub_vio.topic: %s",sub_vio.topic);
    //printf("sub_depth.topic: %s",sub_depth.topic);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    //ros::Duration(0.5).sleep();
    ros::spin();
    return 0;
}
