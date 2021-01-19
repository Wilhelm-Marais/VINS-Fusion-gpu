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

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"
//#include "../../vins_estimator/src/estimator/parameters.h"

using namespace std;


extern int USE_GPS;
extern int USE_COMPASS;
extern int USE_PRESSURE;

extern float density;
extern Eigen::Vector3f mag_world;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy[3]);
	void inputPressure(double t, double pressure, double pressure_var);
	void inputCompass(double t, Eigen::Vector3d mag_field, double mag_var[9]);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::Path global_path;
	float mag_world_size;
	Eigen::Vector3f mag_world_norm;
	Eigen::Vector3f body_t_depth;
	Eigen::Quaterniond imu_q_compass;
	double density;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	double pressure_to_depth(double pressure, double density, double p_0);
	void optimize();
	void updateGlobalPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;
	map<double, vector<double>> depthMap;
    map<double, vector<double>> compassMap;
	map<double, vector<double>> gravMap;
    double depth;
	double p_0;
	bool initGPS;
	bool initPress;
	bool initCompass;
	bool newGPS;
	bool newDepth;
	bool newCompass;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
        Eigen::Vector3d mag_field_global_unit;
	std::thread threadOpt;

};
