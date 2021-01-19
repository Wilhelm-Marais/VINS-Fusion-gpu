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

#include "globalOpt.h"
#include "Factors.h"
#include <stdio.h>

GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
	initPress = false;
	initCompass = false;
    newGPS = false;
	newDepth = false;
	newCompass = false;
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

double GlobalOptimization::pressure_to_depth(double pressure, double density, double p_0)
{
	double depth = (pressure- p_0)/density;
	return depth;
}

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;
    Eigen::Vector3d gravVec(0.,0.,9.8);
    Eigen::Vector3d gravB = OdomQ.conjugate() * gravVec;
    vector<double> gravBody{gravB.x(), gravB.y(), gravB.z(), 0.1};
    gravMap[t] = gravBody;

    //add local odometry to global pose
    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy[3])
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy[0], posAccuracy[1], posAccuracy[2]};
	GPSPositionMap[t] = tmp;
    newGPS = true;

}

void GlobalOptimization::inputCompass(double t, Eigen::Vector3d mag_field, double mag_var[9])
{
	newCompass = true;
    if(!initCompass){
        mag_w[0] = mag_world_norm[0];
        mag_w[1] = mag_world_norm[1];
        mag_w[2] = mag_world_norm[2];
	imu_q_c[0] = imu_q_compass.w();
	imu_q_c[1] = imu_q_compass.x();
	imu_q_c[2] = imu_q_compass.y();
	imu_q_c[3] = imu_q_compass.z();

        initCompass = true;
	printf("mag_w: %f %f %f \n",mag_w[0],mag_w[1],mag_w[2]);
    }
    float mag_size = mag_field.norm();
    double var;
    if(mag_var[0] < 0.01){
	mag_var[0] = 0.1;
    }
    if(fabs(mag_size-1.0) < 0.1){
        var = mag_var[0];
    } else {
        var = 10*mag_var[0];
    }
    Eigen::Vector3d mag_field_unit = mag_field.normalized();
    //printf("mag_field: %f %f %f \n",mag_field_unit[0],mag_field_unit[1],mag_field_unit[2]);
    vector<double> tmp{mag_field_unit[0], mag_field_unit[1], mag_field_unit[2], var};
    compassMap[t] = tmp;
}

void GlobalOptimization::inputPressure(double t, double pressure, double pressure_var)
{   
    //double density = 9.80638;
	if(!initPress){
		p_0 = 0.0;
		imu_t_depth[0] = body_t_depth[0];
		imu_t_depth[1] = body_t_depth[1];
		imu_t_depth[2] = body_t_depth[2];
        initPress = true;
		/* if(localPoseMap.size() > 0){
            initPress = true;
            double d_0 = localPoseMap.begin()->second[2];
            p_0 += d_0 * density; 
            //printf("p_0 intialise to %f at intial depth %f at intial pressure %f \n",p_0,d_0,pressure);
        }*/
	}
    
    if(initPress){
	    depth = pressure_to_depth(pressure, density, p_0);
	    vector<double> tmp{-depth, pressure_var/density};
	    printf("depth: %f \n",depth);
	    depthMap[t] = tmp;
	    newDepth = true;
    }
}


void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS || newDepth || newCompass)
        {
            //newGPS = false;
            printf("global optimization GPS: %d, Depth: %d, Compass: %d \n", newGPS, newDepth, newCompass);
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS, iterDepth, iterDepthLower, iterDepthUpper, iterCompass, iterGrav;
            int i = 0;
            int num_depth = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
                }
                if(newGPS)
		        {
					//gps factor
                	double t = iterVIO->first;
                	iterGPS = GPSPositionMap.lower_bound(t);
                	if (iterGPS != GPSPositionMap.end() && fabs(t - iterGPS->first) < 0.1)
                	{
                    	ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                      iterGPS->second[2], iterGPS->second[3], iterGPS->second[4], iterGPS->second[5]);
                    	//printf("inverse weight %f \n", iterGPS->second[3]);
                    	problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                	}
				}
				if(newDepth)
				{
                    //depth factor
                    double t = iterVIO->first;
                    iterDepth = depthMap.lower_bound(t);
                    if (iterDepth != depthMap.end() && fabs(t - iterDepth->first) < 0.1)
                    {
			    		//printf("depth input with dt = %f\n",fabs(t - iterDepth->first));

                        ceres::CostFunction* depth_function = DepthError::Create(iterDepth->second[0], iterDepth->second[1]);
                        problem.AddResidualBlock(depth_function, loss_function, q_array[i], t_array[i]);

                    }

            	}
				if(newCompass)
				{
                    //compass factor
                    double t = iterVIO->first;
                    iterCompass = compassMap.lower_bound(t);
		    		//printf("compass dt = %f",fabs(t - iterCompass->first));
                    if (iterCompass != compassMap.end() && fabs(t - iterCompass->first) < 0.1)
                    {
			    		//printf("compass input with dt = %f\n",fabs(t - iterCompass->first));
                        iterGrav = gravMap.find(t);
                        
                        ceres::CostFunction* compass_function = CompassError::Create(iterCompass->second[0], iterCompass->second[1],
                                                                      iterCompass->second[2], iterCompass->second[3], iterGrav->second[0],
                                                                      iterGrav->second[1], iterGrav->second[2], iterGrav->second[3]);
                        problem.AddResidualBlock(compass_function, loss_function, q_array[i]);

                    }


				}
			}
            newGPS = false;
            newDepth = false;
            //printf("num_depth: %d \n", depthMap.size());
            //printf("num odom: %d \n", length);
            newCompass = false;
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}
