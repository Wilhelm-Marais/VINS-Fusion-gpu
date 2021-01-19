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
#include <ceres/ceres.h>
#include <ceres/rotation.h>


double mag_w[3];
double imu_q_c[4];
double imu_t_depth[3];


template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};

struct DepthError
{
	DepthError(double depth, double var)
		:depth(depth), var(var){}

	template <typename T>
	bool operator()(const T* w_q_i, const T* zj, T* residuals) const
	{
        //residuals[0] = (tj[0] - T(depth))*0.0;
        //residuals[1] = (tj[1] - T(depth))*0.0;
	    T i_t_depth[3];
		T w_i_t_depth[3];

		i_t_depth[0] = T(imu_t_depth[0]);
		i_t_depth[1] = T(imu_t_depth[1]);
		i_t_depth[2] = T(imu_t_depth[2]);
		ceres::QuaternionRotatePoint(w_q_i, i_t_depth, w_i_t_depth);
				
		residuals[0] = (zj[2] + w_i_t_depth[2] - T(depth))/ T(var);

	    return true;
	}
        static ceres::CostFunction* Create(const double depth, const double var)
        {
          return (new ceres::AutoDiffCostFunction<
                  DepthError, 1, 4, 3>(
                        new DepthError(depth, var)));
        }

        double depth, var;
};

struct CompassError
{
        CompassError(double mag_x, double mag_y, double mag_z,double var, double grav_x,double grav_y,double grav_z,double var_g)
                                  :mag_x(mag_x), mag_y(mag_y), mag_z(mag_z), var(var), grav_x(grav_x), grav_y(grav_y), grav_z(grav_z),var_g(var_g){}

	    template <typename T>
        bool operator()(const T* w_q_i, T* residuals) const
        {
		    T c_q_w[4];
			T w_q_c[4];
			T i_q_c[4];
            T i_q_w[4];

			i_q_c[0] = T(imu_q_c[0]);
			i_q_c[1] = T(imu_q_c[1]);
			i_q_c[2] = T(imu_q_c[2]);
			i_q_c[3] = T(imu_q_c[3]);

			ceres::QuaternionProduct(w_q_i, i_q_c, w_q_c);
            QuaternionInverse(w_q_c, c_q_w);
		    QuaternionInverse(w_q_i, i_q_w);

		    T mag_c[3];
			T mag_world[3];
            T grav_world[3];
            T grav_i[3];
			mag_world[0] = T(mag_w[0]);
			mag_world[1] = T(mag_w[1]);
			mag_world[2] = T(mag_w[2]);
            grav_world[0] = T(0.0);
            grav_world[1] = T(0.0);
            grav_world[2] = T(9.8);

            ceres::QuaternionRotatePoint(c_q_w, mag_world, mag_c);
            ceres::QuaternionRotatePoint(i_q_w, grav_world, grav_i);

            residuals[0] = (mag_c[0] - T(mag_x)) / T(var);
            residuals[1] = (mag_c[1] - T(mag_y)) / T(var);
            residuals[2] = (mag_c[2] - T(mag_z)) / T(var);
            residuals[3] = (grav_i[0] - T(grav_x)) / T(var_g);
            residuals[4] = (grav_i[1] - T(grav_y)) / T(var_g);
            residuals[5] = (grav_i[2] - T(grav_z)) / T(var_g);


            return true;
        }

    	static ceres::CostFunction* Create(const double mag_x, const double mag_y, const double mag_z, const double var, const double grav_x, const double grav_y, const double grav_z, const double var_g)
        {
          return (new ceres::AutoDiffCostFunction<
                  CompassError, 6, 4>(
                        new CompassError(mag_x, mag_y, mag_z, var, grav_x, grav_y, grav_z, var_g)));
        }

        double mag_x, mag_y, mag_z, var, grav_x, grav_y, grav_z, var_g;


};


struct TError
{
	TError(double t_x, double t_y, double t_z, double var_x, double var_y, double var_z)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var_x(var_x), var_y(var_y), var_z(var_z){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{

		residuals[0] = (tj[0] - T(t_x)) / T(var_x);
		residuals[1] = (tj[1] - T(t_y)) / T(var_y);
		//residuals[2] = (tj[2] - T(t_z)) / T(var_z);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var_x, const double var_y, const double var_z) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 2, 3>(
	          	new TError(t_x, t_y, t_z, var_x, var_y, var_z)));
	}

	double t_x, t_y, t_z, var_x, var_y, var_z;

};

struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		T t_i_ij[3];
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};
