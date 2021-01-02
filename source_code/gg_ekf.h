#ifndef __GG_EKF_H__
#define __GG_EKF_H__

/******************************************************************************************
 * Functions to compute filter states of an extended kalman filter.
 * All operations are prefixed with 'gg' to avoid name clashes.
 *
 * @author Rodrigo Alves Medeiros
 ******************************************************************************************/
/****************************************INCLUDE*******************************************/
	#include "gg_quaternion_lib.h"

/****************************************FUNCTIONS*****************************************/
	typedef struct{
		double Tc;          //!< Sample period
		Matrix g;           //!< Gavitational aceleration vector
		Matrix Qx;          //!< Model covariance [q, w, wa, p, v, a, bg, ba]
		Matrix Qz;          //!< Sensor covariance [gyr, acc, cam1, ..., camN]
		Matrix oC;          //!< IMU offset from rotation center
		Matrix KC;          //!< Camera intrinsic parameters
		Matrix RS;          //!< IMU rotation matrix
		Matrix RC;          //!< Camera rotation matrix
		Matrix Fp;          //!< Features position
	} gg_ekf_ctx;

	void free_filter(gg_ekf_ctx ctx);

	void gg_eekf_fusion(Matrix xf, Matrix P, gg_ekf_ctx const ctx,
	        Matrix const gyr_m, Matrix const acc_m, Matrix const cam_m);

#endif
