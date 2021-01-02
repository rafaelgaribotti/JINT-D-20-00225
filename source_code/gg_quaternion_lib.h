#ifndef __GG_QUATERNION_LIB_H__
#define __GG_QUATERNION_LIB_H__

/*******************************************************************************************
 * Interface for some operations on Quaternions.
 * All operations are prefixed with 'gg' to avoid name clashes.
 *
 * @author Rodrigo Alves Medeiros
 ******************************************************************************************/
/****************************************INCLUDE*******************************************/
	#include "matrix.h"

/****************************************FUNCTIONS*****************************************/
	void gg_ea2q(Matrix q, Matrix const ea);                    // OK

	void gg_q2r(Matrix RS, Matrix const qS);                    // OK

	void gg_qconj(Matrix qc, Matrix const q);                   // OK

	void gg_q2v(Matrix v, Matrix const q);                      // OK

	void gg_qtimes(Matrix qy, Matrix const qr, Matrix const q); // OK

	void gg_q2w(Matrix w, Matrix const q, double const T);      // OK

	void gg_norm_q(Matrix q_norm, Matrix const q);              // OK

	/** void gg_qdot(Matrix qdot, Matrix const q, double Ts); TEM-SE A gg_xdot() **/

#endif
