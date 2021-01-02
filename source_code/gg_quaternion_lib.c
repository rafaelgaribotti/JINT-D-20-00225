/*********************************************************************************************************
 * Interface for some operations on Quaternions.
 * All operations are prefixed with 'gg' to avoid name clashes.
 *
 * @author Rodrigo Alves Medeiros
 ********************************************************************************************************/
/************************************************INCLUDE*************************************************/
#include "gg_quaternion_lib.h"

/************************************************FUNCTIONS***********************************************/
void gg_ea2q(Matrix q, Matrix const ea){
    assert(q.cols == ea.cols);
    assert(q.rows == 4);
    assert(ea.rows == 3);
    int i;

    Matrix cang, sang;
    cang = alloc_matrix(3, 1, 3, 1);
    sang = alloc_matrix(3, 1, 3, 1);

    for(i = 0; i < cang.rows; i++){
        cang.data[i][0] = cos(ea.data[i][0]/2);
        sang.data[i][0] = sin(ea.data[i][0]/2);
    }

    for(i = 0; i < ea.cols; i++){
        q.data[0][i] = ((-sang.data[0][i])*sang.data[1][i]*sang.data[2][i])\
            +(cang.data[0][i]*cang.data[1][i]*cang.data[2][i]);
        q.data[1][i] = (  sang.data[0][i] *cang.data[1][i]*cang.data[2][i])\
            +(sang.data[1][i]*sang.data[2][i]*cang.data[0][i]);
        q.data[2][i] = ((-sang.data[0][i])*sang.data[2][i]*cang.data[1][i])\
            +(sang.data[1][i]*cang.data[0][i]*cang.data[2][i]);
        q.data[3][i] = (  sang.data[0][i] *sang.data[1][i]*cang.data[2][i])\
            +(sang.data[2][i]*cang.data[0][i]*cang.data[1][i]);
    }

    free_matrix(cang);
    free_matrix(sang);
}

void gg_q2r(Matrix RS, Matrix const qS){
    assert(RS.rows == RS.cols);
    assert(qS.rows == 4);
    assert(qS.cols == 1);

    RS.data[0][0] = pow(qS.data[0][0], 2) + pow(qS.data[1][0], 2) - pow(qS.data[2][0], 2)\
        - pow(qS.data[3][0], 2);
    RS.data[0][1] = 2*(qS.data[1][0] * qS.data[2][0] - qS.data[0][0] * qS.data[3][0]);
    RS.data[0][2] = 2*(qS.data[1][0] * qS.data[3][0] + qS.data[0][0] * qS.data[2][0]);

    RS.data[1][0] = 2*(qS.data[1][0] * qS.data[2][0] + qS.data[0][0] * qS.data[3][0]);
    RS.data[1][1] = pow(qS.data[0][0], 2) - pow(qS.data[1][0], 2) + pow(qS.data[2][0], 2)\
        - pow(qS.data[3][0], 2);
    RS.data[1][2] = 2*(qS.data[2][0] * qS.data[3][0] - qS.data[0][0] * qS.data[1][0]);

    RS.data[2][0] = 2*(qS.data[1][0] * qS.data[3][0] - qS.data[0][0] * qS.data[2][0]);
    RS.data[2][1] = 2*(qS.data[2][0] * qS.data[3][0] + qS.data[0][0] * qS.data[1][0]);
    RS.data[2][2] = pow(qS.data[0][0], 2) - pow(qS.data[1][0], 2) - pow(qS.data[2][0], 2)\
        + pow(qS.data[3][0], 2);
}

void gg_qconj(Matrix qc, Matrix const q){
    assert(qc.rows == q.rows);
    assert(qc.cols == q.cols);
    int i, j;

    for(i = 0; i < qc.rows; i++){
        for(j = 0; j < qc.cols; j++){
            if(i == 0)
                qc.data[i][j] = q.data[i][j];
            else
                qc.data[i][j] = -q.data[i][j];
        }
    }
}

void gg_q2v(Matrix v, Matrix const q){
    assert(v.cols == q.cols);
    int i, j;

    for(i = 1; i < q.rows; i++)
        for(j = 0; j < q.cols; j++)
            v.data[i-1][j] = q.data[i][j];
}

void gg_qtimes(Matrix qy, Matrix const qr, Matrix const q){
    assert(qr.rows == q.rows);
    assert(qr.cols == q.cols);
    assert(qy.rows == q.rows);
    assert(qy.cols == q.cols);
    int i;

    for(i = 0; i < q.cols; i++){
        qy.data[0][i] = qr.data[0][i] * q.data[0][i] - qr.data[1][i] * q.data[1][i]\
            - qr.data[2][i] * q.data[2][i] - qr.data[3][i] * q.data[3][i];
        qy.data[1][i] = qr.data[0][i] * q.data[1][i] + qr.data[1][i] * q.data[0][i]\
            - qr.data[2][i] * q.data[3][i] + qr.data[3][i] * q.data[2][i];
        qy.data[2][i] = qr.data[0][i] * q.data[2][i] + qr.data[1][i] * q.data[3][i]\
            + qr.data[2][i] * q.data[0][i] - qr.data[3][i] * q.data[1][i];
        qy.data[3][i] = qr.data[0][i] * q.data[3][i] - qr.data[1][i] * q.data[2][i]\
            + qr.data[2][i] * q.data[1][i] + qr.data[3][i] * q.data[0][i];
    }
}

void gg_q2w(Matrix w, Matrix const q, double const T){
    assert(w.cols == q.cols);
    assert(w.rows == 3);
    assert(q.rows == 4);
    Matrix qdot, qc, qy;

    qdot = alloc_matrix(q.rows, q.cols, q.rows, q.cols);
    qc = alloc_matrix(q.rows, q.cols, q.rows, q.cols);
    qy = alloc_matrix(q.rows, q.cols, q.rows, q.cols);

	gg_qconj(qc, q);
    gg_xdot(qdot, q, T);
	gg_qtimes(qy, qc, qdot);
	scale_matrix(qy, 2);
	gg_q2v(w, qy);
}

void gg_norm_q(Matrix q_norm, Matrix const q){
    assert(q_norm.rows == q.rows);
    assert(q_norm.cols == q.cols);
    double sum = 0, sqrtsum = 0;
    int i;

//    printf("\nq:\n");
//    print_matrix(q);

    sum = pow(q.data[0][0],2) + pow(q.data[1][0],2) + pow(q.data[2][0],2) + pow(q.data[3][0],2);
    sqrtsum = sqrt(sum);

//    printf("\nqnorm\n");
//    printf("%.15f", sqrtsum);
//    printf("\n");

    for(i = 0; i < q.rows; i++)
        q_norm.data[i][0] = q.data[i][0]/sqrtsum;
//    printf("\nq_norm:\n");
//    print_matrix(q_norm);
}
