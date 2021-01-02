/************************************************************************************************
 * Functions to compute filter states of an extended kalman filter.
 * All operations are prefixed with 'ram' to avoid name clashes.
 *
 * @author Rodrigo Alves Medeiros
 ***********************************************************************************************/
/********************************************INCLUDE********************************************/
    #include "gg_ekf.h"
    #include "common.h"
    #include "Board_GLCD.h"
    #include "CM3DS_MPS2.h"
    #include "GLCD_Config.h"
 
/********************************************FUNCTIONS*****************************************/
    void free_filter(gg_ekf_ctx ctx){
        free_matrix(ctx.g);
        free_matrix(ctx.Qx);
        free_matrix(ctx.Qz);
        free_matrix(ctx.oC);
        free_matrix(ctx.KC);
        free_matrix(ctx.RS);
        free_matrix(ctx.RC);
        free_matrix(ctx.Fp);
    }

    void gg_eekf_fusion(Matrix xf, Matrix P, gg_ekf_ctx const ctx,
            Matrix const gyr_m, Matrix const acc_m, Matrix const cam_m){
        
        // static uint16_t count_filtro = 1;
        // printf("\nFiltro:\t%d.", count_filtro);

        int i, j, i_cam, loc;
        // // TESTE
        // static int line = 8;
        // float time;
        // char buffu[100];
        // static unsigned int counter_start, counter_end;
        /************************************PREDICTION_STEP***********************************/
            /***********************************pre_computation********************************/
                Matrix q, w, wa, p, v, a, bg, ba, e, Se, Sw, Fw, Gq;
                double n;

                q  = alloc_matrix(4, 1, 4, 1);
                w  = alloc_matrix(3, 1, 3, 1);
                wa = alloc_matrix(3, 1, 3, 1);
                p  = alloc_matrix(3, 1, 3, 1);
                v  = alloc_matrix(3, 1, 3, 1);
                a  = alloc_matrix(3, 1, 3, 1);
                bg = alloc_matrix(3, 1, 3, 1);
                ba = alloc_matrix(3, 1, 3, 1);
                e = alloc_matrix(3, 1, 3, 1);
                Se = alloc_matrix(3, 3, 3, 3);
                Sw = alloc_matrix(3, 3, 3, 3);
                Fw = alloc_matrix(4, 4, 4, 4);
                Gq = alloc_matrix(4, 3, 4, 3);

                for(i = 0; i < q.rows; i++)
                    q.data[i][0] = xf.data[i][0];
                for(i = 0; i < w.rows; i++)
                    w.data[i][0] = xf.data[(i+4)][0];
                for(i = 0; i < wa.rows; i++)
                    wa.data[i][0] = xf.data[(i+7)][0];
                for(i = 0; i < p.rows; i++)
                    p.data[i][0] = xf.data[(i+10)][0];
                for(i = 0; i < v.rows; i++)
                    v.data[i][0] = xf.data[(i+13)][0];
                for(i = 0; i < a.rows; i++)
                    a.data[i][0] = xf.data[(i+16)][0];
                for(i = 0; i < bg.rows; i++)
                    bg.data[i][0] = xf.data[(i+19)][0];
                for(i = 0; i < ba.rows; i++)
                    ba.data[i][0] = xf.data[(i+22)][0];

                n = q.data[0][0];
                for(i = 0; i < e.rows; i++)
                    e.data[i][0] = q.data[(i+1)][0];

                gg_ss(Se, e);
                for(i = 0; i < Se.rows; i++) Se.data[i][i] += n;
                gg_ss(Sw, w);

                for(i = 1; i < Fw.rows; i++){
                    Fw.data[i][0] =  w.data[i-1][0];
                    Fw.data[0][i] = -w.data[i-1][0];
                }

                for(i = 0; i < Sw.rows; i++)
                    for(j = 0; j < Sw.cols; j++)
                        Fw.data[(i+1)][(j+1)] = -Sw.data[i][j];

                scale_matrix(Fw, 0.5);

                for(i = 0; i < Gq.cols; i++)
                    Gq.data[0][i] = -e.data[i][0];

                for(i=0; i < Se.rows; i++)
                    for(j=0; j < Se.cols; j++)
                        Gq.data[i+1][j] = Se.data[i][j];
                scale_matrix(Gq, 0.5);

               // printf("\nFw:\n");
               // print_matrix(Fw);
               // printf("\nGq:\n");
               // print_matrix(Gq);

               free_matrix(Se);
               free_matrix(e);

            /***********************************model_parameters*******************************/
                Matrix Ac, A, fc, Fwq;

                Ac = alloc_matrix(25, 25, 25, 25);
                A  = alloc_matrix(25, 25, 25, 25);
                fc = alloc_matrix(25, 1, 25, 1);
                Fwq = alloc_matrix(4, 1, 4, 1);

                for(i=0; i<Fw.rows; i++)
                    for(j=0; j<Fw.cols; j++)
                        Ac.data[i][j] = Fw.data[i][j];

                for(i=0; i<Gq.rows; i++)
                    for(j=0; j<Gq.cols; j++)
                        Ac.data[i][j+4] = Gq.data[i][j];

                for(i=0; i<3; i++){
                    Ac.data[i+4][i+7] = 1;
                    Ac.data[i+10][i+13] = 1;
                    Ac.data[i+13][i+16] = 1;
                }
               // printf("\nAc:\n");
               // print_matrix(Ac);

                for(i=0; i<A.rows; i++)
                    A.data[i][i] = 1;

                scale_matrix(Ac, ctx.Tc);

                for(i=0; i<A.rows; i++)
                    for(j=0; j<A.cols; j++)
                        A.data[i][j] += Ac.data[i][j];

               free_matrix(Ac);

                multiply_matrix(Fw, q, Fwq);

                for(i=0; i<Fwq.rows; i++)
                    fc.data[i][0] = Fwq.data[i][0];

                for(i=0; i<3; i++){
                    fc.data[i+4][0] = wa.data[i][0];
                    fc.data[i+10][0] = v.data[i][0];
                    fc.data[i+13][0] = a.data[i][0];
                }

               // printf("\nA:\n");
               // print_matrix(A);

            /***********************************extentend_kalman_filter_prediction_step********/
                Matrix PAt;

                PAt  = alloc_matrix(25, 25, 25, 25);

                scale_matrix(fc, ctx.Tc);

                for(i=0; i<xf.rows; i++)
                    for(j=0; j<xf.cols; j++)
                        xf.data[i][j] += fc.data[i][j];

                multiply_by_transpose_matrix(P, A, PAt);
                multiply_matrix(A, PAt, P);

                for(i=0; i<ctx.Qx.rows; i++)
                    for(j=0; j<ctx.Qx.cols; j++)
                        P.data[i][j] += ctx.Qx.data[i][j];

               free_matrix(PAt);

               // printf("\nP:\n");
               // print_matrix(P);

            /***********************************quaternion_normalization***********************/
                Matrix qn;

                qn = alloc_matrix(4, 1, 4, 1);

                for(i=0; i<qn.rows; i++)
                    q.data[i][0] = xf.data[i][0];

                gg_norm_q(qn, q);

               // printf("\nq norm:\n");
               // print_matrix(qn);

                for(i = 0; i < qn.rows; i++)
                    xf.data[i][0] = qn.data[i][0];

               // free_matrix(qn);
               // printf("\nx:\n");
               // print_matrix(xf);

        /************************************UPDATE_STEP***************************************/
            /***********************************pre_computation********************************/
                Matrix Swa, So_imu, ag, Rq, Hc, Swt, Sw2, Jqw, JqagHc, Jwo_imu, agHco_imu,\
                             woimut, oimuwt, wtoimui, wt, scl;

                ag = alloc_matrix(3, 1, 3, 1);
                Rq = alloc_matrix(3, 3, 3, 3);
                Hc = alloc_matrix(3, 3, 3, 3);
                Swt = alloc_matrix(3, 3, 3, 3);
                Sw2 = alloc_matrix(3, 3, 3, 3);
                Swa = alloc_matrix(3, 3, 3, 3);
                Jqw = alloc_matrix(3, 4, 3, 4);
                So_imu = alloc_matrix(3, 3, 3, 3);
                JqagHc = alloc_matrix(3, 4, 3, 4);
                Jwo_imu = alloc_matrix(3, 3, 3, 3);
                woimut = alloc_matrix(3, 3, 3, 3);
                oimuwt = alloc_matrix(3, 3, 3, 3);
                wtoimui = alloc_matrix(3, 3, 3, 3);
                wt = alloc_matrix(1, 3, 1, 3);
                scl = alloc_matrix(1, 1, 1, 1);
                agHco_imu = alloc_matrix(3, 1, 3, 1);

                for(i = 0; i < q.rows; i++)
                    q.data[i][0] = xf.data[i][0];
                for(i = 0; i < w.rows; i++)
                    w.data[i][0] = xf.data[(i+4)][0];
                for(i = 0; i < wa.rows; i++)
                    wa.data[i][0] = xf.data[(i+7)][0];
                for(i = 0; i < p.rows; i++)
                    p.data[i][0] = xf.data[(i+10)][0];
                for(i = 0; i < v.rows; i++)
                    v.data[i][0] = xf.data[(i+13)][0];
                for(i = 0; i < a.rows; i++)
                    a.data[i][0] = xf.data[(i+16)][0];
                for(i = 0; i < bg.rows; i++)
                    bg.data[i][0] = xf.data[(i+19)][0];
                for(i = 0; i < ba.rows; i++)
                    ba.data[i][0] = xf.data[(i+22)][0];

                for(i = 0; i < ag.rows; i++)
                    ag.data[i][0] = a.data[i][0] + ctx.g.data[(i)][0];

                gg_ss(Sw, w);
                gg_ss(Swa, wa);
                gg_ss(So_imu, ctx.oC);

                gg_q2r(Rq, q);

                /// SEGUNDO O MATLAB NÃO USA ATRANSPOSTA AO MULTIPLICAR AO QUADRADO VER COM O PROFESSOR
                // transpose_matrix(Sw, Swt);
                copy_matrix(Sw, Swt);
                multiply_matrix(Sw, Swt, Sw2);

                for(i=0; i<Hc.rows; i++)
                    for(j=0; j<Hc.cols; j++)
                        Hc.data[i][j] = Sw2.data[i][j] + Swa.data[i][j];

                gg_jacobian_rtu(Jqw, q, w);

                multiply_matrix(Hc, ctx.oC, agHco_imu);
                for(i = 0; i < agHco_imu.rows; i++)
                    agHco_imu.data[i][0] += ag.data[i][0];
                gg_jacobian_rtu(JqagHc, q, agHco_imu);

                multiply_by_transpose_matrix(w, ctx.oC, woimut);
                multiply_by_transpose_matrix(ctx.oC, w, oimuwt);
                scale_matrix(oimuwt, -2);
                set_identity_matrix(wtoimui);
                transpose_matrix(w, wt);
                multiply_matrix(wt, ctx.oC, scl);
                scale_matrix(wtoimui, scl.data[0][0]);
                for(i=0; i<Jwo_imu.rows; i++)
                    for(j=0; j<Jwo_imu.cols; j++)
                        Jwo_imu.data[i][j] = woimut.data[i][j] + oimuwt.data[i][j]\
                                                + wtoimui.data[i][j];

                free_matrix(woimut);
                free_matrix(oimuwt);
                free_matrix(wtoimui);
                free_matrix(wt);
                free_matrix(scl);
                free_matrix(Sw2);
                free_matrix(Swt);

            /***********************************model_parameters*******************************/
                Matrix z, h, C, Qz, aux_0, aux_1;

                z = alloc_matrix(6, 1, 14, 1);
                h = alloc_matrix(6, 1, 14, 1);
                C = alloc_matrix(6, 25, 14, 25);
                Qz = alloc_matrix(6, 6, 14, 14);
                aux_0 = alloc_matrix(3, 3, 3, 4);
                aux_1 = alloc_matrix(3, 1, 25, 25);

                for(i = 0; i < gyr_m.rows; i++)
                    z.data[i][0] = gyr_m.data[i][0];
                for(i = 0; i < acc_m.rows; i++)
                    z.data[i+3][0] = acc_m.data[i][0];

                multiply_matrix(ctx.RS, w, aux_1);
                for(i = 0; i < bg.rows; i++)
                   h.data[i][0] = aux_1.data[i][0] + bg.data[i][0];

                multiply_by_transpose_matrix(ctx.RS, Rq, aux_0);
                multiply_matrix(aux_0, agHco_imu, aux_1);
                for(i = 0; i < ba.rows; i++)
                   h.data[i+3][0] = aux_1.data[i][0] + ba.data[i][0];

                aux_0.rows = 3;
                aux_0.cols = 4;
                multiply_matrix(ctx.RS, JqagHc, aux_0);

                for(i=0; i<aux_0.rows; i++)
                    for(j=0; j<aux_0.cols; j++)
                        C.data[i+3][j] = aux_0.data[i][j];

                aux_1.rows = 3;
                aux_1.cols = 3;
                multiply_by_transpose_matrix(ctx.RS, Rq, aux_1);

                for(i=0; i<aux_1.rows; i++){
                    for(j=0; j<aux_1.cols; j++){
                        C.data[i][j+4] = aux_1.data[i][j];
                        C.data[i+3][j+16] = aux_1.data[i][j];
                    }
                }

                aux_0.rows = 3;
                aux_0.cols = 3;
                multiply_matrix(aux_1, Jwo_imu, aux_0);

                for(i=0; i<aux_0.rows; i++)
                    for(j=0; j<aux_0.cols; j++)
                        C.data[i+3][j+4] = aux_0.data[i][j];

                scale_matrix(So_imu, -1);
                multiply_matrix(aux_1, So_imu, aux_0);

                for(i=0; i<aux_0.rows; i++)
                    for(j=0; j<aux_0.cols; j++)
                        C.data[i+3][j+7] = aux_0.data[i][j];

                for(i=0; i<6; i++)
                    C.data[i][i+19] = 1;

                for(i=0; i<Qz.rows; i++)
                    for(j=0; j<Qz.cols; j++)
                        Qz.data[i][j] = ctx.Qz.data[i][j];

            static uint8_t flag_cam = 0;
            /***********************************cam********************************************/
                if(cam_m.data[0][0] >= 0 && cam_m.data[1][0] >= 0){
                    flag_cam = 1;
                    Matrix Spxi, Jqi, Jpi, CC, cam_aux, SpxiKcRc, aux_2;

                    Spxi = alloc_matrix(3, 3, 3, 3);
                    Jqi  = alloc_matrix(3, 4, 3, 4);
                    Jpi  = alloc_matrix(3, 3, 3, 3);
                    CC   = alloc_matrix(2, 3, 2, 3);
                    aux_2   = alloc_matrix(3, 1, 4, 4);
                    cam_aux  = alloc_matrix(3, 1, 3, 1);
                    SpxiKcRc = alloc_matrix(3, 3, 3, 3);

                    z.rows = 14;
                    h.rows = 14;
                    C.rows = 14;
                    Qz.rows = 14;
                    Qz.cols = 14;

                    aux_0.rows = 3;
                    aux_0.cols = 1;
                    aux_1.rows = 3;
                    aux_1.cols = 3;

                    for(i=0; i<Qz.rows; i++)
                        for(j=0; j<Qz.cols; j++)
                            Qz.data[i][j] = ctx.Qz.data[i][j];

                    for(i=0; i<CC.rows; i++)
                        CC.data[i][i] = 1;

                    loc = 0;
                    for(i_cam=0; i_cam<cam_m.cols; i_cam++){
                        aux_0.rows = 3;
                        aux_0.cols = 1;
                        aux_1.rows = 3;
                        aux_1.cols = 3;

                        for(i=0; i<cam_m.rows; i++)
                            cam_aux.data[i][0] = cam_m.data[i][i_cam];
                        cam_aux.data[2][0] = 1;

                        for(i=0; i<ctx.Fp.rows; i++)
                            aux_2.data[i][0] = ctx.Fp.data[i][i_cam];

                        gg_ss(Spxi, cam_aux);

                        multiply_matrix(ctx.KC, ctx.RC, aux_1);
                        multiply_matrix(Spxi, aux_1, SpxiKcRc);

                        aux_1.rows = 3;
                        aux_1.cols = 4;
                        for(i=0; i<aux_2.rows; i++)
                            aux_2.data[i][0] -= p.data[i][0];

                        gg_jacobian_rtu(aux_1, q, aux_2);
                        multiply_matrix(SpxiKcRc, aux_1, Jqi);

                        scale_matrix(SpxiKcRc, -1);
                        multiply_by_transpose_matrix(SpxiKcRc, Rq, Jpi);

                        /// para evitar usar outra variavel
                        scale_matrix(Jpi, -1);

                        aux_0.rows = 2;
                        aux_0.cols = 3;
                        multiply_matrix(CC, Jpi, aux_0);

                        aux_1.rows = 2;
                        aux_1.cols = 1;
                        multiply_matrix(aux_0, aux_2, aux_1);

                        /// preenchendo h com dados da camera
                        for(i=0; i<aux_1.rows; i++)
                            h.data[6+loc+i][0] = aux_1.data[i][0];
                        scale_matrix(Jpi, -1);

                        aux_0.rows = 2;
                        aux_0.cols = 3;
                        multiply_matrix(CC, Jpi, aux_0);

                        for(i=0; i<aux_0.rows; i++)
                            for(j=0; j<aux_0.cols; j++)
                                C.data[i+loc+6][j+10] = aux_0.data[i][j];

                        aux_0.rows = 2;
                        aux_0.cols = 4;
                        multiply_matrix(CC, Jqi, aux_0);

                        for(i=0; i<aux_0.rows; i++)
                            for(j=0; j<aux_0.cols; j++)
                                C.data[i+loc+6][j] = aux_0.data[i][j];

                        loc = loc + 2;
                    }
                    free_matrix(Spxi);
                    free_matrix(Jqi);
                    free_matrix(Jpi);
                    free_matrix(CC);
                    free_matrix(cam_aux);
                    free_matrix(SpxiKcRc);
                    free_matrix(aux_2);
                }

            /***********************************extended_kalman_filter_update_step*************/
                Matrix M, S, K, L, S_inv;

                M = alloc_matrix(25, C.rows, 25, 14);
                L = alloc_matrix(25, 25, 25, 25);
                K = alloc_matrix(25, Qz.rows, 25, 14);
                S = alloc_matrix(Qz.rows, Qz.cols, Qz.rows, Qz.cols);
                S_inv = alloc_matrix(Qz.rows, Qz.cols, Qz.rows, Qz.cols);

                // counter_start = CM3DS_MPS2_FPGASYS->COUNTCYC;
                /// M = P*C'
                // multiply_by_transpose_matrix(P, C, M);
                if(flag_cam == 1)
                    multiply2_by_transpose_matrix(P, C, M, 25,14);
                else
                    multiply2_by_transpose_matrix(P, C, M, 25, 6);
                // counter_end = CM3DS_MPS2_FPGASYS->COUNTCYC;
                // time = (float)(counter_end - counter_start)*0.00000004;
                // sprintf(buffu, "Tt: %.06f",time);
                // GLCD_DrawString(START_COL, line++ * FONTS_H, buffu);
                /// S = C*M + Qz
                multiply_matrix(C, M, S);
                for(i=0; i<S.rows; i++)
                    for(j=0; j<S.cols; j++)
                        S.data[i][j] += Qz.data[i][j];
                /// K = M/S
                destructive_invert_matrix(S, S_inv);
                multiply_matrix(M, S_inv, K);
                /// L = eye(size(P)) - K*C
                multiply_matrix(K, C, L);
                scale_matrix(L, -1);
                for(i=0; i<L.rows; i++)
                    L.data[i][i] += 1;
                /// x = x + K*(z - h)
                for(i=0; i<z.rows; i++)
                    z.data[i][0] -= h.data[i][0];
                aux_1.rows = 25;
                aux_1.cols = 1;
                multiply_matrix(K, z, aux_1);
                for(i=0; i<xf.rows; i++)
                    xf.data[i][0] += aux_1.data[i][0];
                /// P = L*P
                aux_1.rows = 25;
                aux_1.cols = 25;
                copy_matrix(P, aux_1);
                multiply_matrix(L, aux_1, P);
            /***********************************quaternion_normalization***********************/
                Matrix qn_;

                qn_ = alloc_matrix(4, 1, 4, 1);

                for(i=0; i<qn.rows; i++)
                    q.data[i][0] = xf.data[i][0];

                gg_norm_q(qn_, q);

                for(i = 0; i < qn_.rows; i++)
                    xf.data[i][0] = qn_.data[i][0];

               free_matrix(qn_);

               if(flag_cam = 1)
                    flag_cam = 0;

            /***********************************free_matrix************************************/
               free_matrix(Swa);
               free_matrix(So_imu);
               free_matrix(ag);
               free_matrix(Rq);
               free_matrix(Hc);
               free_matrix(Jqw);
               free_matrix(JqagHc);
               free_matrix(Jwo_imu);
               free_matrix(agHco_imu);
               free_matrix(M);
               free_matrix(L);
               free_matrix(K);
               free_matrix(S);
               free_matrix(C);
               free_matrix(z);
               free_matrix(h);
               free_matrix(Qz);
               free_matrix(aux_0);
               free_matrix(aux_1);
               free_matrix(S_inv);
               free_matrix(A);
               free_matrix(fc);
               free_matrix(Fwq);
               free_matrix(q);
               free_matrix(w);
               free_matrix(wa);
               free_matrix(p);
               free_matrix(v);
               free_matrix(a);
               free_matrix(bg);
               free_matrix(ba);
               free_matrix(Sw);
               free_matrix(Fw);
               free_matrix(Gq);

               // printf("\nFiltro:\t%d.", count_filtro);
               // count_filtro ++;

    }
