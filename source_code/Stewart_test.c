/************************************************************************************************************
 * Sensor fusion of a inertial sensors and a camera through EKF.
 *
 * @author Rodrigo Alves Medeiros
 *
 * Obs: Apagar arquivo dos estados estimados na pasta de destino
 * antes de executar o programa.
 ***********************************************************************************************************/
/*******************************************************INCLUDE*********************************************/
    #include <time.h>
    #include <string.h>
    // Bib locais
    #include "gg_ekf.h"
    #include "protocol.h"
    #include "config.h"

    // Bib locais - Dados
    #include "RC.h"
    #include "tC.h"
    #include "KC.h"
    #include "Accel.h"
    #include "Gyro.h"
    #include "mkr.h"
    // Bib locais - Placa
    #include "common.h"
    #include "Board_GLCD.h"
    #include "GLCD_Config.h"
    #include "uart_stdout.h"
    #include "CM3DS_MPS2_driver.h"
    #include "CM3DS_MPS2.h"

/*******************************************************DEFINE**********************************************/
    #define biasSensorN 100//200
    #define MAX_BUFFER  10//4000
    
    /**************FPGA_PARAMETROS_INTERNOS***************/
    #define START_COL 1
    #define FONTS_W 6
    #define FONTS_H 8
    extern GLCD_FONT GLCD_Font_6x8;
    #define FONT_SMALL &GLCD_Font_6x8
    #define FONTL_W 16
    #define FONTL_H 24
    extern GLCD_FONT GLCD_Font_16x24;
    #define FONT_LARGE &GLCD_Font_16x24

/*******************************************************FUNCTIONS*******************************************/
    void stateMachine(uint8_t command, uint8_t *data, uint32_t data_size);
    void gg_UartStdOutInit(void);
    void gg_UartPutc(unsigned char my_ch);
    unsigned char gg_UartGetc(void);

/*******************************************************FLAGS***********************************************/
    volatile uint8_t flag_protocol = 0;
    volatile uint8_t flag_dataProc = 0;
    volatile uint8_t flag_filter = 0;
    volatile uint8_t flag_log = 0;
    volatile uint8_t flag_logErr = 0;
    volatile uint8_t flag_end = 0;

/*******************************************************MAIN************************************************/
    int main(){
        int line = 0;
        uint8_t data_in[BUFFER_SIZE];
        uint8_t data_out[BUFFER_SIZE];
        uint8_t command;
        uint32_t data_size;
        unsigned char c = 0;
        unsigned int counter_start, counter_end;
        float time;
        __clear_terminal();
        char buffu[100];

        // Initialize the Protocol, GLCD, UART0 and UART2:
        protocolInit(ADDR1, ADDR1);
        GLCD_Initialize(); 
        UartStdOutInit();
        gg_UartStdOutInit();

        /* display initial screen */
        GLCD_SetFont(FONT_SMALL);
        GLCD_SetBackgroundColor(GLCD_COLOR_WHITE);
        GLCD_ClearScreen();
        GLCD_SetForegroundColor(GLCD_COLOR_BLACK);

        char * str1 = "---- STEWART TEST ----\n\n";
        GLCD_DrawString(START_COL, line++ * FONTS_H, str1);

        counter_start = CM3DS_MPS2_FPGASYS->COUNTCYC;

        int i=0, j=0, k=0, l=0;
        /***********************************************DATAS***********************************************/
            /**********************************************RC.h*********************************************/
                Matrix RC;
                RC = alloc_matrix(3, 3, 3, 3);

    		    str1 = "RC.h - start!\n";
    		    GLCD_DrawString(START_COL, line++ * FONTS_H, str1);
                // printf(str1);

                // sprintf(buffu, "ROW: %d.\nCOL: %d.", RC.rows, RC.cols);
                // GLCD_DrawString(START_COL, line++ * FONTS_H, buffu);
                // for(i = 0; i < 3; i++){
                //     for(j = 0; j < 3; j++)
                //         printf("%f\t", my_RC[i][j]);
                //     printf("\n");
                // }  
                 
                for(i = 0; i < RC.rows; i++)
                    for(j = 0; j < RC.cols; j++)
                        RC.data[i][j] = my_RC[i][j];
    		    str1 = "RC.h - end!\n\n";
    		    GLCD_DrawString(START_COL, line++ * FONTS_H, str1);
                // printf(str1);

            /**********************************************KC.h*********************************************/
                Matrix KC;
                KC = alloc_matrix(3, 3, 3, 3);

    		    str1 = "KC.h - start!\n";
    		    // printf(str1);
    		    GLCD_DrawString(START_COL, line++ * FONTS_H, str1);
                for(i = 0; i < KC.rows; i++)
                    for(j = 0; j < KC.cols; j++)
                        KC.data[i][j] = my_KC[i][j];
    		    str1 = "KC.h - end!\n\n";
    		    // printf(str1);
    		    GLCD_DrawString(START_COL, line++ * FONTS_H, str1);

            /**********************************************tC.h*********************************************/
                Matrix tC;
                tC = alloc_matrix(3, 1, 3, 1);

    		    str1 = "tC.h - start!\n";
    		    // printf(str1);
    		    GLCD_DrawString(START_COL, line++ * FONTS_H, str1);
                for(i = 0; i < tC.rows; i++)
                    tC.data[i][0] = my_tC[i];
    		    str1 = "tC.h - end!\n\n";
    		    // printf(str1);
    		    GLCD_DrawString(START_COL, line++ * FONTS_H, str1);

        /***********************************************SENSORS*********************************************/
            /**********************************************imu**********************************************/
                Matrix eaS, qS, RS, RSt;

                eaS = alloc_matrix(3, 1, 3, 1);
                qS  = alloc_matrix(4, 1, 4, 1);
                RS  = alloc_matrix(3, 3, 3, 3);
                RSt = alloc_matrix(3, 3, 3, 3);

                eaS.data[2][0] = -pi/2;
                gg_ea2q(qS, eaS);
                gg_q2r(RS, qS);
                transpose_matrix(RS, RSt);
                RSt.data[0][0] = 0.000000000000000222044604925031;
                RSt.data[1][1] = 0.000000000000000222044604925031;

            /**********************************************cam**********************************************/
                Matrix p0, RCt, oC, g, featurePos;
                double xC, yC, zC;

                p0  = alloc_matrix(3, 1, 3, 1);
                oC  = alloc_matrix(3, 1, 3, 1);
                g   = alloc_matrix(3, 1, 3, 1);
                RCt = alloc_matrix(3, 3, 3, 3);
                featurePos  = alloc_matrix(3, 4, 3, 4);

                transpose_matrix(RC, RCt);
                scale_matrix(RCt, -1);
                multiply_matrix(RCt, tC, p0);

                xC = p0.data[0][0];
                yC = p0.data[1][0];
                zC = p0.data[2][0] - pz;

                set_matrix(oC, px+xC,\
                               py+yC,\
                               pz+zC);

                set_matrix(featurePos, px+0.1, px, px-0.1, px,\
                                       py, py-0.1, py, py+0.1,\
                                       0,0,0,0);
                featurePos.data[2][3] = 0.1;
                g.data[2][0] = gValue;

            /**********************************************bias*********************************************/
                Matrix ba, bg, acc_m, gyr_m;

                ba  = alloc_matrix(3, 1, 3, 1);
                bg  = alloc_matrix(3, 1, 3, 1);
                acc_m  = alloc_matrix(3, biasSensorN, 3, biasSensorN);
                gyr_m  = alloc_matrix(3, biasSensorN, 3, biasSensorN);

    			for (i = 0; i < biasSensorN; i++){
    				acc_m.data[0][i] = my_Accel[i][0];
    				acc_m.data[1][i] = my_Accel[i][1];
    				acc_m.data[2][i] = my_Accel[i][2];
    			}

    			for (i = 0; i < biasSensorN; i++){
    				gyr_m.data[0][i] = my_Gyro[i][0];
    				gyr_m.data[1][i] = my_Gyro[i][1];
    				gyr_m.data[2][i] = my_Gyro[i][2];
    			}

                gg_mean(ba, acc_m, biasSensorN);
                ba.data[2][0] -= gValue;

                gg_mean(bg, gyr_m, biasSensorN);

        /***********************************************EXTENDED_KALMAN_FILTER******************************/
            /**********************************************prediction_model_covariance**********************/
                Matrix Qz, Qx;
                double scalar;

                Qz = alloc_matrix(14, 14, 14, 14);
                Qx = alloc_matrix(25, 25, 25, 25);

                if(artigo){
                        for(i =  0; i <  4; i++) Qx.data[i][i] = 0.000000000010869;
                        for(i =  4; i <  7; i++) Qx.data[i][i] = 28780;
                        for(i =  7; i < 10; i++) Qx.data[i][i] = 0.000000000000000011824;
                        for(i = 10; i < 13; i++) Qx.data[i][i] = 25911;
                        for(i = 13; i < 16; i++) Qx.data[i][i] = 60022;
                        for(i = 16; i < 19; i++) Qx.data[i][i] = 0.00000000000014907;
                }else{
                    int kx = -5;
                    scalar = pow(10, kx);

                    for(i = 0; i < 19; i++) Qx.data[i][i] = 0.9999999999999999;
                    scale_matrix(Qx, scalar);
                }

            /**********************************************measurement_update_covariance********************/
                if(artigo){
                    // Covariância gyr
                    Qz.data[0][0] = 0.00337;
                    Qz.data[1][1] = 0.00294;
                    Qz.data[2][2] = 0.01249;
                    // Covariância acc
                    Qz.data[3][3] = 0.0001423;
                    Qz.data[4][4] = 0.0001378;
                    Qz.data[5][5] = 0.0001630;
                }else{
                    // Covariância gyr
                    Qz.data[0][0] = 0.00001733805709951477;
                    Qz.data[1][1] = 0.00001581483854821477;
                    Qz.data[2][2] = 0.00001775463798416693;
                    // Covariância acc
                    Qz.data[3][3] = 0.00005206261666041296;
                    Qz.data[4][4] = 0.00003474762155803392;
                    Qz.data[5][5] = 0.0001671849329391898;
                }

                // Covariância cam
                for(i = 6; i < 14; i++) Qz.data[i][i] = 0.001;

            /**********************************************initial_covariance*******************************/
                Matrix P;

                // Covariância inicial zero
                P = alloc_matrix(25, 25, 25, 25);

            /**********************************************initial_condition********************************/
                Matrix xf;

                xf = alloc_matrix(25, 1, 25, 1);

                // Condições iguais ao do MATLAB
                xf.data[0][0] =  1;
                xf.data[1][0] = -0.00000000000000008923215625647697;
                xf.data[2][0] =  0.00000000000000009471120210330687;
                xf.data[3][0] = -0.00000000000000039810350309597820;

                xf.data[10][0] = 0.6974231583606496;
                xf.data[11][0] = 0.4995296662965494;
                xf.data[12][0] = 0.5575084344976036;

                xf.data[19][0] = 0.04715192415495705;
                xf.data[20][0] = -0.0873896173950604;
                xf.data[21][0] = -0.0110444435066201;

                xf.data[22][0] = 0.08428305644999998;
                xf.data[23][0] = 0.0643171068000000;
                xf.data[24][0] = 0.04454107875000091;

            /**********************************************EKF_Context**************************************/
                gg_ekf_ctx ctx;

                ctx.Tc = Ts;
                ctx.g  = alloc_matrix(g.rows, g.cols, g.rows, g.cols);
                ctx.Qx = alloc_matrix(Qx.rows, Qx.cols, Qx.rows, Qx.cols);
                ctx.Qz = alloc_matrix(Qz.rows, Qz.cols, Qz.rows, Qz.cols);
                ctx.oC = alloc_matrix(oC.rows, oC.cols, oC.rows, oC.cols);
                ctx.KC = alloc_matrix(KC.rows, KC.cols, KC.rows, KC.cols);
                ctx.RS = alloc_matrix(RS.rows, RS.cols, RS.rows, RS.cols);
                ctx.RC = alloc_matrix(RC.rows, RC.cols, RC.rows, RC.cols);
                ctx.Fp = alloc_matrix(featurePos.rows, featurePos.cols, featurePos.rows, featurePos.cols);
                copy_matrix(g, ctx.g);
                copy_matrix(Qx, ctx.Qx);
                copy_matrix(Qz, ctx.Qz);
                copy_matrix(oC, ctx.oC);
                copy_matrix(KC, ctx.KC);
                copy_matrix(RSt, ctx.RS);
                copy_matrix(RC, ctx.RC);
                copy_matrix(featurePos, ctx.Fp);

            //--------TESTE_PARA_UART2_BEGIN----------//
                // char txChar = 's';
                // double teste_inf0, teste_inf1;
                // while(1){
                //     gg_UartPutc(txChar);

                //     for (teste_inf0 = 0; teste_inf0 < 20000; ++teste_inf0){
                //        if(teste_inf0==19999) teste_inf1++;
                //     }
                //     if(teste_inf1==19999) teste_inf1=0;
                // }

                // str1 = "TESTE TERMINADO!\n\n";
                // GLCD_DrawString(START_COL, line++ * FONTS_H, str1);
                
                // return 0;
            //--------TESTE_PARA_UART2_END------------//

            /**********************************************Fusion*******************************************/
                Matrix g_m, a_m, cam_m, cam_bkp;

                char var_t[28];
                char est_out[250];
                // float_num_t data_est[25];
                double_num_t data_est[25];
                int16_t temp[14];
                int8_t cr, cc, count;
                unsigned char end_filter = 'c';

                g_m = alloc_matrix(3, 1, 3, 1);
                a_m = alloc_matrix(3, 1, 3, 1);
                cam_m = alloc_matrix(2, 4, 2, 4);
                cam_bkp = alloc_matrix(2, 4, 2, 4);

                k = 0;
                l = 0;
                // FILE *file0, *file_raw;
                // file0 = fopen("my_estimated_states.txt","w");
                // file_raw = fopen("data_raw.txt","w");

                // line = 0;
                // GLCD_ClearScreen();
                i = 0;

                // TEMPORÁRIO{
                // flag_dataProc = 1;                
                // }

                while(1){
                        flag_protocol = protocolUpdate(UartGetc());
                        
                        if(flag_protocol == true){
                             if(protocolCheckCRC() == true){
                                 // Acquire variables:
                                 command = protocolGetCommand();
                                 data_size = protocolGetDataSize();
                                 protocolGetData(data_in, data_size);
                                 // Execution:
                                 stateMachine(command, data_in, data_size);
                             }else{
                                 sprintf(buffu, "Protocol CRC ERROR.\n");
                                 GLCD_DrawString(START_COL, line++ * FONTS_H, buffu);
                                 flag_logErr = 1;
                             }
                             protocolReset();
                             flag_protocol = 0;
                         }
                        
                         for (k = 0; k < 54; k++){ 
                             data_in[k] = UartGetc();
                             if (k == 53) flag_dataProc = 1;
                         }

                        if(flag_dataProc == 1){
                            // Entradas do gyr:
                            temp[0] = (data_in[1] << 8) | data_in[0];
                            g_m.data[0][0] = (double)temp[0] * 0.00875 * (pi/180);
                            temp[1] = (data_in[3] << 8) | data_in[2];
                            g_m.data[1][0] = (double)temp[1] * 0.00875 * (pi/180);
                            temp[2] = (data_in[5] << 8) | data_in[4];
                            g_m.data[2][0] = (double)temp[2] * 0.00875 * (pi/180);

                            // Entradas do acc:
                            temp[3] = (data_in[7] << 8) | data_in[6];
                            a_m.data[0][0] = (double)temp[3] * 0.061 * (gValue/1000);
                            temp[4] = (data_in[9] << 8) | data_in[8];
                            a_m.data[1][0] = (double)temp[4] * 0.061 * (gValue/1000);
                            temp[5] = (data_in[11] << 8) | data_in[10];
                            a_m.data[2][0] = (double)temp[5] * 0.061 * (gValue/1000);
                            
                            // Entradas da cam:
                            temp[6] = (data_in[13] << 8) | data_in[12];                            
                            cam_m.data[0][0] = (double)temp[6];
                            temp[7] = (data_in[15] << 8) | data_in[14];
                            cam_m.data[1][0] = (double)temp[7];
                            temp[8] = (data_in[17] << 8) | data_in[16];
                            cam_m.data[0][1] = (double)temp[8];
                            temp[9] = (data_in[19] << 8) | data_in[18];
                            cam_m.data[1][1] = (double)temp[9];
                            temp[10] = (data_in[21] << 8) | data_in[20];
                            cam_m.data[0][2] = (double)temp[10];
                            temp[11] = (data_in[23] << 8) | data_in[22];
                            cam_m.data[1][2] = (double)temp[11];
                            temp[12] = (data_in[25] << 8) | data_in[24];
                            cam_m.data[0][3] = (double)temp[12];
                            temp[13] = (data_in[27] << 8) | data_in[26];
                            cam_m.data[1][3] = (double)temp[13];    

                            count = 0;
                            for(cr = 0; cr < 2; cr++)
                                for(cc = 0; cc < 4; cc++)
                                    if(cam_m.data[cr][cc] == -1) count++;
                            
                            if(count > 0){
                               for(cr = 0; cr < 2; cr++)
                                    for(cc = 0; cc < 4; cc++)
                                        cam_m.data[cr][cc] = cam_bkp.data[cr][cc];
                            }else{
                                for(cr = 0; cr < 2; cr++)
                                    for(cc = 0; cc < 4; cc++)
                                        cam_bkp.data[cr][cc] = cam_m.data[cr][cc];
                            }

                            flag_filter = 1;
                            flag_dataProc = 0;
                        }
                        
                        if(flag_filter == 1){
                            printf("\ni = %d.", i);
                            g_m.data[0][0] = my_Gyro[i][0];
                            g_m.data[1][0] = my_Gyro[i][1];
                            g_m.data[2][0] = my_Gyro[i][2];

                            a_m.data[0][0] = my_Accel[i][0];
                            a_m.data[1][0] = my_Accel[i][1];
                            a_m.data[2][0] = my_Accel[i][2];  

                            cam_m.data[0][0] = my_markers[i][0];
                            cam_m.data[1][0] = my_markers[i][1];
                            cam_m.data[0][1] = my_markers[i][2];
                            cam_m.data[1][1] = my_markers[i][3];
                            cam_m.data[0][2] = my_markers[i][4];
                            cam_m.data[1][2] = my_markers[i][5];
                            cam_m.data[0][3] = my_markers[i][6];
                            cam_m.data[1][3] = my_markers[i][7];

                            if(i>0) gg_eekf_fusion(xf, P, ctx, g_m, a_m, cam_m);

                            sprintf(buffu, "Filtro (i) a=  %d", i);
                            GLCD_DrawString(START_COL, line * FONTS_H, buffu);

                            i++;

                            // MANDAR COMANDO DE FILTRO OK //
                            // Send:
                        //     data_size = protocolGetFrame(data_out, ADDR1, REG01R, &c, 1);
                        //     for (l = 0; l < data_size; l++)
                        //         UartPutc(data_out[l]);
                            
                        //     flag_log = 1;
                        //     flag_filter = 0;
                        }
                        
                        if(flag_log == 1){
                            // sprintf("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ", xf.data[0][0],\
                            //     xf.data[1][0],xf.data[2][0],xf.data[3][0],xf.data[4][0],xf.data[5][0],xf.data[6][0],xf.data[7][0],\
                            //     xf.data[8][0],xf.data[9][0],xf.data[10][0],xf.data[11][0],xf.data[12][0],xf.data[13][0],xf.data[14][0],\
                            //     xf.data[15][0],xf.data[16][0],xf.data[17][0],xf.data[18][0],xf.data[19][0],xf.data[20][0],xf.data[21][0],\
                            //     xf.data[22][0],xf.data[23][0],xf.data[24][0]);
                            
                            for(k = 0; k < xf.rows; k++)
                                data_est[k].d = xf.data[k][0];
                            
                            gg_UartPutc((char)START);
                            gg_UartPutc((char)START);
                            for(k = 0; k < 25; k++)
                                for(l = 0; l < 8; l++)
                                    gg_UartPutc((char)data_est[k].c[l]);
                            // gg_UartPutc('\n');

                            // for(k = 0; k < 28; k++)
                            //     gg_UartPutc((char)data_in[k]);
                            // gg_UartPutc('\n');

                            //**FAZER_O_LOG_DOS_DADOS_ESTIMADOS**//

                            // TEMPORÁRIO{
                            // if(i == 255) break;
                            // flag_dataProc = 1;
                            // }

                            flag_log = 0;
                        }

                        flag_dataProc = 1;
                        
                        // if(flag_logErr == 1){
                        //     data_size = protocolGetFrame(data_out, ADDR1, REG01R, &c, 1);
                        //     for (l = 0; l < data_size; l++)
                        //         UartPutc(data_out[l]);

                        //     for(k = 0; k < xf.rows; k++)
                        //         data_est[k].d = -1;
                            
                        //     gg_UartPutc((char)STOP);
                        //     gg_UartPutc((char)STOP);
                        //     for(k = 0; k < 25; k++)
                        //         for(l = 0; l < 8; l++)
                        //             gg_UartPutc((char)data_est[k].c[l]);
                        //     //gg_UartPutc('\n');

                        //     flag_logErr = 0;
                        // }
                        
                        if(flag_end == 1){
                            // fclose(file0);
                            // fclose(file_raw);
                            break;
                        }
                }

    	str1 = "The Estimated States log_file is ok!\n\n";
    	GLCD_DrawString(START_COL, line++ * FONTS_H, str1);

    	UartEndSimulation();

        return 0;
    }

//---------------------------------------------------------------------------------------------------------//

void stateMachine(uint8_t command, uint8_t *data, uint32_t data_size){
    // Processamento de dados:
    if(command == REG01R){
        flag_dataProc = 1;
    }
    // Finalização do programa:
    else if(command == REG02R){
        flag_end = 1;
    }

    else{
    }
    
}

//---------------------------------------------------------------------------------------------------------//
// UART2 initialization:
void gg_UartStdOutInit(void){
    CM3DS_MPS2_gpio_SetAltFunc(CM3DS_MPS2_GPIO0, 17);

    // Interrupt RX/TX ON :
    // CM3DS_MPS2_uart_init(CM3DS_MPS2_UART2, (SystemCoreClock / 38400), 1, 1, 1, 1, 0, 0);
    // NVIC_ClearPendingIRQ(UART2_IRQn);
    // NVIC_EnableIRQ(UART2_IRQn);

    // Interrupt RX/TX OFF :
    CM3DS_MPS2_uart_init(CM3DS_MPS2_UART2, (SystemCoreClock / 38400), 1, 1, 0, 0, 0, 0);

    return;

    // CM3DS_MPS2_UART2->BAUDDIV = SystemCoreClock / 38400;  // 38400 at 25MHz
    // CM3DS_MPS2_UART2->CTRL    = ((1ul <<  0) | (1ul <<  1) );             /* TX/RX enable */
    
    // CM3DS_MPS2_GPIO0->ALTFUNCSET = (1ul <<  4) | (1ul << 0);  // Função alternativa dos pinos             
    // return;
}

//---------------------------------------------------------------------------------------------------------//
// Output a character:
void gg_UartPutc(unsigned char my_ch){
    CM3DS_MPS2_uart_SendChar(CM3DS_MPS2_UART2, my_ch);

    return;
    // while ((CM3DS_MPS2_UART2->STATE & 1)); // Wait if Transmit Holding register is full

    // if (my_ch == '\n'){
    //     CM3DS_MPS2_UART2->DATA  = '\r';
    //     while ((CM3DS_MPS2_UART2->STATE & 1)); // Wait if Transmit Holding register is full
    // }

    // CM3DS_MPS2_UART2->DATA = my_ch; // write to transmit holding register

    // return;
}

//---------------------------------------------------------------------------------------------------------//
// Get a character:
unsigned char gg_UartGetc(void){
  return CM3DS_MPS2_uart_ReceiveChar(CM3DS_MPS2_UART2);

  // unsigned char my_ch;

  // while ((CM3DS_MPS2_UART2->STATE & 2)==0); // Wait if Receive Holding register is empty
  
  // my_ch = CM3DS_MPS2_UART2->DATA;

  // //Convert CR to LF
  // if(my_ch == '\r')
  //     my_ch = '\n';

  // return (my_ch);
}

//---------------------------------------------------------------------------------------------------------//
// UART2 interrupt handler:
void UART2_Handler(void){
    // Clear TX IRQ:
    CM3DS_MPS2_uart_ClearRxIRQ(CM3DS_MPS2_UART2);

    flag_protocol = protocolUpdate(gg_UartGetc());

    // Ensure Interrupt is not pending:
    // NVIC_ClearPendingIRQ(UART2_IRQn);
    // Enable Interrupts:
    NVIC_EnableIRQ(UART2_IRQn);
}
