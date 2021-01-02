#ifndef __CONFIG_H__
#define __CONFIG_H__
/*********************************************************************
 * Configurações padrões para o FPGA.
 *
 * @author Rodrigo Alves Medeiros
 *.
 ********************************************************************/
/******************************DEFINE********************************/
    #define ADDR1   0x0B    // Endereço de identificação;
    #define REG01R  0x86    // Comando de processamento de dados;
    #define REG02R  0x87    // Comando de finalização;

    #define BUFFER_SIZE 64  // Tamanho máximo do buffer;

    #define px      0.701000000000000
    #define py      0.499500000000000
    #define pz      0.569750000000000
    #define pi      3.141592653589793
    #define Ts      1.2 //0.009615384615384999
    #define gValue  9.810000000000001

    #define artigo  0

/********************************************************************/
// Union to float:
typedef union{
    float f;        // 32 bits;
    uint8_t c[4];   // 8 bits (c[3] = MSB ... c[0] = LSB);
}float_num_t;

// Union to long:
typedef union{
    uint32_t l;     // 32 bits;
    uint8_t c[4];   // 8 bits (c[3] = MSB ... c[0] = LSB);
}long_num_t;

// Union to double:
typedef union{
    double d;       // 64 bits;
    uint8_t c[8];   // 8 bits (c[7] = MSB ... c[0] = LSB);
}double_num_t;

/********************************************************************/

#endif