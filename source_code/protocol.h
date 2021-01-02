/*******************************************************************************
	Communication Protocol Library.

	Description:

	Generic communication protocol to stewart platform.

		Frame:
			 _________________________________________________
			|												  |
			| START + ADDRESS + COMMAND + DATA + CRC16 + STOP |
			|_________________________________________________|

			START	->	1 Byte(s);
			ADDRESS	->	1 Byte(s);
			COMMAND	->	1 Byte(s);
			DATA	->	N Byte(s);
			CRC16	->	2 Byte(s);
			STOP	->	1 Byte(s);

	@ Author:	Vinícius Melo;

	@ Date:		August-2017;
	
	@ Version:	1.0;
	
*******************************************************************************/

#ifndef PROTOCOL_H
#define PROTOCOL_H

//----------------------------------------------------------------------------//

#include "stdlib.h"
#include "stdint.h"
#include "config.h"

//----------------------------------------------------------------------------//

#define PROTOCOL_BUFFER_SIZE	64

//----------------------------------------------------------------------------//

#define START	(uint8_t)0xFE
#define STOP	(uint8_t)0x55
#define ESCAPE	(uint8_t)0x13

//----------------------------------------------------------------------------//

// // Union to float:
// typedef union{
// 	float f;		// 32 bits;
// 	uint8_t c[4];	// 8 bits (c[3] = MSB ... c[0] = LSB);
// }float_num_t;

// // Union to long:
// typedef union{
// 	uint32_t l;		// 32 bits;
// 	uint8_t c[4];	// 8 bits (c[3] = MSB ... c[0] = LSB);
// }long_num_t;

//----------------------------------------------------------------------------//

// Initialize:
void protocolInit(uint8_t slaveAddress, uint8_t allSlaveAddress);

// Reception:
uint8_t protocolUpdate(uint8_t data);
uint32_t protocolGetPackSize(void);
void protocolGetPack(uint8_t *data, uint32_t data_size);
uint32_t protocolGetDataSize(void);
void protocolGetData(uint8_t *data, uint32_t data_size);
uint8_t protocolGetCommand(void);
uint8_t protocolCheckCRC(void);

// Transmission:
uint32_t protocolGetFrame(uint8_t *buffer, uint8_t address, uint8_t command, uint8_t *data, uint32_t data_size);

// General:
void protocolReset(void);
int16_t crc16(uint8_t *data, uint32_t data_size);

//----------------------------------------------------------------------------//

#endif // PROTOCOL_H

//----------------------------------------------------------------------------//
