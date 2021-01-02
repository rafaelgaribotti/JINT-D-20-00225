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

#include "protocol.h"

//----------------------------------------------------------------------------//

uint8_t g_buffer[PROTOCOL_BUFFER_SIZE];	// Used to store datas;

typedef struct{
	
	uint8_t slaveAddress;		// Slave address;
	uint8_t allSlaveAddress;	// Generic slave address;
	uint8_t flag_data;			// Data status (true = last data is dummy (ESCAPE));
	uint8_t flag_pack;			// Protocol status (true = acquire complete);
	uint8_t last_data;			// Last data;
	uint32_t data_count;		// Pack Size;
	uint32_t frame_ptr;			// Frame pointer;

}protocol_t;

protocol_t g_protocol;

//----------------------------------------------------------------------------//

// Internal Functions:
void protocolStateMachine(uint8_t data);
void dataWrite(uint8_t data);
void dataRead(uint8_t *data, uint32_t data_size);
void frameWrite(uint8_t data, uint8_t *buffer);

/*******************************************************************************

	@ Name		:	protocolInit;

	@ Brief		:	Initialize protocol;

	@ Parameter	:	Slave address and generic slave address;

	@ Return	:	None;

*******************************************************************************/

void protocolInit(uint8_t slaveAddress, uint8_t allSlaveAddress)
{
	g_protocol.slaveAddress = slaveAddress;
	g_protocol.allSlaveAddress = allSlaveAddress;
	g_protocol.flag_data = 0;
	g_protocol.flag_pack = 0;
	g_protocol.last_data = 0;
	g_protocol.data_count = 0;
	g_protocol.frame_ptr = 0;
}

/*******************************************************************************

	@ Name		:	protocolUpdate;

	@ Brief		:	Acquire new data while pack is incomplete;

	@ Parameter	:	New data;

	@ Return	:	Flag that indicates received pack;

*******************************************************************************/

uint8_t protocolUpdate(uint8_t data)
{
	if(g_protocol.flag_pack == 0 && g_protocol.data_count < PROTOCOL_BUFFER_SIZE) protocolStateMachine(data);
	g_protocol.last_data = data;
	return g_protocol.flag_pack;
}

/*******************************************************************************

	@ Name		:	protocolGetPackSize;

	@ Brief		:	Return the pack size if pack was received;

	@ Parameter	:	None;

	@ Return	:	Pack size;

*******************************************************************************/

uint32_t protocolGetPackSize(void)
{
	if(g_protocol.flag_pack) return g_protocol.data_count;
	else return 0;
}

/*******************************************************************************

	@ Name		:	protocolGetPack;

	@ Brief		:	Store pack (COMMAND + DATA + CRC16) if pack was received;

	@ Parameter	:	Data pointer and data size;

	@ Return	:	None;

*******************************************************************************/

void protocolGetPack(uint8_t *data, uint32_t data_size)
{
	if(data_size <= g_protocol.data_count && g_protocol.flag_pack)
	{
		for(uint32_t i = 0; i < data_size; i++) data[i] = g_buffer[i];
	}
}

/*******************************************************************************

	@ Name		:	protocolGetDataSize;

	@ Brief		:	Return the data size if pack was received;

	@ Parameter	:	None;

	@ Return	:	Data size;

*******************************************************************************/

uint32_t protocolGetDataSize(void)
{
	if(g_protocol.flag_pack) return g_protocol.data_count - 3;    // Just DATA bytes;
	else return 0;
}

/*******************************************************************************

	@ Name		:	protocolGetData;

	@ Brief		:	Store data (DATA) if pack was received;

	@ Parameter	:	Data pointer and data size;

	@ Return	:	None;

*******************************************************************************/

void protocolGetData(uint8_t *data, uint32_t data_size)
{
	if(data_size <= (g_protocol.data_count - 3) && g_protocol.flag_pack)
	{
		for(uint32_t i = 0; i < data_size; i++) data[i] = g_buffer[i + 1];
	}
}

/*******************************************************************************

	@ Name		:	protocolGetCommand;

	@ Brief		:	Return the command if pack was received;

	@ Parameter	:	None;

	@ Return	:	Command;

*******************************************************************************/

uint8_t protocolGetCommand(void)
{
	if(g_protocol.flag_pack) return g_buffer[0];
	else return 0;
}

/*******************************************************************************

	@ Name		:	protocolCheckCRC;

	@ Brief		:	Checks the CRC if pack was received;

	@ Parameter	:	None;

	@ Return	:	Flag that indicates if CRC16 is true;

*******************************************************************************/

uint8_t protocolCheckCRC(void)
{
	int16_t crc;

	if(g_protocol.flag_pack)
	{
		crc = crc16(g_buffer, g_protocol.data_count - 2);
		if((uint8_t)crc == (g_buffer[g_protocol.data_count - 1]) && (uint8_t)(crc >> 8) == (g_buffer[g_protocol.data_count - 2])) return 1;
		else return 0;
	}

	else return 0;
}

/*******************************************************************************

	@ Name		:	protocolGetFrame;

	@ Brief		:	Generates the pack with ESCAPE when necessary and with CRC16;

	@ Parameter	:	Buffer pointer, address, command, data pointer and data size;

	@ Return	:	Pack size;

*******************************************************************************/

uint32_t protocolGetFrame(uint8_t *buffer, uint8_t address, uint8_t command, uint8_t *data, uint32_t data_size)
{
	uint8_t crcBuffer[PROTOCOL_BUFFER_SIZE];
	int16_t crc;
	uint32_t crc_ptr = 0;

	g_protocol.frame_ptr = 0;
	crc_ptr = 0;

	buffer[g_protocol.frame_ptr++] = START;

	buffer[g_protocol.frame_ptr++] = address;

	frameWrite(command, buffer);
	crcBuffer[crc_ptr++] = command;

	for(uint32_t i = 0; i < data_size; i++)
	{
		frameWrite(data[i], buffer);
		crcBuffer[crc_ptr++] = data[i];
	}

	crc = crc16(crcBuffer, crc_ptr);
	frameWrite((uint8_t)(crc >> 8), buffer);
	frameWrite((uint8_t)(crc), buffer);

	buffer[g_protocol.frame_ptr++] = STOP;

	return g_protocol.frame_ptr;

}

/*******************************************************************************

	@ Name		:	reset;

	@ Brief		:	Reset protocol is necessary for new acquired;

	@ Parameter	:	None;

	@ Return	:	None;

*******************************************************************************/

void protocolReset(void)
{
	g_protocol.flag_data = 0;
	g_protocol.flag_pack = 0;
	g_protocol.last_data = 0;
	g_protocol.data_count = 0;
}

/*******************************************************************************

	@ Name		:	crc16;

	@ Brief		:	Generates the CRC-CCITT (0xFFFF);

	@ Parameter	:	Data pointer and data size;

	@ Return	:	crc16;

*******************************************************************************/

int16_t crc16(uint8_t *data, uint32_t data_size)
{
	int16_t crc = 0xFFFF;

	for(uint32_t i = 0; i < data_size; i++)
	{
		crc ^= ((*data++) << 8);
		for(uint32_t j = 0; j < 8; j++)
		{
			if(crc < 0) crc = (crc << 1) ^ 0x1021;
			else crc = crc << 1;
		}
	}

	return crc;
}

/*******************************************************************************

	@ Name		:	protocolStateMachine;

	@ Brief		:	State machine to data acquisition (data tratament);

	@ Parameter	:	New data;

	@ Return	:	None;

*******************************************************************************/

void protocolStateMachine(uint8_t data)
{
	static uint8_t state = 0;

	if(state == 0)			// Start acquisition;
	{
		if(data == START && g_protocol.last_data != ESCAPE) state = 1;
	}

	else if(state == 1)		// Address acquisition;
	{
		if(data == g_protocol.slaveAddress || data == g_protocol.allSlaveAddress) state = 2;

		else state = 0;
	}

	else					// Stop verification and store;
	{
		if(data == START)
		{
			if(g_protocol.flag_data)
			{
				dataWrite(data);
			}
		}

		else if(data == STOP)
		{
			if(g_protocol.flag_data)
			{
				dataWrite(data);
			}

			else
			{
				g_protocol.flag_pack = 1;
				state = 0;
			}
		}

		else if(data == ESCAPE)
		{
			if(g_protocol.flag_data)
			{
				dataWrite(data);
			}

			else
			{
				g_protocol.flag_data = 1;		// Last data is false;
			}
		}

		else
		{
			dataWrite(data);
		}
	}
}

/*******************************************************************************

	@ Name		:	dataWrite;

	@ Brief		:	Store data in blogal buffer;

	@ Parameter	:	Data;

	@ Return	:	None;

*******************************************************************************/

void dataWrite(uint8_t data)
{
	g_buffer[g_protocol.data_count] = data;	// Store;
	g_protocol.flag_data = 0;				// Last data is true;
	g_protocol.data_count++;				// Count increment;
}

/*******************************************************************************

	@ Name		:	dataRead;

	@ Brief		:	Read data from global buffer;

	@ Parameter	:	Data pointer and data size;

	@ Return	:	None;

*******************************************************************************/

void dataRead(uint8_t *data, uint32_t data_size)
{
	for(uint32_t i = 0; i < data_size; i++) data[i] = g_buffer[i];
}

/*******************************************************************************

	@ Name		:	frameWrite;

	@ Brief		:	Store data in buffer with ESCAPE when necessary;

	@ Parameter	:	Data and buffer pointer;

	@ Return	:	None;

*******************************************************************************/

void frameWrite(uint8_t data, uint8_t *buffer)
{
	if(data == START || data == STOP || data == ESCAPE)
	{
		// buffer[g_protocol.frame_ptr++] = ESCAPE;
		buffer[g_protocol.frame_ptr++] = data;
	}
	else buffer[g_protocol.frame_ptr++] = data;

}

//----------------------------------------------------------------------------//
