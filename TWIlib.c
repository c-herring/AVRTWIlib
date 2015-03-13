/*
 * TWIlib.c
 *
 * 3/13/2015 Chris Herring
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * The Software is provided "as is", without warranty of any kind.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "TWIlib.h"
#include "util/delay.h"
#include <string.h>

// Initialize the TWI hardware. This function must be called before any other TWI functions can be used however it should be called after 
// global interrupts have been enabled (a call to sei(); )
void TWIInit()
{
	TWIInfo.mode = Ready;
	TWIInfo.errorCode = 0xFF;
	TWIInfo.repStart = 0;
	// Set pre-scalers (no pre-scaling)
	TWSR = 0;
	// Set bit rate
	TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;
	// Enable TWI and interrupt
	TWCR = (1 << TWIE) | (1 << TWEN);
}

// Check the ready status of the TWI interface.
// Returns:
//		1	- TWI is ready to go.
//		0	- TWI is not ready.
uint8_t isTWIReady()
{
	if ( (TWIInfo.mode == Ready) | (TWIInfo.mode == RepeatedStartSent) )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


// TWITransmitData begins a TWI transmission. A hard coded buffer length of MAXBUFLEN is used for the transmit buffer.
// If you require larger transmissions then this may be changed in the TWIlib.h header. This function accepts a void
// pointer, which will then be cast to a uint8_t*. This allows any datatype to be sent one bute at a time.
// As with any application, the programmer should ensure that dataLen is not longer than the data, or else this function
// will attempt to index out of bounds.
//
// Inputs:
//		TXdata	- Void pointer to the first byte of data stream to be sent.
//		dataLen	- Length of the data in bytes. This must not exceed the hard coded TXMAXBUFLEN define.
//		repStart- Should the device relinquish control of the bus after the transmission or should it send a 
//					repeated start signal? 1 = send the repeated start; 0 = sent stop.
//
// Returns:
//		0		- Transmission successfully started.
//		1		- Attempted to send a data stream longer than MAXBUFLEN bytes.
//					Split the data stream, compress it or increase MAXBUFLEN.
//		2		- The TWI bus was not ready, no transmission has been started yet. Try again later.
//
uint8_t TWITransmitData(void *const TXdata, uint8_t dataLen, uint8_t repStart)
{
	if (dataLen <= TXMAXBUFLEN)
	{
		// Wait until ready
		//while (!isTWIReady()) {_delay_us(1);}
		if (!isTWIReady()) {return 2;}
		// Set repeated start mode
		TWIInfo.repStart = repStart;
		// Copy data into the transmit buffer
		uint8_t *data = (uint8_t *)TXdata;
		for (int i = 0; i < dataLen; i++)
		{
			TWITransmitBuffer[i] = data[i];
		}
		// Copy transmit info to global variables
		TXBuffLen = dataLen;
		TXBuffIndex = 0;
		
		// If a repeated start has been sent, then devices are already listening for an address
		// and another start does not need to be sent. 
		if (TWIInfo.mode == RepeatedStartSent)
		{
			TWIInfo.mode = Initializing;
			TWDR = TWITransmitBuffer[TXBuffIndex++]; // Load data to transmit buffer
			TWISendTransmit(); // Send the data
		}
		else // Otherwise, just send the normal start signal to begin transmission.
		{
			TWIInfo.mode = Initializing;
			TWISendStart();
		}
		
	}
	else
	{
		return 1; // return an error if data length is longer than buffer
	}
	return 0;
}

// TWIReadData reads begins a master read operation. A write operation is usually performed before the read in order to 
// set the slave up to transmit the desired data. This function takes a salve address, utilizes the transmit function to 
// address the slave with the read bit set, then lets the interrupts do the rest of the work. The receive buffer length 
// global variable is set to the number of bytes to read. The logic in the interrupt vector code will reply with ACK 
// until there is only one more byte to read then reply with NACK. So it should be ensured that the number of bytes to read is 
// correctly specified. If it is too long then the slave might stop sending and we enter the realm of undefined behavior. 
// If a transmit failed due to the TWI module not being ready then this function will return a 2 and it should be handled in the 
// same manner as the transmit function. It is assumed that the transmit function will never return 1, this can only happen if you have
// the maximum transmit buffer length set to zero. If that is the case you should probably give up now.
//
// Inputs:
//		TWIaddr		- The 7 bit address of the slave to read from. the 8th bit should be 0, but if it is not then it will be shifted
//						out anyway. So it does not really matter.
//		bytesToRead	- The number of bytes to read. Be careful this is not more than the slave knows how to send. Otherwise you may 
//						encounter some undefined behavior.
//		repStart	-	1 to send a repeated start signal after the transmission and maintain control of the bus.
//						0 to send a stop signal after the transmission and relinquish control of the bus.
//
uint8_t TWIReadData(uint8_t TWIaddr, uint8_t bytesToRead, uint8_t repStart)
{
	// Check if number of bytes to read can fit in the RXbuffer
	if (bytesToRead <= RXMAXBUFLEN)
	{
		// Reset buffer index and set RXBuffLen to the number of bytes to read
		RXBuffIndex = 0;
		RXBuffLen = bytesToRead;
		// Create the one value array for the address to be transmitted
		uint8_t TXdata[1];
		// Shift the address and AND a 1 into the read write bit (set to write mode)
		TXdata[0] = (TWIaddr << 1) | 0x01;
		// Use the TWITransmitData function to initialize the transfer and address the slave
		if (TWITransmitData(TXdata, 1, repStart) == 2)
			return 2;
	}
	else
	{
		return 1;
	}
	return 0;
}

ISR (TWI_vect)
{
	switch (TWI_STATUS)
	{
		// ----\/ ---- MASTER TRANSMITTER OR WRITING ADDRESS ----\/ ----  //
		case TWI_MT_SLAW_ACK: // SLA+W transmitted and ACK received
		// Set mode to Master Transmitter
		TWIInfo.mode = MasterTransmitter;
		case TWI_START_SENT: // Start condition has been transmitted
		case TWI_MT_DATA_ACK: // Data byte has been transmitted, ACK received
			if (TXBuffIndex < TXBuffLen) // If there is more data to send
			{
				TWDR = TWITransmitBuffer[TXBuffIndex++]; // Load data to transmit buffer
				TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
				TWISendTransmit(); // Send the data
			}
			// This transmission is complete however do not release bus yet
			else if (TWIInfo.repStart)
			{
				TWIInfo.errorCode = 0xFF;
				TWISendStart();
			}
			// All transmissions are complete, exit
			else
			{
				TWIInfo.mode = Ready;
				TWIInfo.errorCode = 0xFF;
				TWISendStop();
			}
			break;
		
		// ----\/ ---- MASTER RECEIVER ----\/ ----  //
		
		case TWI_MR_SLAR_ACK: // SLA+R has been transmitted, ACK has been received
			// Switch to Master Receiver mode
			TWIInfo.mode = MasterReceiver;
			// If there is more than one byte to be read, receive data byte and return an ACK
			if (RXBuffIndex < RXBuffLen-1)
			{
				TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
				TWISendACK();
			}
			// Otherwise when a data byte (the only data byte) is received, return NACK
			else
			{
				TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
				TWISendNACK();
			}
			break;
		
		case TWI_MR_DATA_ACK: // Data has been received, ACK has been transmitted.
		
			/// -- HANDLE DATA BYTE --- ///
			TWIReceiveBuffer[RXBuffIndex++] = TWDR;
			// If there is more than one byte to be read, receive data byte and return an ACK
			if (RXBuffIndex < RXBuffLen-1)
			{
				TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
				TWISendACK();
			}
			// Otherwise when a data byte (the only data byte) is received, return NACK
			else
			{
				TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
				TWISendNACK();
			}
			break;
		
		case TWI_MR_DATA_NACK: // Data byte has been received, NACK has been transmitted. End of transmission.
		
			/// -- HANDLE DATA BYTE --- ///
			TWIReceiveBuffer[RXBuffIndex++] = TWDR;	
			// This transmission is complete however do not release bus yet
			if (TWIInfo.repStart)
			{
				TWIInfo.errorCode = 0xFF;
				TWISendStart();
			}
			// All transmissions are complete, exit
			else
			{
				TWIInfo.mode = Ready;
				TWIInfo.errorCode = 0xFF;
				TWISendStop();
			}
			break;
		
		// ----\/ ---- MT and MR common ----\/ ---- //
		
		case TWI_MR_SLAR_NACK: // SLA+R transmitted, NACK received
		case TWI_MT_SLAW_NACK: // SLA+W transmitted, NACK received
		case TWI_MT_DATA_NACK: // Data byte has been transmitted, NACK received
		case TWI_LOST_ARBIT: // Arbitration has been lost
			// Return error and send stop and set mode to ready
			if (TWIInfo.repStart)
			{				
				TWIInfo.errorCode = TWI_STATUS;
				TWISendStart();
			}
			// All transmissions are complete, exit
			else
			{
				TWIInfo.mode = Ready;
				TWIInfo.errorCode = TWI_STATUS;
				TWISendStop();
			}
			break;
		case TWI_REP_START_SENT: // Repeated start has been transmitted
			// Set the mode but DO NOT clear TWINT as the next data is not yet ready
			TWIInfo.mode = RepeatedStartSent;
			break;
		
		// ----\/ ---- SLAVE RECEIVER ----\/ ----  //
		
		// TODO  IMPLEMENT SLAVE RECEIVER FUNCTIONALITY
		
		// ----\/ ---- SLAVE TRANSMITTER ----\/ ----  //
		
		// TODO  IMPLEMENT SLAVE TRANSMITTER FUNCTIONALITY
		
		// ----\/ ---- MISCELLANEOUS STATES ----\/ ----  //
		case TWI_NO_RELEVANT_INFO: // It is not really possible to get into this ISR on this condition
								   // Rather, it is there to be manually set between operations
			break;
		case TWI_ILLEGAL_START_STOP: // Illegal START/STOP, abort and return error
			TWIInfo.errorCode = TWI_ILLEGAL_START_STOP;
			TWIInfo.mode = Ready;
			TWISendStop();
			break;
	}
	
}
