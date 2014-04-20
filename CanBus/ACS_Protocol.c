/*
 * ACS_Protocol.c
 *
 * Created: 2014-04-18 20:35:19
 *  Author: Krzysiu
 */ 

#include "ACS_Protocol.h"
#include "CAN_Lib.h"

uint8_t ACS_ConnectToSystem()
{
	StandartMessage TxMessage;
	StandartMessage RxMessage;
	uint16_t TimeOut = 0;
	uint8_t InBuffer = 0;
	
	TxMessage.StdIdentifier = MASTER_ID;
	TxMessage.MessageLength = 3;
	TxMessage.RemoteTransmission = 0;
	TxMessage.Messages[0] = CONNECT_DEVICE;
	TxMessage.Messages[1] = DEVICE_ID;
	TxMessage.Messages[2] = AUTHENTICATOR;
	TxMessage.OutputBuffer = 0;
	TxMessage.Priority = 0;
	
	CAN_TransmitMessage(&TxMessage);
	PORTD = ~(0x03);
	while((CAN_CheckInbox() == -1) && (TimeOut != TIMEOUT))
	{
		TimeOut++;
		_delay_ms(1);
	}
	
	PORTD = ~(0x04);
	if(TimeOut == TIMEOUT)
		ACS_NoConnectionACK_ErrorHandler();
	
	PORTD = ~(0x05);
	InBuffer = CAN_CheckInbox();
	
	CAN_ReceiveMessage(InBuffer, &RxMessage);
	
	if( RxMessage.StdIdentifier == DEVICE_ID &&
		RxMessage.Messages[0] == CONNECTION_ACK )
	{
		PORTD = ~(0b10000001);
		PORTD = ~(0xF0);
		return 0;
	}
	else
	{
		ACS_NoConnectionACK_ErrorHandler();
	}
			
	
	
	return -1;
}


uint8_t ACS_NoConnectionACK_ErrorHandler()
{
	PORTD = ~(0b00011000);
	return -1;
}