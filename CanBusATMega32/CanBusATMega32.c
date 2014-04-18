/*
 * CanBus.c
 *
 * Created: 2014-03-15 19:57:12
 *  Author: Krzysztof Antosz
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include "CAN_Lib.h"

#define DEVICE_ID 0x02

//#define SET_RED PORTD&=~(_BV(PD5))
//#define SET_YELLOW PORTD&=~(_BV(PD6))
//#define SET_GREEN PORTD&=~(_BV(PD7))
//
void blink()
{
	PORTD = 0;
	_delay_ms(400);
}






int main(void)
{
	
	StandartMessage TxMessage;
	StandartMessage RxMessage;
	
	TxMessage.OutputBuffer = 0;
	TxMessage.Priority = 0;
	TxMessage.StdIdentifier = DEVICE_ID;
	TxMessage.MessageLength = 4;
	TxMessage.RemoteTransmission = 0;
	TxMessage.Messages[0] = 0xA0;
	TxMessage.Messages[1] = 0x0A;
	TxMessage.Messages[2] = 0x50;
	TxMessage.Messages[3] = 0x05;
	
	
		
	DDRD = 0xff;
	PORTD = 0xff;
	
	uint8_t OutTable[8];
	SPI_Init();
	CAN_Reset();
	CAN_Init();
	 
	 PORTD = ~(CAN_GetBuffer(CANSTAT_ADR));
	  _delay_ms(1000);
	 //CAN_SetLoopbackMode();
	 CAN_SetNormalMode();
	  _delay_ms(100);
	  PORTD = ~(CAN_GetBuffer(CANSTAT_ADR));
	  _delay_ms(1000);
	
	 //CAN_TransmitMessage(&TxMessage);
	 
	int counter = 0;
	
	
    while(1)
    {
		
		//if(counter > 2)
		//{
			//CAN_TransmitMessage(&TxMessage);
			//counter = 0;
		//}
		//else
		//{
			//counter++;
		//}
		
		
        if(CAN_GetBuffer(CANINTF_ADR)&RX0IF)
        {
	        CAN_ReceiveMessage(0,&RxMessage);
	        
	        blink();
	        PORTD = ~RxMessage.StdIdentifier;
	        _delay_ms(2000);
	        blink();
	        PORTD = ~RxMessage.MessageLength;
	        _delay_ms(2000);
	        
	        if(RxMessage.RemoteTransmission)
				PORTD = ~RxMessage.RemoteTransmission;
	        else
	        {
		        for(int i=0; i<RxMessage.MessageLength; i++)
		        {
			        blink();
			        PORTD = ~RxMessage.Messages[i];
			        _delay_ms(2000);
		        }
		        
	        }
        }
		else
		{
			blink();
			PORTD = ~(0b11000011);
			_delay_ms(2000);
		}			
    }
}