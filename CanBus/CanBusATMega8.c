/*
 * CanBus.c
 *
 * Created: 2014-03-15 19:57:12
 *  Author: Krzysiu
 */ 


#include <avr/io.h>
#include <util/delay.h>

#include "mifare.h"
#include "ACS_Protocol.h"





void blink()
{
	PORTD = 0;
	_delay_ms(400);
}


int main(void)
{
	
	//StandartMessage TxMessage;
	//StandartMessage RxMessage;
	
	//TxMessage.OutputBuffer = 0;
	//TxMessage.Priority = 0;
	//TxMessage.StdIdentifier = DEVICE_ID;
	//TxMessage.MessageLength = 2;
	//TxMessage.RemoteTransmission = 0;
	//TxMessage.Messages[0] = 0xF0;
	//TxMessage.Messages[1] = 0x0F;
	
	
	
		
	DDRD = 0xff;
	PORTD = 0xff;
	
	uint8_t OutTable[8];
	SPI_Init();
	//MFRC522_Init();
	
	//PORTD = ~ ( Read_MFRC522(0x37) );
	//_delay_ms(2000);
	

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

	
	
	uint8_t i,tmp;
	uint8_t status;
	uint8_t str[MAX_LEN];
	uint8_t RC_size;
	uint8_t blockAddr;
	char mynum[8];
	
	ACS_ConnectToSystem();
	
    while(1)
    {
		
		/*
		blink();
		PORTD = 0xFF;
		_delay_ms(500);
		*/
		
		/*
		if(counter > 5)
		{
			CAN_TransmitMessage(&TxMessage);
			counter = 0;
			_delay_ms(2000);
		}
		else
		{
			counter++;
			_delay_ms(2000);
		}
		*/
		
		/*
        if(CAN_GetBuffer(CANINTF_ADR)&(RX0IF|RX1IF))
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
			blink();
			PORTD = ~(CAN_GetBuffer(0x1D));
			_delay_ms(2000);
		}		
		*/	
    }
}