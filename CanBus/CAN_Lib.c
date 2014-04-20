/*
 * CAN_Lib.c
 *
 * Created: 2014-03-15 21:54:25
 *  Author: Krzysiu
 */ 

#include "CAN_Lib.h"

inline void SPI_SetSS()
{
	//PORTB|=_BV(PB2);
	SPI_PORT|=_BV(SS);
}

inline void SPI_ResetSS()
{
	//PORTB&=~(_BV(PB2));
	SPI_PORT&=~(_BV(SS));
}

void SPI_Init()
{
	/*
		Konfiguracja kontrolera CAN MCP2515:
		
		- pierwszy MSB
		- polaryzacja zegara: 0-stan bezczynnosci, przejscie na 1- transmisja (str. 63 dokumentacja)
		- stan lini danych próbkowany na zboczu spadaj¹cym
	*/
	
	DDRB |= ( _BV(SS) | _BV(MOSI) | _BV(SCK)); //Inicjacja MOSI, SCK, SS
	SPI_SetSS();
	SPCR = _BV(SPE) | _BV(MSTR);
	SPSR;
	SPDR;
}

uint8_t SPI_SendReceiveByte(uint8_t data)
{
	
	SPDR = data;
	while(!(SPSR & _BV(SPIF)));
	return SPDR;
	
}

void CAN_Reset()
{
	SPI_ResetSS();
	SPI_SendReceiveByte(RESET_CMD);
	SPI_SetSS();
}

void CAN_Init()
{
	/*
		Oscilator Frequency: 16 MHz
		CAN Bus Baud Rate 250 kbps
		
		Baud Rate prescaler: 1
		Tq(Time Quanta): 250 ns
		Number of Time Quanta: 16
		% Error: 0,0%
		
		Propagation Delay: 1
		Phase Segment 1: 8
		Phase Segment 2: 6
		Syncronization Jump Width(SJW): 1
	*/
	
	CAN_SetBuffer(CNF1_ADR, 0x01);
	_delay_ms(10);
	//CAN_SetBuffer(CNF2_ADR, 0xB8);
	//_delay_ms(10);
	CAN_SetBuffer(CNF2_ADR, 0xAC);
	_delay_ms(10);
	//CAN_SetBuffer(CNF3_ADR, 0x05);
	//_delay_ms(10);
	CAN_SetBuffer(CNF3_ADR, 0x03);
	_delay_ms(10);
}

void CAN_SetLoopbackMode()
{
	SPI_ResetSS();
	
	SPI_SendReceiveByte(BIT_MODIFY_CMD);
	SPI_SendReceiveByte(CANCTRL_ADR);
	SPI_SendReceiveByte(0xE0); //Sending Mask
	SPI_SendReceiveByte(0x40);
	
	SPI_SetSS();
	
}

void CAN_SetNormalMode()
{
	SPI_ResetSS();
	
	SPI_SendReceiveByte(BIT_MODIFY_CMD);
	SPI_SendReceiveByte(CANCTRL_ADR);
	SPI_SendReceiveByte(0xE0); //Sending Mask
	SPI_SendReceiveByte(0x00);
	
	SPI_SetSS();
	
}

uint8_t CAN_GetBuffer(uint8_t bufferAdr)
{
	uint8_t rec;
	
	SPI_ResetSS();
	SPI_SendReceiveByte(READ_CMD);
	SPI_SendReceiveByte(bufferAdr);
	rec = SPI_SendReceiveByte(0x00);
	SPI_SetSS();
	return rec;
}

void CAN_SetBuffer(uint8_t bufferAdr, uint8_t value)
{
	SPI_ResetSS();
	SPI_SendReceiveByte(WRITE_CMD);
	SPI_SendReceiveByte(bufferAdr);
	SPI_SendReceiveByte(value);
	SPI_SetSS();
}

void CAN_SendCmd(uint8_t cmd)
{
	SPI_ResetSS();
	SPI_SendReceiveByte(cmd);
	SPI_SetSS();
}

void CAN_TransmitMessage(StandartMessage* stdMessage){
	
	SPI_ResetSS();
	
	switch(stdMessage->OutputBuffer)
	{
		case 0: SPI_SendReceiveByte(LOAD_TXBUFFER0);
				break;
		case 1: SPI_SendReceiveByte(LOAD_TXBUFFER1);
				break;
		case 2: SPI_SendReceiveByte(LOAD_TXBUFFER2);
				break;
	}
	
	
	
	//Loading Standard Identifier
	SPI_SendReceiveByte(stdMessage->StdIdentifier >> 3);	//TXB0SIDH
	SPI_SendReceiveByte(stdMessage->StdIdentifier << 5);	//TXB0SIDL
	SPI_SendReceiveByte(0x00); //TXB0EID8
	SPI_SendReceiveByte(0x00); //TXB0EID0
	
	//Loading DataLength
	SPI_SendReceiveByte(stdMessage->RemoteTransmission? ENABLE_REMOTE_TRANSMISION: stdMessage->MessageLength);	//TXB0DLC
	
	if(!stdMessage->RemoteTransmission)
	{
		//Loading Data
		for(int i=0; i<stdMessage->MessageLength; ++i)
		{
			SPI_SendReceiveByte(stdMessage->Messages[i]);
		}
	}	
	SPI_SetSS();
	
	//Ready so send command
	switch(stdMessage->OutputBuffer)
	{
		case 0: CAN_SendCmd(RTS_TXB0_CMD);
		break;
		case 1: CAN_SendCmd(RTS_TXB1_CMD);
		break;
		case 2: CAN_SendCmd(RTS_TXB2_CMD);
		break;
	}
	
	
}

void CAN_ReceiveMessage(uint8_t InputBuffer, StandartMessage* stdMessage)
{
	uint8_t RecBufSTIDL = 0;
	
	
	SPI_ResetSS();
	
	switch(InputBuffer)
	{
		case 0: SPI_SendReceiveByte(READ_RXBUF0);
		break;
		case 1: SPI_SendReceiveByte(READ_RXBUF1);
		break;
	}
	
	stdMessage->StdIdentifier = SPI_SendReceiveByte(0x00);
	stdMessage->StdIdentifier <<= 3;
	
	RecBufSTIDL = SPI_SendReceiveByte(0x00);
	
	if(RecBufSTIDL & 0x10)
	{
		stdMessage->RemoteTransmission = 1;
		stdMessage->StdIdentifier |= (RecBufSTIDL>>5 );
		SPI_SetSS();
		return;
	}		
	else
		stdMessage->RemoteTransmission = 0;
	
	//stdMessage->StdIdentifier |= (SPI_SendReceiveByte(0x00)>>5 );
	stdMessage->StdIdentifier |= (RecBufSTIDL>>5 );
	SPI_SendReceiveByte(0x00);
	SPI_SendReceiveByte(0x00);
	stdMessage->MessageLength = (SPI_SendReceiveByte(0x00) & 0x0F);
	
	for(int i=0; i<stdMessage->MessageLength; i++)
	{
		stdMessage->Messages[i] = SPI_SendReceiveByte(0x00);
	}
	
	SPI_SetSS();
}


void CAN_InitRXBuffer()
{
	SPI_ResetSS();
	SPI_SendReceiveByte(BIT_MODIFY_CMD);
	SPI_SendReceiveByte(0x60);
	SPI_SendReceiveByte(0x60);
	SPI_SendReceiveByte(0x60);
	SPI_SetSS();
	
	_delay_us(50);
	
	SPI_ResetSS();
	SPI_SendReceiveByte(BIT_MODIFY_CMD);
	SPI_SendReceiveByte(0x70);
	SPI_SendReceiveByte(0x60);
	SPI_SendReceiveByte(0x60);
	SPI_SetSS();
	
	
}

int8_t CAN_CheckInbox()
{
	char InboxState = CAN_GetBuffer(CANINTF_ADR);
	
	if(InboxState&RX0IF)
	return RXBUF0;
	if(InboxState&RX1IF)
	return RXBUF1;
	
	return -1;
}