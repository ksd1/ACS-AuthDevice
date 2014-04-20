/*
 * CAN_Lib.h
 *
 * Created: 2014-03-15 21:55:29
 *  Author: Krzysiu
 */ 


#ifndef CAN_LIB_H_
#define CAN_LIB_H_

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>


typedef struct {
	uint8_t OutputBuffer;
	uint8_t Priority;
	uint16_t StdIdentifier;
	uint8_t MessageLength;
	uint8_t Messages[8];
	uint8_t RemoteTransmission;
	}StandartMessage;

//PINS

#define SPI_PORT PORTB
#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5


//Commands
#define RESET_CMD 0b11000000
#define RTS_TXB0_CMD 0b10000001
#define RTS_TXB1_CMD 0b10000010
#define RTS_TXB2_CMD 0b10000100
#define READ_STATUS_CMD 0b10100000
#define BIT_MODIFY_CMD 0b00000101
#define READ_CMD 0b00000011
#define WRITE_CMD 0b00000010
#define LOAD_TXBUFFER0 0b01000000
#define LOAD_TXBUFFER1 0b10000010
#define LOAD_TXBUFFER2 0b10000100
#define READ_RXBUF0 0b10010000
#define READ_RXBUF1 0b10010100

//Register
#define CANCTRL_ADR 0x0F
#define CANINTF_ADR 0x2C
#define CANSTAT_ADR 0x0E
#define TXB0SIDH_ADR 0x31
#define TXB0SIDL_ADR 0x32
#define TXBOD_ADR 0x36
#define TXB1D_ADR 0x46
#define TXB2D_ADR 0x56

#define CNF1_ADR 0x2A
#define CNF2_ADR 0x29
#define CNF3_ADR 0x28
//Bits in register
#define RX0IF 0x01
#define RX1IF 0x02




//Mode
#define LOOPBACK_MODE 0x02
#define NORMAL_MODE 0x00

//Flags
#define ENABLE_REMOTE_TRANSMISION 0x40
#define RXBUF0 0
#define RXBUF1 1


void SPI_SetSS();
void SPI_ResetSS();
void SPI_Init();
uint8_t SPI_SendReceiveByte(uint8_t data);
void SpiSendByte(uint8_t data);

void CAN_Reset();
void CAN_Init();
void CAN_InitRXBuffer();
void CAN_SetNormalMode();
void CAN_SetLoopbackMode();
uint8_t CAN_GetBuffer(uint8_t bufferAdr);
void CAN_SetBuffer(uint8_t bufferAdr, uint8_t value);
void CAN_SendCmd(uint8_t cmd);
void CAN_TransmitMessage(StandartMessage* stdMessage);
void CAN_ReceiveMessage(uint8_t InputBuffer, StandartMessage* stdMessage);
int8_t CAN_CheckInbox();





#endif /* CAN_LIB_H_ */