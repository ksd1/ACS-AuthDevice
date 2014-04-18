/*
 * ACS_Protocol.h
 *
 * Created: 2014-04-18 19:58:35
 *  Author: Krzysztof Antosz
 */ 


#ifndef ACS_PROTOCOL_H_
#define ACS_PROTOCOL_H_

//ACS Request

#define CONNECT_DEVICE 0xF1
#define CONNECTION_ACK 0xF3

//Types of Devices
//Requester
#define SWITCH 0xB1
#define AUTHENTICATOR 0xB2

//Executor
#define LIGHT 0xD1
#define LOCK 0xD2


#endif /* ACS_PROTOCOL_H_ */