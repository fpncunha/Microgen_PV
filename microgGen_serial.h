/* 
 * File:   microgGen_serial.h
 * Author: VMORAIS
 *
 * Created on 11 de Maio de 2017, 10:37
 */

#ifndef MICROGGEN_SERIAL_H
#define	MICROGGEN_SERIAL_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#include <stdio.h>
#include <stdlib.h>
#include <libpic30.h>
#include <p30f4011.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "microgen.h"

#define PROTOCOL_ENA 1
#define PROTOCOL_DIS 0
#define SERIAL_PROTOCOL PROTOCOL_ENA


extern char receivedMessage[100];
//boolean status_command;
extern int STATE_cardinal;
extern int toSendCRC, nbytes, cnt;
extern pv2rpi pv2rpi_;

extern int commStatus;


void serialwrite(char *message, int count);
void serialwritechar(char byte);
void manageMessage(void);


int str2float(char msg[4]);

void float2str(int num);
void float2strGain10(int num);
void float2strGain100(int num);
void clearMessage();

void sendACK();

void sendTimeout();

void sendInvalidComamnd();


/*
/*
void setMessage(char msg[100]){ 
	strcpy(receivedMessage,msg); 
}

void enableCommand(void){
  status_command = COMMAND_ENA;
}
void microGen_serial::disableCommand(void){
  status_command = COMMAND_DIS;
}

*/

#endif	/* MICROGGEN_SERIAL_H */

