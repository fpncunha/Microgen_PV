#include <libpic30.h>
#include <p30f4011.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "microgGen_serial.h"
#include "microgen.h"

/*******************************************/
/************* protocol ********************/
char receivedMessage[100];
//boolean status_command;
int STATE_cardinal = 0;
int toSendCRC = 0, nbytes = 0, cnt = 0;
pv2rpi pv2rpi_;

int commStatus = PROTOCOL_ENA;



void serialwrite(char *message, int count) {
     
    int n = 0, i = 0;

   
    //sizeof (message);
    for (n = 0; n < (count - 1); n++) {
        while (U2STAbits.UTXBF) {
            // do nothing
        }

        for (i = 0; i < 1000; i++)
            asm volatile("nop");

        U2TXREG = message[n];
    }

}

void serialwritechar(char byte) {

    int i=0;
    while (U2STAbits.UTXBF) {
        // do nothing
    }
    for (i = 0; i < 1000; i++)
        asm volatile("nop");
    U2TXREG = byte;

}

void manageMessage(void) {
    int i = 0;
    char msg[4];
    msg[0] = '\0';
    char com[5];
    //com[0] = '\0';

    if (strcmp(receivedMessage, "SYNC") == 0) {


        clearMessage();
        //  grid2rpi_.APOWER=100;
        //     this->sync_ = SYNC_ENA;
        //break;
        //digitalWrite(pinLED, state);
    }

    else if (strcmp(receivedMessage, "DCBUS") == 0) {

        

        float2str(pv2rpi_.DCBUS);
        clearMessage();
    }
    else if (strcmp(receivedMessage, "VPV") == 0) {
      
        float2str(pv2rpi_.VPV);
        clearMessage();
    }
    else if (strcmp(receivedMessage, "IPV") == 0) {
        
        float2str(pv2rpi_.IPV);
        clearMessage();
    }
    else if (strcmp(receivedMessage, "PWR") == 0) {
        
        float2str(pv2rpi_.PWR);
        clearMessage();
    }
    else if (strcmp(receivedMessage, "WHOIS") == 0) {
        serialwrite("#SOLAR3#06#\n", sizeof ("#SOLAR1#06#\n")); ///  !!! aten��o: solar 10 numchar = 07; solar7 numchar 06
        clearMessage();
    }
    else if (strcmp(receivedMessage, "BROADCAST") == 0) {
        serialwrite("#DIS#03#\n", sizeof ("#DIS#03#\n"));
        commStatus = PROTOCOL_DIS;
        clearMessage();
    } 
    else if (strcmp(receivedMessage, "COMMERROR") == 0) {
        serialwrite("#ERR#03#\n", sizeof ("#ERR#03#\n"));
        clearMessage();
        
    } 
    else {
        clearMessage();
    }

}


int str2float(char msg[4]) {
    int resp = 0, base = 1000;
    resp = ((int) msg[0] - 48) * base;

    base = 100;
    resp = resp + (((int) msg[1] - 48) * base);

    base = 10;
    resp = resp + (((int) msg[2] - 48) * base);

    base = 1;
    resp = resp + (((int) msg[3] - 48) * base);

    return resp;
}

void float2str(int num) {


    int base, nchar, i;
    char message[10], crc[2];


    for (i = 0; i < 10000; i++)
        asm volatile("nop");



    if (num < 0)
        num *= -1;

    if (num > 100000) {
        serialwrite("#ERROR#05#\n", sizeof ("#ERROR#05#\n"));
        //Serial.println("error");
    } else {
        for (i = num, base = 1; i >= 10; base *= 10) //identifica o peso maximo do numero
            i /= 10;


        for (nchar = 0; base >= 1; base /= 10, nchar++) {
            message[nchar] = (char) (num / base + 48);
            num = num % base;
        }
        message[nchar] = '\0';


    if (num < 0)
        num *= -1;

    if (num > 100000) {
        serialwrite("#ERROR#05#\n", sizeof ("#ERROR#05#\n"));
        //Serial.println("error");
    } else {
        for (i = num, base = 1; i >= 10; base *= 10) //identifica o peso maximo do numero
            i /= 10;

        crc[0] = '0';
        crc[1] = (char) (nchar + 48);

        serialwritechar('#');
        serialwrite(message, nchar + 1);
        serialwritechar('#');
        serialwrite(crc, 3);
        serialwritechar('#');
        //serialwritechar('\0');

        /*  Serial.write('#');
        Serial.print(message);
        Serial.write('#');
        Serial.print(crc);
        Serial.write('#');
         */
    }

}

void float2strGain100(int num) {

    int base, nchar, i;
    char message[10], crc[2];


    for (i = 0; i < 10000; i++)
        asm volatile("nop");



    if (num < 0)
        num *= -1;

    if (num > 100000) {
        serialwrite("#ERROR#05#\n", sizeof ("#ERROR#05#\n"));
    }
    else {
        for (i = num, base = 1; i >= 10; base *= 10) //identifica o peso maximo do numero
            i /= 10;

        if (num > 100) {

            for (nchar = 0; base >= 1; base /= 10, nchar++) {
                if (base == 100) {
                    message[nchar] = (char) (num / base + 48);
                    nchar++;
                    message[nchar] = '.';
                    num = num % base;
                } else {
                    message[nchar] = (char) (num / base + 48);
                    num = num % base;
                }
            }
        } else {

            message[0] = '0';
            message[1] = '.';
            message[2] = (char) (num / 10 + 48);
            message[3] = (char) (num % 10 + 48);
            nchar = 4;
        }

        message[nchar] = '\0';


        crc[0] = '0';
        crc[1] = (char) (nchar + 48);

        serialwritechar('#');
        serialwrite(message, nchar + 1);
        serialwritechar('#');
        serialwrite(crc, 3);
        serialwrite("#\n", 2);
    }

}

void float2strGain10(int num) {

    int base, nchar, i;
    char message[10], crc[2];


    for (i = 0; i < 10000; i++)
        asm volatile("nop");



    if (num < 0)
        num *= -1;

    if (num > 100000) {
        serialwrite("#ERROR#05#\n", sizeof ("#ERROR#05#\n"));
    } else {
        for (i = num, base = 1; i >= 10; base *= 10) //identifica o peso maximo do numero
            i /= 10;


        if (num > 10) {

            for (nchar = 0; base >= 1; base /= 10, nchar++) {
                if (base == 10) {
                    message[nchar] = (char) (num / base + 48);
                    nchar++;
                    message[nchar] = '.';
                    num = num % base;
                } else {
                    message[nchar] = (char) (num / base + 48);
                    num = num % base;
                }
            }
        } else {

            message[0] = '0';
            message[1] = '.';
            message[2] = (char) (num + 48);
            nchar = 3;
        }



        message[nchar] = '\0';


        crc[0] = '0';
        crc[1] = (char) (nchar + 48);

        serialwritechar('#');
        serialwrite(message, nchar + 1);
        serialwritechar('#');
        serialwrite(crc, 3);
        serialwrite("#\n", 2);


    }

}

void clearMessage() {

    toSendCRC = 0;
    nbytes = 0;
    receivedMessage[0] = '\0';
}

void sendACK() {

    /*  Serial.print("ACK");
      Serial.write('#');
      Serial.print("03");
      Serial.write('#');
     */

}

void sendTimeout() {
    /*   
      Serial.write('#');
      Serial.print("TIMEOUT");
      Serial.write('#');
      Serial.print("07");
      Serial.write('#');
     */

}

void sendInvalidComamnd() {
    /*
  Serial.write('#');
  Serial.print("INVALID");
  Serial.write('#');
  Serial.print("07");
  Serial.write('#');
     */
}