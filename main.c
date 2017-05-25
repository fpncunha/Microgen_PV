#include <libpic30.h>
#include <p30f4011.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "microgGen_serial.h"
#include "microgen.h"


_FOSC(CSW_FSCM_OFF& FRC_PLL16); // 
_FWDT(WDT_OFF); // Watchdog off
_FBORPOR(MCLR_EN & PWRT_OFF & BORV27); // Disable MCLR reset pin aSnd turn off the power-up timers

/***************************************/
/************ Definitions **************/
#define AD_FS 5 //Full-scale ADC voltage
#define AD16Bit_FS 65535 //16bit resulution - 2^16
#define HY15P_IN 15 //HY15-P transducer nominal current
#define HY15P_VN 4 //HY15-P transducer max. output voltage (+-4V)
#define AT_PV 12 //PV gain
#define AT_VDC 101 //Vdc gain
#define NUM_FILTERS 7 //Number of filters used
#define VDC_MIN 340  //DC BUS Threshold lower limit
#define VDC_MAX 440 //DC BUS Threshold upper limit
#define DCBUS_OK 0 //DC BUS OK flag VDC > VDC_MIN && VDC < VDC_MAX
#define DCBUS_NOK 1 //DC BUS NOT OK flag VDC < VDC_MIN || VDC > VDC_MAX



//#define START_MSG 0   //first  # 
//#define MSG_DATA  1   //second #
//#define CRC_DATA  2   //third  #

#define CRC_ERROR "CRC_ERROR"
#define NOT_DEF   "NOT_DEF"
#define CRC_NUM   "CRC_NUM"
#define TIMEOUT_PROTOCOL   1500
/***************************************/
/***************************************/


/*******************************************/
/***********  Function Headers *************/
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void); // 1Khz interrupt
void configure_pins();
void set_duty_cycle(float duty);
unsigned intreadAnalogChannel(int n);
void txInt(unsigned int variable);
void txFloat(float variable);
void txChar(char caracter);
unsigned int number(unsigned int y, unsigned int operator);
void print_data(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, float average_PowerPhotovoltaic, float average_voltageOutput, unsigned int duty);
void print_data_parsing(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, float average_PowerPhotovoltaic, float average_voltageOutput, unsigned int duty);
void print_header(char* msg, int count);
float filter_250hz(float input_value, int filter_number);
float filter_10hz(float input_value, int filter_number);
float filter_1hz(float input_value, int filter_number);
float filter_01hz(float input_value, int filter_number);
void run_mppt(float vfilt, float vfilt_ant, float ifilt, float ifilt_ant);
void turn_on(void);
void turn_off(void);
void DCBUS_lock(void);
unsigned int readAnalogChannel(int channel);
void Task_Handler(void);
void Serial_monitor(void);
void TaskHandler(void);
void init_filters(void);
void updateCommCounter(void);
void TaskCommBroadcast(void);
void TaskCommBroadcast_parsing(void);
void toggleCommBroadcast(void);
void toggleCommBroadcast_parsing(void);
void synchronizeCommData(void);
/*******************************************/
/*******************************************/


/******************************/

/*******  STATE MACHINE *******/
typedef enum SystemState_t {
    STATE_INITIAL, //INITIAL STATE
    STATE_OFF, //TURN OFF STATE
    STATE_ON, //TURN ON STATE
    STATE_LOCKED, //VDC LOCK STATE (DCBUS_NOK DETECTED)
    STATE_IDLE_OFF, //OFF STATE
    STATE_IDLE_ON, //ON STATE
    STATE_LOCKED_IDLE, //LOCKED STATE (UNTIL DCBUS_OK)
} SystemState_t;
/******************************/
/******************************/

/*******  STATE MACHINE *******/
typedef enum Microgen_state_t {
    START_MSG, //INITIAL STATE
    MSG_DATA, //TURN OFF STATE
    CRC_DATA, //TURN ON STATE
    
} Microgen_state_t;
/******************************/
/******************************/



/********************************************/
/******************* Variables *************/
float duty = 0; //Duty cycle initialization
int mppt_counter = 0; //MPPT counter initialization
int comm_counter = 0; //USART broadcast counter initialization (1s)
int sync_counter = 0; //microgen sync counter initialization (1s)
int DCBUS_flag = DCBUS_NOK;
int unlock_count = 0;
int serial_com = 1;
int toggle_broadcast_aux = 0;
int toggle_broadcast_parsing_aux = 0;
volatile SystemState_t System_state = STATE_INITIAL;
int taskHander_runflag = 0; //taskHander flag - synchronize state machine with the main 1kHz interrupt
int taskCommBroadcast_runflag = 0; //broadcast to USART
int taskCommBroadcast_parsing_runflag = 0;
int timeoutProtocol = 0;
volatile Microgen_state_t Microgen_state = START_MSG;

float v0, v1, v2, analog_voltagePhotovoltaic, analog_currentPhotovoltaic, analog_voltageOutput;
float voltagePhotovoltaic, currentPhotovoltaic, voltageOutput, PowerPV;
long clockFrequency = 30000000; // 30MHz serial_cmd clock
/********************************************/
/********************************************/



/******************************************/
/********** Filters' variables ************/
float value_ant1[NUM_FILTERS];
float value_filt_ant1[NUM_FILTERS];
float value_filt[NUM_FILTERS];

float voltagePV_filtered_01Hz = 0;
float currentPV_filtered_01Hz = 0;
float voltageDCBUS_filtered_01Hz = 0;
float powerPV_filtered_01Hz = 0;

float voltagePV_filtered_1Hz = 0;
float currentPV_filtered_1Hz = 0;
float voltageDCBUS_filtered_1Hz = 0;
float powerPV_filtered_1Hz = 0;

float voltagePV_filtered_10Hz = 0;
float currentPV_filtered_10Hz = 0;
float voltageDCBUS_filtered_10Hz = 0;
float powerPV_filtered10Hz = 0;

float voltagePV_filtered_250Hz = 0;
float currentPV_filtered_250Hz = 0;
float voltageDCBUS_filtered_250Hz = 0;
float powerPV_filtered_250Hz = 0;

float voltagePV_filtered_ant = 0;
float currentPV_filtered_ant = 0;
float powerPV_filtered_ant = 0;
/******************************************/
/******************************************/


/*******************************************/
/************* Serial Com. ****************/
char Message0[] = "\n 1-Turn on System";
char MessageA[] = "\n 0-Turn off System";
char Message1[] = "\n V_PV=";
char MessageB[] = "\n R-Read Data";
char Message2[] = "\n I_PV=";
char Message3[] = "\n Duty=";
char Message4[] = "\n V_out=";
char Message5[] = "\n Power=";
char Message7[] = "\n ***System ON*** \n";
char Message8[] = "\n ***System OFF*** \n";
char MessageX[] = "\nDUTY";
int n;
/*******************************************/
/************* protocol ********************/
//extern char receivedMessage[100];
//boolean status_command;
//extern int STATE_cardinal = 0;
//extern int toSendCRC = 0, nbytes = 0, cnt = 0;
/*******************************************/

/*******************************************/
void synchronizeCommData(void) {
    float powerPV;
    pv2rpi_.DCBUS = (int) (10 * voltageDCBUS_filtered_10Hz);
    pv2rpi_.VPV = (int) (10 * voltagePV_filtered_10Hz);
    pv2rpi_.IPV = (int) (10 * currentPV_filtered_10Hz);
    powerPV = voltagePV_filtered_10Hz * currentPV_filtered_10Hz;
    pv2rpi_.PWR = (int) (10 * powerPV);

    //print_data_parsing(voltagePV_filtered_10Hz, currentPV_filtered_10Hz, powerPV_filtered_01Hz, voltageDCBUS_filtered_10Hz, PDC1);
}

void updateCommCounter(void) {
    if (comm_counter >= 1000 && toggle_broadcast_aux == 1) {
        TaskCommBroadcast();
        comm_counter = 0;
    } else if (comm_counter >= 1000 && toggle_broadcast_parsing_aux == 1) {
        TaskCommBroadcast_parsing();
        comm_counter = 0;
    } else {
        comm_counter += 1;
    }

    if (sync_counter >= 1000) {
        sync_counter = 0;
        synchronizeCommData();
    } else {
        sync_counter += 1;
    }


}

void toggleCommBroadcast(void) {
    taskCommBroadcast_runflag = (taskCommBroadcast_runflag == 1) ? 0 : 1;
}

void toggleCommBroadcast_parsing(void) {
    taskCommBroadcast_parsing_runflag = (taskCommBroadcast_parsing_runflag == 1) ? 0 : 1;
}
/***** broadcast de comunica��o a 1seg *****/

/*******************************************/
void TaskCommBroadcast(void) {
    if (taskCommBroadcast_runflag == 1) {
        print_data(voltagePV_filtered_10Hz, currentPV_filtered_10Hz, powerPV_filtered_01Hz, voltageDCBUS_filtered_10Hz, PDC1);
    }
}

void TaskCommBroadcast_parsing(void) {
    if (taskCommBroadcast_parsing_runflag == 1) {
        print_data_parsing(voltagePV_filtered_10Hz, currentPV_filtered_10Hz, powerPV_filtered_01Hz, voltageDCBUS_filtered_10Hz, PDC1);
    }
}
/************* Serial Com. ****************/

/*******************************************/


void init_filters(void) {
    int aux_i = 0;
    for (aux_i = 0; aux_i < NUM_FILTERS; aux_i++) {
        value_ant1[aux_i] = 0;
        value_filt_ant1[aux_i] = 0;
        value_filt[aux_i] = 0;
    }
}

/*******************************************/

void Serial_monitor(void) {

    int y = 0;
    
    char receivedByte = 0;
    int i = 0;

    if (U2STAbits.URXDA == 1) { //Checks if received a character

        if (commStatus == PROTOCOL_DIS) {
            y = U2RXREG; //Y = received character
            if (y == '1' && DCBUS_flag == DCBUS_OK) {
                toggle_broadcast_parsing_aux = 0;
                toggle_broadcast_aux = 0;
                serial_com = 1;
                System_state = STATE_ON;
                //taskHander_runflag = 1;
            }

            if (y == '1' && DCBUS_flag == DCBUS_NOK) {
                toggle_broadcast_parsing_aux = 0;
                toggle_broadcast_aux = 0;
                serial_com = 1;
                System_state = STATE_LOCKED;
                //taskHander_runflag = 1;
            }

            if ((y == '0')) {
                toggle_broadcast_parsing_aux = 0;
                toggle_broadcast_aux = 0;
                serial_com = 0;
                System_state = STATE_OFF;
                //  taskHander_runflag = 1;
            }
            if (y == 'r' || y == 'R') {
                //taskCommBroadcast_runflag = 1;
            }
            if (y == 'b' || y == 'B') {
                toggle_broadcast_aux = 1;
                toggle_broadcast_parsing_aux = 0;
                toggleCommBroadcast();
            }
            if (y == 'e' || y == 'E') {
                toggle_broadcast_parsing_aux = 1;
                toggle_broadcast_aux = 0;
                toggleCommBroadcast_parsing();

            }
            if (y == 'p' || y == 'P') {
                commStatus = PROTOCOL_ENA;
                toggle_broadcast_parsing_aux = 0;
                toggle_broadcast_aux = 0;
            }
        }//*** end of protocol DISABLE***///
        else {
            //////////////////////////////////////////////////
            /////////////   MICROGEN  SERIAL PROTOCOL    /////
            
            
            receivedByte =  U2RXREG;
            //receivedMessage[nbytes] = U2RXREG;
           /* if (timeoutProtocol > TIMEOUT_PROTOCOL) {
                timeoutProtocol = 0;
                clearMessage();
                nbytes = 0;
                Microgen_state = START_MSG;
                LATBbits.LATB4 = 1;
                for (i = 0; i < 500; i++)
                    asm volatile("nop");
                LATBbits.LATB4 = 0;
            }*/
            
            switch (Microgen_state) {
                case START_MSG:
                {
                    if (receivedByte == '{') {
                        LATBbits.LATB6 = 1; //Debug only - Toggle B8 pin
                        clearMessage();
                        Microgen_state = MSG_DATA;
                    }
                    else {
                        //do nothing;
                        nbytes = 0;
                        receivedMessage[nbytes] = '\0';
                    }
                  //  timeoutProtocol = 0;
                }
                    break;
                case MSG_DATA:
                {
                    LATBbits.LATB5 = 1; 
                    if (receivedByte == '#') {
                        receivedMessage[nbytes] = '\0';
                        cnt = 10;
                        Microgen_state = CRC_DATA;
                       // timeoutProtocol = 0;
                    }
                    else  if (receivedByte == '{' || receivedByte == '}') {
                        clearMessage();
                        Microgen_state = START_MSG;
                    }
                    else {
                        receivedMessage[nbytes] = receivedByte;
                        nbytes++; 
                    }
                    LATBbits.LATB5 = 0; 
                }
                    break;
                case CRC_DATA:
                {
                    LATBbits.LATB4 = 1; 
                    if (receivedByte == '}') {

                        if (nbytes == toSendCRC) {
                            receivedMessage[nbytes] = '\0';
                            manageMessage();
                            LATBbits.LATB6 = 0; //Debug only - Toggle B8 pin
                            Microgen_state = START_MSG;
                        } 
                        else {
                           // strcpy(receivedMessage, "COMMERROR");
                            //receivedMessage[strlen("COMMERROR")+1] = '\0';
                            //manageMessage();
                            LATBbits.LATB6 = 0; //Debug only - Toggle B8 pin
                            clearMessage();
                            Microgen_state = START_MSG;
                        }
                        timeoutProtocol = 0;
                    }
                    else  if (receivedByte == '#' || receivedByte == '{') {
                        clearMessage();
                        Microgen_state = START_MSG;
                    }
                    else {
                        toSendCRC += (receivedByte - 48) * cnt;
                        if (cnt < 1) {
                            clearMessage();
                            Microgen_state = START_MSG;
                        }
                        cnt /= 10; 
                    }
                    LATBbits.LATB4 = 0; 
                }
                    break;

                default:
                    break;
            }
            
           
             //////////////////////////////////////////////////
            //////   END of MICROGEN  SERIAL PROTOCOL    /////
            
        }
        
       /* if (timeoutProtocol > TIMEOUT_PROTOCOL) {
            LATBbits.LATB5 = 1; //Debug only - Toggle B8 pin
            timeoutProtocol = 0;
            clearMessage();
            Microgen_state = START_MSG;
            LATBbits.LATB5 = 0; //Debug only - Toggle B8 pin
        }
        */ 

    }
}

void turn_on(void) {
    _LATD2 = 1;
    _LATD3 = 1; //Reset
    __delay32(0.5 * clockFrequency);
    _LATD3 = 0;
}

void turn_off(void) {
    set_duty_cycle(0);
}

void DCBUS_lock(void) {

    if (DCBUS_flag == DCBUS_OK) {
        unlock_count++;
    } else {
        unlock_count = 0;
    }

    if (unlock_count >= 500 && serial_com == 1) {
        unlock_count = 0;
        System_state = STATE_ON;
    }
}

void run_mppt(float vfilt, float vfilt_ant, float ifilt, float ifilt_ant) {
    float dI, dV;
    float dutyMax = 0.9;
    float dutyMin = 0.5;

    float dutyStep = 0.002;
    float deltaV = 0.01;
    float deltaI = 0.01;
    float deltaG = 0.01;

    float auxG, auxDG, G;

    dI = ifilt - ifilt_ant;
    dV = vfilt - vfilt_ant;
    if ((dV <= deltaV) && (dV >= -deltaV)) {
        if ((dI <= deltaI) && (dI >= -deltaI)) {
            //Do nothing
        } else {
            if (dI > deltaI) {
                duty -= dutyStep;
            } else {
                duty += dutyStep;
            }
        }
    } else {
        auxG = ifilt / vfilt;
        auxDG = dI / dV;
        G = auxG + auxDG;
        if ((G <= deltaG) && (G >= -deltaG)) {
            //Do nothing
        } else {
            if (G > deltaG) {
                duty -= dutyStep;
            } else {
                duty += dutyStep;
            }
        }
    }
    //Saturates PWM between 0,4 and 0,9
    if (duty >= dutyMax) {
        duty = dutyMax;
    }
    if (duty <= dutyMin) {
        duty = dutyMin;
    }

    set_duty_cycle(duty); //Update duty cycle

}

void set_duty_cycle(float duty) //Delays PWM's 180
{
    PDC1 = duty * (2 * 301.5);
    PDC2 = (1 - duty) * (2 * 301.5);
}

void configure_pins() {
    //Configure Port D and B
    LATD = 0; //Output equals 0
    TRISD = 0b11110000; //RD0 and RD1 output, RD2 to RD8 input
    TRISB = 0x01FF; //Ports B are inputs
    //TRISBbits.TRISB8 = 0; //Debug only - Pin B8 is configured as output

    // Configure TIMER1
    // Timer1 = PR1 * PRESCALER * (1/Fclk) = 3685*8*(1/30000000) = 1ms = 1kHz //
    T1CON = 0; //Clear Timer 1 configuration
    T1CONbits.TCKPS = 1; //Set timer 1 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
    PR1 = 3685; //Set Timer 1 period (max value is 65535) 16bit
    _T1IP = 1; //Set Timer 1 interrupt priority
    _T1IF = 0; //Clear Timer 1 interrupt flag
    _T1IE = 1; //Enable Timer 1 interrupt
    T1CONbits.TON = 1; //Turn on Timer 1

    //Configure ADC
    ADPCFG = 0xFF00; //Lowest 8 PORTB pins are analog inputs
    ADCON1 = 0x0000; //Manually clear SAMP to end sampling, start
    ADCON1bits.FORM = 2; //Output format - =2 means fraction
    ADCSSL = 0; //ADC input scan select register
    ADCON2 = 0; //Voltage reference from AVDD and AVSS
    ADCON3 = 0x0005; //Manual Sample, ADCS=5 -> Tad = 3*Tcy
    ADCHS = 0x0000; //Selects the inp\ut channels to be converted
    ADCON1bits.ADON = 1; //Turn ADC ON

    //Configuration of PWMs
    PWMCON1 = 0b00110011; //Enable channels 1 & 2 in complementary mode
    PWMCON2 = 0; //Bit 0 UDIS = 0 - Updates from duty cycle and period buffer registers are enabled
    //Bit 1 OSYNC = 0 - Output overrides via the OVDCON register occur on next Tcy boundary
    //Bit 2 IUE = 0 - Updates to the active PDC registers are synchronized to the PWM time base
    //Bit 11-8 SEVOPS = 0000 -  PWM Special Event Trigger Output Postscale Select bits (1:1 Postscale)
    PTCON = 0;
    _PTCKPS = 0; //Prescale=1:1 (0=1:1, 1=1:4, 2=1:16, 3=1:64)
    _PTMOD = 0b10; //Select up-down counting for centre-aligned PWM
    PTPER = 300; //The PTPER register sets the counting period for PTMR // 50kHz in up/down counting mode
    PTMR = 0; //Clear 15-bit PWM timer counter
    _PTEN = 1; //Enable PWM time base

    //Serial Port
    //8 data bits, 1 stop bit, no parity bit
    U2BRG = 32; //97; //Baudrate = 19200 bits per second
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXISEL = 1;
    U2STAbits.UTXEN = 1;


}

unsigned int readAnalogChannel(int channel) {
    ADCHS = channel;
    ADCON1bits.SAMP = 1;
    __delay32(1);
    ADCON1bits.SAMP = 0;
    while (!ADCON1bits.DONE) {
        // Do nothing
    }
    return ADCBUF0;
}

void txInt(unsigned int variable) {
    unsigned int aux_var = variable;

    if (variable >= 10000) {
        aux_var = number(aux_var, 10000);
        aux_var = number(aux_var, 1000);
        aux_var = number(aux_var, 100);
        aux_var = number(aux_var, 10);
        aux_var = number(aux_var, 1);
    }
    if (variable >= 1000 && variable < 10000) {
        aux_var = number(aux_var, 1000);
        aux_var = number(aux_var, 100);
        aux_var = number(aux_var, 10);
        aux_var = number(aux_var, 1);
    }
    if (variable >= 100 && variable < 1000) {
        aux_var = number(aux_var, 100);
        aux_var = number(aux_var, 10);
        aux_var = number(aux_var, 1);
    }
    if (variable >= 10 && variable < 100) {
        aux_var = number(aux_var, 10);
        aux_var = number(aux_var, 1);
    }
    if (variable < 10) {
        aux_var = number(aux_var, 1);
    }
}

void txFloat(float variable) {
    float f2;
    int d1, d2;
    d1 = (int) variable;
    f2 = variable - d1;
    d2 = (f2 * 1000);
    txInt((unsigned int) d1);
    txChar('.');
    txInt((unsigned int) d2);
    txInt(d2);
}

unsigned int number(unsigned int y, unsigned int operator) {
    while (U2STAbits.UTXBF) {
        //Waits until buffer is full
    }

    U2TXREG = (y / operator) + 48;
    y = (y % operator);
    return y;
}

void txChar(char txChar) {
    while (U2STAbits.UTXBF) {
        //waits until buffer is full
    }

    U2TXREG = txChar;
}

void print_data(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, float average_PowerPhotovoltaic, float average_voltageOutput, unsigned int duty) {
    print_header(Message1, sizeof (Message1));
    txFloat(average_voltagePhotovoltaic);
    txChar('V');
    print_header(Message2, sizeof (Message2));
    txFloat(average_currentPhotovoltaic);
    txChar('A');
    print_header(Message5, sizeof (Message5));
    txFloat(average_PowerPhotovoltaic);
    txChar('W');
    print_header(Message4, sizeof (Message4));
    txFloat(average_voltageOutput);
    txChar('V');
    print_header(Message3, sizeof (Message3));
    txFloat(duty / (2 * 3.015));
    txChar('%');
}

void print_data_parsing(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, float average_PowerPhotovoltaic, float average_voltageOutput, unsigned int duty) {
    txFloat(average_voltagePhotovoltaic);
    txChar(';');
    txFloat(average_currentPhotovoltaic);
    txChar(';');
    txFloat(average_PowerPhotovoltaic);
    txChar(';');
    txFloat(average_voltageOutput);
    txChar(';');
    txFloat(duty / (2 * 3.015));
    txChar('\n');
}

void print_header(char* msg, int count) {
    for (n = 0; n < (count - 1); n++) {
        while (U2STAbits.UTXBF) {
            // do nothing
        }
        U2TXREG = msg[n];
    }
}

float filter_250hz(float input_value, int filter_number) {
    //Filtro a 250Hz primeira ordem: coeficientes obtidos em \..\3. Simula��o\3. controlo e�lica\filtro.m
    //Values(i) = (b(1)*signal(i)+b(2)*signal(i-1)-a(2)*values(i-1))/a(1);

    //A1 = 1
    //A2 = -5.551115123125783e-17
    //B1 = 0.500000000000000
    //B2 = 0.500000000000000
    //value_filt[filter_number] = ( (B1*input_value + B2*value_ant1[filter_number] - A2*value_filt_ant1[filter_number]));
    value_filt[filter_number] = ((0.5 * input_value + 0.5 * value_ant1[filter_number]));
    value_ant1[filter_number] = input_value;
    value_filt_ant1[filter_number] = value_filt[filter_number];
    return value_filt[filter_number];
}

float filter_10hz(float input_value, int filter_number) {
    //Filtro a 10Hz primeira ordem: coeficientes obtidos em \..\3. Simula��o\3. controlo e�lica\filtro.m
    //Values(i) = (b(1)*signal(i)+b(2)*signal(i-1)-a(2)*values(i-1))/a(1);

    //A1 = 1
    //A2 = -0.939062505817492
    //B1 = 0.030468747091254
    //B2 = 0.030468747091254
    //value_filt[filter_number] = ( (B1*input_value + B2*value_ant1[filter_number] - A2*value_filt_ant1[filter_number]));

    value_filt[filter_number] = ((0.030468747091254 * input_value + 0.030468747091254 * value_ant1[filter_number] + 0.939062505817492 * value_filt_ant1[filter_number]));
    value_ant1[filter_number] = input_value;
    value_filt_ant1[filter_number] = value_filt[filter_number];
    return value_filt[filter_number];
}

float filter_1hz(float input_value, int filter_number) {
    //Filtro a 1Hz primeira ordem: coeficientes obtidos em \..\3. Simula��o\3. controlo e�lica\filtro.m
    //Values(i) = (b(1)*signal(i)+b(2)*signal(i-1)-a(2)*values(i-1))/a(1);

    //A1 = 1
    //A2 = -0.993736471541615
    //B1 = 0.003131764229193
    //B2 = 0.003131764229193
    //value_filt[filter_number] = ( (B1*input_value + B2*value_ant1[filter_number] - A2*value_filt_ant1[filter_number]));

    value_filt[filter_number] = ((0.003131764229193 * input_value + 0.003131764229193 * value_ant1[filter_number] + 0.993736471541615 * value_filt_ant1[filter_number]));
    value_ant1[filter_number] = input_value;
    value_filt_ant1[filter_number] = value_filt[filter_number];
    return value_filt[filter_number];
}

float filter_01hz(float input_value, int filter_number) {
    //Filtro a 0.1Hz primeira ordem: coeficientes obtidos em \..\3. Simula��o\3. controlo e�lica\filtro.m
    //Values(i) = (b(1)*signal(i)+b(2)*signal(i-1)-a(2)*values(i-1))/a(1);

    //A1 = 1
    //A2 = -0,999371878778719
    //B1 = 0.0003140606106404320
    //B2 = 0.0003140606106404320
    //value_filt[filter_number] = ( (B1*input_value + B2*value_ant1[filter_number] - A2*value_filt_ant1[filter_number]));

    value_filt[filter_number] = ((0.0003140606106404320 * input_value + 0.0003140606106404320 * value_ant1[filter_number] + 0.999371878778719 * value_filt_ant1[filter_number]));
    value_ant1[filter_number] = input_value;
    value_filt_ant1[filter_number] = value_filt[filter_number];
    return value_filt[filter_number];
}

void SysInit(void) {

    set_duty_cycle(0);
    if (DCBUS_flag == DCBUS_OK) {
        unlock_count++;
    } else {
        unlock_count = 0;
    }

    if (unlock_count >= 500 && serial_com == 1) {
        unlock_count = 0;
        System_state = STATE_ON;
    }
}

void TaskHandler(void) {
    switch (System_state) {
        case STATE_INITIAL:
        {
            SysInit(); //inicializa o systema e valida se DCBUS_OK. Se DCBUS_OK e serial_com == 1 passa para STATE_ON
            set_duty_cycle(0.5);
        }
            break;

        case STATE_ON: //Liga MPPT e passa para o estado IDLE_ON
        {
            turn_on();
            System_state = STATE_IDLE_ON;
        }
            break;

        case STATE_OFF: //Desliga MPPT e passa para o estado IDLE_OFF
        {
            turn_off();
            System_state = STATE_IDLE_OFF;
        }
            break;

        case STATE_LOCKED: //Desliga MPPT, DCBUS_NOK e passa para LOCKED_IDLE
        {
            turn_off();
            System_state = STATE_LOCKED_IDLE;
        }

        case STATE_LOCKED_IDLE: //Verifica continuamente VDC at� que DCBUS_OK
        {
            DCBUS_lock();

        }
            break;

            break;

        case STATE_IDLE_ON: //N�o faz nada
        {
        }
            break;

        case STATE_IDLE_OFF: //N�o faz nada
        {
        }
            break;

        default:
            break;
    }
}

int main() {
    TRISD = 0b11111100; //RD0 and RD1 output, RD2 to RD8 input
    _LATD1 = 1;
    //LATBbits.LATB8 = 1;
    
    set_duty_cycle(0);

    configure_pins(); 
    
    TRISBbits.TRISB8 = 0;   //pin 10 output
    TRISBbits.TRISB7 = 0;   //pin 9 output
    TRISBbits.TRISB6 = 0;   //pin 8 output
    TRISBbits.TRISB5 = 0;   //pin 9 output
    TRISBbits.TRISB4 = 0;   //pin 8 output
    init_filters();

    while (1) {
 
        if (taskHander_runflag == 1) {
            TaskHandler();
            taskHander_runflag = 0;
        }
        Serial_monitor();
  
    }

    return 0;

}

void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void) {

    _T1IF = 0; // Clear Timer 1 interrupt flag
    LATBbits.LATB8 = 1; //Debug only - Toggle B8 pin

    /****************************************/
    /*************  ACQUISITION  ************/
    //Reads de current from the PV panel (IPV)
    analog_currentPhotovoltaic = readAnalogChannel(0);
    v0 = (analog_currentPhotovoltaic * AD_FS) / AD16Bit_FS;
    currentPhotovoltaic = (v0 * HY15P_IN) / HY15P_VN;
    currentPhotovoltaic = (currentPhotovoltaic * 1.0125 + 0.24); //correc��o do ADC com base em ensaios em pot�ncia
    //Reads the voltage from de photovoltaic panel (VPV)

    analog_voltagePhotovoltaic = readAnalogChannel(1);
    v1 = (analog_voltagePhotovoltaic * AD_FS) / AD16Bit_FS;

    voltagePhotovoltaic = v1 * AT_PV;
    voltagePhotovoltaic = (voltagePhotovoltaic * 0.8912 + 0.63); //correc��o do ADC com base em ensaio de fonte de tens�o
    // Reads the output voltage (VDC)
    analog_voltageOutput = readAnalogChannel(2);
    v2 = (analog_voltageOutput * AD_FS) / AD16Bit_FS;
    voltageOutput = v2 * AT_VDC;
    voltageOutput = (voltageOutput * 0.981 + 5.89); //correc��o do ADC com base em ensaio de fonte de tens�o


    PowerPV = currentPhotovoltaic * voltagePhotovoltaic;

    // Filters
    voltagePV_filtered_10Hz = filter_10hz(voltagePhotovoltaic, 0);
    currentPV_filtered_10Hz = filter_10hz(currentPhotovoltaic, 1);
    voltageDCBUS_filtered_10Hz = filter_10hz(voltageOutput, 2);
    /*************  ACQUISITION  ************/
    /****************************************/

    /******************************************/
    /*************  DC PROTECTION  ************/
    /******** 250HZ filter + overvoltage and undervoltage protection - Vdc Min. = X; Vdc Max. = X ********/
    voltagePV_filtered_250Hz = filter_250hz(voltagePhotovoltaic, 3);
    currentPV_filtered_250Hz = filter_250hz(currentPhotovoltaic, 4);
    voltageDCBUS_filtered_250Hz = filter_250hz(voltageOutput, 5);
    powerPV_filtered_01Hz = filter_1hz(PowerPV, 6);

    if ((voltageDCBUS_filtered_250Hz > VDC_MIN && voltageDCBUS_filtered_250Hz < VDC_MAX)) {
        DCBUS_flag = DCBUS_OK;
    } else {

        if ((System_state = !STATE_INITIAL) || (System_state = !STATE_LOCKED) || (System_state = !STATE_LOCKED_IDLE)) {
            System_state = STATE_LOCKED;
            // txFloat(voltageOutput);
            //  turn_off();
        }
        DCBUS_flag = DCBUS_NOK;

    }
    /*************  DC PROTECTION  ************/
    /******************************************/

    /*********************************/
    /*************  MPPT  ************/
    //TODO: ver se o else n�o inclui "ifs" em que o mppt � maior que 100
    if ((mppt_counter >= 100) && (System_state == STATE_IDLE_ON)) {
        //__delay32(10000); //Random delay
        run_mppt(voltagePV_filtered_10Hz, voltagePV_filtered_ant, currentPV_filtered_10Hz, currentPV_filtered_ant);
        mppt_counter = 0;
        voltagePV_filtered_ant = voltagePV_filtered_10Hz;
        currentPV_filtered_ant = currentPV_filtered_10Hz;
    } else {
        mppt_counter++;
    }
    /*************  MPPT  ************/
    /*********************************/

    updateCommCounter();
    taskHander_runflag = 1;
    timeoutProtocol += 1;

    LATBbits.LATB8 = 0; //Debug only - Toggle B8 pin
}
