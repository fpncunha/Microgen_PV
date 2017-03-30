#include <libpic30.h>
#include <p30f4011.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

_FOSC(CSW_FSCM_OFF& FRC_PLL16); // 
_FWDT(WDT_OFF); // Watchdog off
_FBORPOR(MCLR_EN & PWRT_OFF); // Disable MCLR reset pin aSnd turn off the power-up timers


#define AD_FS 5 //Full-scale ADC voltage
#define AD16Bit_FS 65535 //16bit resulution - 2^16
#define HY15P_IN 15 //HY15-P transducer nominal current
#define HY15P_VN 4 //HY15-P transducer max. output voltage (+-4V)
#define AT_PV 12 //PV gain
#define AT_VDC 101 //Vdc gain
#define NUM_FILTERS 6 //Number of filters used
#define VDC_MIN 20
#define VDC_MAX 40

/*******************************************/
/***********  Function Headers *************/
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void); // 1Khz interrupt 
void configure_pins(); 
void set_duty_cycle(float duty); 
unsigned intreadAnalogChannel(int n);
void txInt(unsigned int variable);
void txFloat(float variable);
void txChar(char caracter);
unsigned int number(unsigned int y , unsigned int operator);
void print_data(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, unsigned int duty, float average_voltageOutput);
void print_header(char* msg, int count);
float filter_250hz(float input_value, int filter_number);
float filter_10hz(float input_value, int filter_number);
void run_mppt(float vfilt, float vfilt_ant, float ifilt, float ifilt_ant);
void turn_on(void);
void turn_off(void);
void DCBUS_lock(void);
unsigned int readAnalogChannel(int channel);
void Task_Handler (void);
void Serial_monitor (void);
void TaskHandler(void);
void init_filters(void);
/**********  Function Headers *************/
/*******************************************/

typedef enum SystemState_t {
	STATE_INITIAL,
	STATE_OFF,
	STATE_ON,
	STATE_LOCKED,
    STATE_IDLE_OFF,
    STATE_IDLE_ON,
} SystemState_t;

volatile SystemState_t System_state = STATE_INITIAL;


/********************************************/
/********** Auxiliary Variables *************/
int serial_cmd;
float f2;
unsigned int x;
int d1, d2;
float duty = 0; //Duty cycle initialization
int mppt_counter = 0; //MPPT counter initialization
int DCBUS_threshold;
int unlock_count = 0;
/********** Auxiliary Variables *************/
/********************************************/



/*******************************************/
/********** Filters' variables *************/
float value_ant1[NUM_FILTERS];
float value_filt_ant1[NUM_FILTERS];
float value_filt[NUM_FILTERS];

float voltagePV_filtered = 0;
float currentPV_filtered = 0;
float voltageDCBUS_filtered = 0;

float voltagePV_filtered250 = 0;
float currentPV_filtered250 = 0;
float voltageDCBUS_filtered250 = 0;

float voltagePV_filtered_ant = 0;
float currentPV_filtered_ant = 0;
/********** Filters' variables *************/
/*******************************************/


 float v0, v1, v2, analog_voltagePhotovoltaic, analog_currentPhotovoltaic, analog_voltageOutput;
 float voltagePhotovoltaic, currentPhotovoltaic, voltageOutput;
 long clockFrequency = 30000000; // 30MHz serial_cmd clock


/*******************************************/
/************* Serial Com. ****************/
char Message0[] = "\n 1-Turn on System";
char MessageA[] = "\n 0-Turn off System";
char Message1[] = "\n V_PV=";
char MessageB[] = "\n R-Read Data";
char Message2[] = "\n I_PV=";
char Message3[] = "\n Duty=";
char Message4[] = "\n V_out=";
char Message7[] = "\n ***System ON*** \n";
char Message8[] = "\n ***System OFF*** \n";
char MessageX[] = "\nDUTY";
int n;
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


void Serial_monitor(void){
    
    int y=0;
    
    if (U2STAbits.URXDA == 1) { //Checks if received a character
            y = U2RXREG; 
            if (y == '1' && System_state == STATE_IDLE_OFF && DCBUS_threshold == 0) {
                serial_cmd = 1;
                System_state = STATE_ON; 
            }
            if (y == '0'  && (System_state = STATE_IDLE_ON || (DCBUS_threshold == 1))) {
                serial_cmd = 0;
                System_state = STATE_OFF;
            }
            if (y == 'r' || y == 'R') {
                print_data(voltageDCBUS_filtered250, currentPV_filtered, PDC1, voltageDCBUS_filtered);
                //print_data(voltagePV_filtered, currentPV_filtered, PDC1, voltageDCBUS_filtered);
            }
    }

}


void turn_on(void){    
         _LATD2 = 1;
         _LATD0 = 1;	//reset
         __delay32(0.1 * clockFrequency);
         _LATD0 = 0;
         print_header(Message7, sizeof(Message7));  // TURN ON MESSAGE
    }
    
void turn_off(void){
    set_duty_cycle(0);
    _LATD2 = 0;
    print_header(Message8, sizeof(Message8));  // TURN OFF MESSAGE
}
 

void DCBUS_lock(void){
      
    turn_off();
    int unlock_count;
    unlock_count = 0;
    DCBUS_threshold = 1;

    
    analog_voltageOutput = readAnalogChannel(2);
    v2 = (analog_voltageOutput * AD_FS) / AD16Bit_FS;
    voltageOutput = v2 * AT_VDC;
    voltageDCBUS_filtered250 = filter_250hz(voltageOutput, 5);
    if (voltageDCBUS_filtered250 > VDC_MIN && voltageDCBUS_filtered250 < VDC_MAX)
    {
        unlock_count ++;
    }    
    
    if (unlock_count == 500){
        unlock_count = 0;
        DCBUS_threshold = 0;
        System_state = STATE_ON;
    }
    
}

void run_mppt(float vfilt, float vfilt_ant, float ifilt, float ifilt_ant)
{
    if (serial_cmd == 1){
    float dI, dV;
    float dutyMax = 0.9;
    float dutyMin = 0.4;

    float dutyStep = 0.002;
    float deltaV = 0.01;
    float deltaI = 0.01;
    float deltaG = 0.01;

    float auxG, auxDG, G;

    dI = ifilt - ifilt_ant;
    dV = vfilt - vfilt_ant;
    if ((dV <= deltaV) && (dV >= -deltaV)) {
        if ((dI <= deltaI) && (dI >= -deltaI)) {
            //do nothing
        }
        else {
            if (dI > deltaI){ 
                duty -= dutyStep;
            }
            else{
                duty += dutyStep;
            }
        }
    }
    else {
        auxG = ifilt / vfilt;
        auxDG = dI / dV;
        G = auxG + auxDG;
        if ((G <= deltaG) && (G >= -deltaG)) {
            //do nothing
        }
        else {
            if (G > deltaG){
                duty -= dutyStep;
            }
            else{
                duty += dutyStep;
            }
        }
    }
    //Saturates PWM between 0,4 and 0,9
    if (duty >= dutyMax){
        duty = dutyMax;
    }
    if (duty <= dutyMin){
        duty = dutyMin;
    }
    set_duty_cycle(duty); //Update duty cycle
    }
    }

void set_duty_cycle(float duty) //Delays PWM's 180
{
    PDC1 = duty * (2 * 301.5);
    PDC2 = (1 - duty) * (2 * 301.5);
}

void configure_pins()
{
    //Configure Port D and B
    LATD = 0; //Output equals 0
    TRISD = 0b11111100; //RD0 and RD1 output, RD2 to RD8 input
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
    ADCHS = 0x0000; //Selects the input channels to be converted
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
    U2BRG = 97; //Baudrate = 19200 bits per second
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXISEL = 1;
    U2STAbits.UTXEN = 1;
}

unsigned int readAnalogChannel(int channel)
{
    ADCHS = channel;
    ADCON1bits.SAMP = 1;
    __delay32(1);
    ADCON1bits.SAMP = 0;
    while (!ADCON1bits.DONE){
        // do nothing
    }
    return ADCBUF0;
}

void txInt(unsigned int variable)
{
    x = variable;
    if (variable >= 10000) {
        x = number(x, 10000);
        x = number(x, 1000);
        x = number(x, 100);
        x = number(x, 10);
        x = number(x, 1);
    }
    if (variable >= 1000 && variable < 10000) {
        x = number(x, 1000);
        x = number(x, 100);
        x = number(x, 10);
        x = number(x, 1);
    }
    if (variable >= 100 && variable < 1000) {
        x = number(x, 100);
        x = number(x, 10);
        x = number(x, 1);
    }
    if (variable >= 10 && variable < 100) {
        x = number(x, 10);
        x = number(x, 1);
    }
    if (variable < 10)
        x = number(x, 1);
}

void txFloat(float variable)
{
    d1 = (int)variable;
    f2 = variable - d1;
    d2 = (f2 * 1000);
    txInt(d1);
    txChar('.');
    txInt(d2);
}

unsigned int number(unsigned int y, unsigned int operator)
{
    while (U2STAbits.UTXBF){
         //Waits until buffer is full
    }
        
    U2TXREG = (y / operator) + 48;
    y = (y % operator);
    return 0;
}

void txChar(char txChar)
{
    while (U2STAbits.UTXBF){
        //waits until buffer is full
    }
     
    U2TXREG = txChar;
}

void print_data(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, unsigned int duty, float average_voltageOutput)
{
    print_header(Message1, sizeof(Message1));
    txFloat(average_voltagePhotovoltaic);
    txChar('V');
    print_header(Message2, sizeof(Message2));
    txFloat(average_currentPhotovoltaic);
    txChar('A');
    print_header(Message3, sizeof(Message3));
    txFloat(duty / (2 * 3.015));
    txChar('%');
    print_header(Message4, sizeof(Message4));
    txFloat(average_voltageOutput);
    txChar('V');
}

void print_header(char* msg, int count)
{
    for (n = 0; n < (count - 1); n++) {
        while (U2STAbits.UTXBF){
            // do nothing
        }
         //Waits until buffer is full
        U2TXREG = msg[n];
    }
}

float filter_250hz(float input_value, int filter_number)
{
    //Filtro a 200Hz primeira ordem: coeficientes obtidos em \..\3. Simulação\3. controlo eólica\filtro.m
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

float filter_10hz(float input_value, int filter_number)
{
    //Filtro a 200Hz primeira ordem: coeficientes obtidos em \..\3. Simulação\3. controlo eólica\filtro.m
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


void SysInit(void){
            
    analog_voltageOutput = readAnalogChannel(2);
    v2 = (analog_voltageOutput * AD_FS) / AD16Bit_FS;
    voltageOutput = v2 * AT_VDC;
    voltageDCBUS_filtered250 = filter_250hz(voltageOutput, 5);
    if (voltageDCBUS_filtered250 > VDC_MIN && voltageDCBUS_filtered250 < VDC_MAX)
    {
        unlock_count ++;
    }    
    
    if (unlock_count == 500){
        unlock_count = 0;
        DCBUS_threshold = 0;
        turn_on();
        System_state = STATE_ON;
    }
}

void TaskHandler(void)
{
	switch (System_state) {
		case STATE_INITIAL:
		{
			SysInit();
		}
		break;

		case STATE_ON:
		{ 
            turn_on();
            System_state = STATE_IDLE_ON;
		}
		break;
		
		case STATE_OFF:
		{
            turn_off();
            System_state = STATE_IDLE_OFF;
		}
		break;

		case STATE_LOCKED:
		{
			DCBUS_lock();
		}
		break;
        
        case STATE_IDLE_ON:
		{
            
		}
		break;
        
        case STATE_IDLE_OFF:
		{
            
		}
		break;
        
		default:
		break;
	}
}

int main()
{
    
    configure_pins();
    init_filters();
    
    while (1) {

		Serial_monitor();
        TaskHandler();
        
    }

    return 0;

}


void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
   
    _T1IF = 0; // Clear Timer 1 interrupt flag
    
    _LATD1 = 1 - _LATD1; //Debug only - Toggle RD1 pin
    LATBbits.LATB8 = 1; //Debug only - Toggle B8 pin
    
    /****************************************/
    /*************  ACQUISITION  ************/
    //Reads de current from the PV panel (IPV)
    analog_currentPhotovoltaic = readAnalogChannel(0);
    v0 = (analog_currentPhotovoltaic * AD_FS) / AD16Bit_FS;
    currentPhotovoltaic = (v0 * HY15P_IN) / HY15P_VN;

    //Reads the voltage from de photovoltaic panel (VPV)
    analog_voltagePhotovoltaic = readAnalogChannel(1);
    v1 = (analog_voltagePhotovoltaic * AD_FS) / AD16Bit_FS;\
    voltagePhotovoltaic = v1 * AT_PV;
    
    // Reads the output voltage (VDC)
    analog_voltageOutput = readAnalogChannel(2);
    v2 = (analog_voltageOutput * AD_FS) / AD16Bit_FS;
    voltageOutput = v2 * AT_VDC;
    
    // Filters
    voltagePV_filtered = filter_10hz(voltagePhotovoltaic, 0);
    currentPV_filtered = filter_10hz(currentPhotovoltaic, 1);
    voltageDCBUS_filtered = filter_10hz(voltageOutput, 2);
    /*************  ACQUISITION  ************/
    /****************************************/

    /******************************************/
    /*************  DC PROTECTION  ************/
    /******** 250HZ filter + overvoltage and undervoltage protection - Vdc Min. = X; Vdc Max. = X ********/
    voltagePV_filtered250 = filter_250hz(voltagePhotovoltaic, 3);
    currentPV_filtered250 = filter_250hz(currentPhotovoltaic, 4);
    voltageDCBUS_filtered250 = filter_250hz(voltageOutput, 5);
    
    if (voltageDCBUS_filtered250 < VDC_MIN || voltageDCBUS_filtered250 > VDC_MAX)
    {
        DCBUS_threshold = 1;
        System_state = STATE_LOCKED;
    }
  
    /*************  DC PROTECTION  ************/
    /******************************************/

    /*********************************/
    /*************  MPPT  ************/
    if (mppt_counter >= 100) {

        //__delay32(10000); //Random delay
        run_mppt(voltagePV_filtered, voltagePV_filtered_ant, currentPV_filtered, currentPV_filtered_ant);
        mppt_counter = 0;
        voltagePV_filtered_ant = voltagePV_filtered;
        currentPV_filtered_ant = currentPV_filtered;
    }
    else {
        mppt_counter++;
    }
    /*************  MPPT  ************/
    /*********************************/
    LATBbits.LATB8 = 0; //Debug only - Toggle B8 pin
      
}
