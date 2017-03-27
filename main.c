#include <libpic30.h>
#include <p30f4011.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


_FOSC(CSW_FSCM_OFF & FRC_PLL16); 			// Turn off the WatchDog Timer
_FWDT(WDT_OFF);                             // Watchdog off
_FBORPOR(MCLR_DIS & PWRT_OFF);				// Disable MCLR reset pin and turn off the power-up timers


#define AD_FS 5 
#define AD16Bit_FS 65535
#define HY15P_IN 15
#define HY15P_VN 4
#define AT_PV 12 // 
#define AT_VDC 101 

#define NUM_FILTERS 3




void configure_pins();
void set_duty_cycle(float duty);
unsigned intreadAnalogChannel(int n);
void txInt(unsigned int variable);
void txFloat(float variable);
void txChar(char caracter);
unsigned int number(unsigned int y,unsigned int operator);
void print_data(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, unsigned int duty, float average_voltageOutput);
void print_header(char *msg, int count);
float filter_250hz(float input_value, int filter_number);
float filter_10hz(float input_value, int filter_number);

float value_ant1[NUM_FILTERS];
float value_filt_ant1[NUM_FILTERS];
float value_filt[NUM_FILTERS];

unsigned int x;
int d1,d2;

float f2;
char Message0[]="\n 1-Turn on System";
char MessageA[]="\n 0-Turn off System";
char Message1[] = "\n V_PV=";
char MessageB[]="\n R-Read Data";
char Message2[] = "\n I_PV=";
char Message3[] = "\n Duty=";
char Message4[] = "\n V_out=";
char Message7[] = "\n ***System ON*** \n";
char Message8[] = "\n ***System OFF*** \n";
char MessageX[] = "\nDUTY";
int n;





void set_duty_cycle(float duty)  //Delays PWM's 180
{
  PDC1 = duty * (2 * 301.5);
  PDC2 = (1 - duty) * (2 * 301.5);
}

void configure_pins()
{
        //configure Port D and B
        LATD = 0;						// output equals 0
        TRISD = 0b11111110;				// RD0 output RD1 to RD8 input
        TRISB = 0x01FF;					// Ports B are inputs
        TRISBbits.TRISB8=0; //pin B9 is configured as output

        //configure ADC
        ADPCFG = 0xFF00;				// Lowest 8 PORTB pins are analog inputs
        ADCON1 = 0x0000;				// Manually clear SAMP to end sampling, start
        ADCON1bits.FORM = 2;			// Output format - =2 means fraction
        ADCSSL = 0;						// A/D input scan select register
        ADCON2 = 0;						// Voltage reference from AVDD and AVSS
        ADCON3 = 0x0005;				// Manual Sample, ADCS=5 -> Tad = 3*Tcy
        ADCHS = 0x0000;					// Selects the input channels to be converted
        ADCON1bits.ADON = 1;			// Turn ADC ON

        //Configuration of PWM's
        PWMCON1 = 0b00110011;			// Enable channels 1 & 2 in complementary mode
        PWMCON2 = 0;					// bit 0 UDIS = 0 - Updates from duty cycle and period buffer registers are enabled
										// bit 1 OSYNC = 0 - Output overrides via the OVDCON register occur on next Tcy boundary
										// bit 2 IUE = 0 - Updates to the active PDC registers are synchronized to the PWM time base
										// bit 11-8 SEVOPS = 0000 -  PWM Special Event Trigger Output Postscale Select bits (1:1 Postscale)
        PTCON = 0;
        _PTCKPS = 0;					// prescale=1:1 (0=1:1, 1=1:4, 2=1:16, 3=1:64)
        _PTMOD = 0b10;					// Select up-down counting for centre-aligned PWM
        PTPER = 300;					// The PTPER register sets the counting period for PTMR // 50kHz in up/down counting mode
        PTMR = 0;						// Clear 15-bit PWM timer counter
        _PTEN = 1;						// Enable PWM time base

        // Porta-Serie
        //8 data bits, 1 stop bit, no parity bit
        U2BRG = 97;
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
  while (!ADCON1bits.DONE);
  return ADCBUF0;
}

void txInt(unsigned int variable)
{
  x = variable;
  if(variable >= 10000)
  {
    x=number(x,10000);
    x=number(x,1000);
    x=number(x,100);
    x=number(x,10);
    x=number(x,1);
  }
  if(variable >= 1000 && variable < 10000)
  {
    x = number(x,1000);
    x = number(x,100);
    x = number(x,10);
    x = number(x,1);
  }
  if(variable >= 100 && variable < 1000)
  {
    x = number(x,100);
    x = number(x,10);
    x = number(x,1);
  }
  if(variable >= 10 && variable < 100)
  {
    x = number(x,10);
    x = number(x,1);
  }
  if(variable < 10)
    x = number(x,1);

}

void txFloat(float variable)
{
  d1 = (int) variable;
  f2 = variable - d1;
  d2 = (f2 * 1000);
  txInt(d1);
  txChar('.');
  txInt(d2);
}

unsigned int number(unsigned int y,unsigned int operator) {
  while(U2STAbits.UTXBF);   				//waits until buffer is full 
  U2TXREG = (y/operator) + 48;
  y = (y%operator);
}

void txChar(char txChar) {
  while(U2STAbits.UTXBF);  				//waits until buffer is full
  U2TXREG = txChar;
}

void print_data(float average_voltagePhotovoltaic, float average_currentPhotovoltaic, unsigned int duty, float average_voltageOutput) {
  print_header(Message1, sizeof(Message1));
  txFloat(average_currentPhotovoltaic);
  txChar('V');
  print_header(Message2, sizeof(Message2));
  txFloat(average_voltagePhotovoltaic);
  txChar('A');
  print_header(Message3, sizeof(Message3));
  txFloat(duty/(2 * 3.015));
  txChar('%');
  print_header(Message4, sizeof(Message4));
  txFloat(average_voltageOutput);
  txChar('V');
}

void print_header(char *msg, int count)
{
    for(n = 0; n < (count -1); n++) {
         while(U2STAbits.UTXBF); //waits until buffer is full
         U2TXREG = msg[n];
    }
}

float filter_250hz(float input_value, int filter_number) {
//filtro a 200Hz primeira ordem: coeficientes obtidos em \..\3. Simulação\3. controlo eólica\filtro.m
//values(i) = (b(1)*signal(i)+b(2)*signal(i-1)-a(2)*values(i-1))/a(1);

//A1 = 1
//A2 = -5.551115123125783e-17
//B1 = 0.500000000000000
//B2 = 0.500000000000000

//value_filt[filter_number] = ( (B1*input_value + B2*value_ant1[filter_number] - A2*value_filt_ant1[filter_number]));

  value_filt[filter_number] = ( (0.5*input_value + 0.5*value_ant1[filter_number]));
  value_ant1[filter_number]	= input_value;
  value_filt_ant1[filter_number]	= value_filt[filter_number];
  return value_filt[filter_number];			
}

float filter_10hz(float input_value, int filter_number) {
//filtro a 200Hz primeira ordem: coeficientes obtidos em \..\3. Simulação\3. controlo eólica\filtro.m
//values(i) = (b(1)*signal(i)+b(2)*signal(i-1)-a(2)*values(i-1))/a(1);

//A1 = 1
//A2 = -0.939062505817492
//B1 = 0.030468747091254
//B2 = 0.030468747091254

//value_filt[filter_number] = ( (B1*input_value + B2*value_ant1[filter_number] - A2*value_filt_ant1[filter_number]));

  value_filt[filter_number] = ( (0.030468747091254*input_value + 0.030468747091254*value_ant1[filter_number] + 0.939062505817492*value_filt_ant1[filter_number]));
  value_ant1[filter_number]	= input_value;
  value_filt_ant1[filter_number]	= value_filt[filter_number];
  return value_filt[filter_number];			
}


int main()
{

    /////////Initialization//////////////////////////////

    float v0, v1, v2, analog_voltagePhotovoltaic, analog_currentPhotovoltaic, analog_voltageOutput;
    float voltagePhotovoltaic, currentPhotovoltaic, voltageOutput;

    int numberOfSamples = 100;
    float measure_currentPhotovoltaic[numberOfSamples], average_currentPhotovoltaic;
    float measure_voltagePhotovoltaic[numberOfSamples], average_voltagePhotovoltaic;
    float measure_voltageOutput[numberOfSamples], average_voltageOutput;
    int i = 0;
    int j = 0, y, system = 0;
    int flag=0;
    int aux_i = 0;
    float voltage_filtered = 0;

    float dI, dV, Iold, Vold;
    float duty = 0.5;     // Duty Initialization
    float dutyMax = 0.9;
    float dutyMin = 0.4;

    float dutyStep = 0.002;      
    float deltaV = 0.01;
    float deltaI = 0.01;
    float deltaG = 0.01;

    float auxG, auxDG, G;

    //float T_mppt=0.001;        // in seconds
    float samplingTime = 0.001;
    long  clockFrequency = 30000000;    // Hz


    configure_pins();
    set_duty_cycle(0);


     // filters' initialization
    for (aux_i = 0 ;  aux_i < NUM_FILTERS ; aux_i++)
    {
     value_ant1[aux_i] = 0;
     value_filt_ant1[aux_i] = 0;
     value_filt[aux_i] = 0;
    }

while(1) {
  
    //  { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
     __delay32(samplingTime * clockFrequency);  // argument = clock cycles
     LATBbits.LATB8=~LATBbits.LATB8; 
      // reads the current from photovoltaic panel
     analog_currentPhotovoltaic = readAnalogChannel(0);
      v0 = (analog_currentPhotovoltaic*AD_FS)/AD16Bit_FS;  // analogic value [0 4]
      currentPhotovoltaic = (v0*HY15P_IN)/HY15P_VN; 

      // reads the voltage from de photovoltaic panel
      analog_voltagePhotovoltaic = readAnalogChannel(1);
      v1 = (analog_voltagePhotovoltaic*AD_FS)/AD16Bit_FS;  // analogic value [0 4,5]
      voltagePhotovoltaic = v1*AT_PV;
      //voltagePhotovoltaic=v1*11.066;

      // reads the output voltage
      analog_voltageOutput = readAnalogChannel(2);
      v2 = (analog_voltageOutput*AD_FS)/AD16Bit_FS;  //analogic value [0 5]
      voltageOutput = v2*AT_VDC; 

      voltage_filtered = filter_250hz(voltageOutput,0);


      //voltageOutput = v2*115.2;

      // each measure contains 100 samples
      measure_currentPhotovoltaic[i] = currentPhotovoltaic;
      measure_voltagePhotovoltaic[i] = voltagePhotovoltaic;
      measure_voltageOutput[i] = voltageOutput;
      //    __delay32(samplingTime * clockFrequency);
      i++;

    if(i == numberOfSamples) {
        average_currentPhotovoltaic = 0; average_voltagePhotovoltaic = 0; average_voltageOutput = 0;
        for(i = 0;i < numberOfSamples; i++) {
            average_currentPhotovoltaic = average_currentPhotovoltaic + measure_currentPhotovoltaic[i];
            average_voltagePhotovoltaic = average_voltagePhotovoltaic + measure_voltagePhotovoltaic[i];
            average_voltageOutput = average_voltageOutput + measure_voltageOutput[i];
        }
        average_currentPhotovoltaic = average_currentPhotovoltaic/numberOfSamples;
        average_voltagePhotovoltaic = average_voltagePhotovoltaic/numberOfSamples;
        average_voltageOutput = average_voltageOutput/numberOfSamples;
        i = 0;
             //end media


			  //Sets system's Mode
      
           // if (U2STAbits.URXDA == 1)           //Checks if received a caracter
            //{
        y = U2RXREG;
        if(y == '1' ) {
            if(system == 0 ) {
                system = 1;
                print_header(Message7, sizeof(Message7));
                _LATD2 = 1;
                _LATD0 = 1;		//reset
                __delay32(0.1 * clockFrequency);
                _LATD0 = 0;
                }
            }
        if(y == '0') {
            system = 0;
            print_header(Message8, sizeof(Message8));
        }
        y='r';
        if(y == 'r' || y == 'R')
        print_data(average_voltagePhotovoltaic,voltage_filtered, PDC1,average_voltageOutput);
       //     }

//////////    MPPT INCREMENTAL CONDUCTANCE  ////////////////////////

        if(system == 0) {
            set_duty_cycle(0);
            _LATD2 = 0;
        }

        if(system == 1) {
            dI = average_currentPhotovoltaic - Iold;
            dV = average_voltagePhotovoltaic - Vold;
            if ((dV <= deltaV) && (dV >= - deltaV)) {
                if ((dI <= deltaI) && (dI >= - deltaI)) {
                    //do nothing
                }
                else {
                    if (dI > deltaI) {
                        duty -= dutyStep;
                    }
                    else {
                        duty += dutyStep;
                    }
                }
            }
            else {
                auxG = average_currentPhotovoltaic/average_voltagePhotovoltaic;
                auxDG = dI/dV;
                G = auxG+auxDG;
                if((G <= deltaG) && (G >= - deltaG)) {
                    //do nothing
                }
                else {
                    if(G > deltaG) {
                        duty -= dutyStep;
                    }
                    else {
                        duty += dutyStep;
                    }
                }
            }
            //Saturates PWM between 0,4 and 0,9
            if(duty >= dutyMax) {
                duty = dutyMax;
            }
            if(duty <= dutyMin) {
                duty = dutyMin;
            }

            Iold = average_currentPhotovoltaic;
            Vold = average_voltagePhotovoltaic;

            set_duty_cycle(duty);  // update duty
            //print_data(average_currentPhotovoltaic,average_voltagePhotovoltaic,PDC1,average_voltageOutput);

            } //end MPPT
        }
    }

    return 0;
} // end Main




