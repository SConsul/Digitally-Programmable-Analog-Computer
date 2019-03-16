/**********************************************************************
* File: Main_6.c -- Solution File for Lab 6
* Devices: TMS320F28x7x
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************/

#include "Lab.h"                        // Main include file

//--- Global Variables
Uint16 DEBUG_TOGGLE = 1;                // Used for realtime mode investigation test
Uint16 SINE_ENABLE = 0;                 // Used for DAC waveform generation
Uint16 AdcBuf[ADC_BUF_LEN];             // ADC buffer allocation
Uint16 DacOffset;                       // DAC offset
Uint16 DacOutput;                       // DAC output


/**********************************************************************
* Function: main()
*
* Description: Main function for C28x workshop labs
**********************************************************************/
void main(void)
{
//--- CPU Initialization
 	InitSysCtrl();						// Initialize the CPU (FILE: SysCtrl.c)
	InitGpio();							// Initialize the shared GPIO pins (FILE: Gpio.c)
	InitXbar();							// Initialize the input, output & ePWM X-Bar (FILE: Xbar.c)
	InitPieCtrl();						// Initialize and enable the PIE (FILE: PieCtrl.c)
	InitWatchdog();						// Initialize the Watchdog Timer (FILE: WatchDog.c)

//--- Peripheral Initialization
	InitAdca();							// Initialize the ADC-A (FILE: Adc.c)
	InitDacb();                         // Initialize the DAC-B (File: Dac.c)
	InitEPwm();							// Initialize the EPwm (FILE: EPwm.c)

//--- Enable global interrupts
	asm(" CLRC INTM, DBGM");			// Enable global interrupts and realtime debug

//--- Main Loop
	AdcaRegs.ADCSOCFRC1.all = 0x0003;
	while(1)							// endless loop - wait for an interrupt
	{
		asm(" NOP");
	}


} //end of main()

interrupt void ADCA1_ISR(void)                      // PIE1.1 @ 0x000D40  ADC-A interrupt #1
{
//static Uint16 *AdcBufPtr = AdcBuf;                  // Pointer to buffer
static Uint16 iQuadratureTable = 0;                 // Quadrature table index
static volatile Uint16 GPIO34_count = 0;            // Counter for pin toggle
//static Uint16 adc_op1 = 0,adc_op2=0;
static Uint16 adc_ip0 = 0,adc_ip1 = 0;
static float v_op0,v_op1,d_op=0,x=0,sin;

    AdcaRegs.ADCSOCFRC1.all = 0x0000;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;         // Must acknowledge the PIE group

//--- Manage the ADC registers
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;          // Clear ADCINT1 flag

//--- Read the ADC result
//    *AdcBufPtr++ =  AdcaResultRegs.ADCRESULT0;          // Read the result
    adc_ip0 = AdcaResultRegs.ADCRESULT0;
    adc_ip1 = AdcaResultRegs.ADCRESULT1;

//    adc_op2 = AdcaResultRegs.ADCRESULT1;
    v_op0 = (3.3*((float)adc_ip0))/4096;
    v_op1 = (3.3*((float)adc_ip1))/4096;

    sin = (v_op1-1.5);
    x = (v_op0-1.5)/2810;

//--- Brute-force the circular buffer
//    if( AdcBufPtr == (AdcBuf + ADC_BUF_LEN) )
//    {
//        AdcBufPtr = AdcBuf;                     // Rewind the pointer to beginning
//    }

//--- Example: Toggle GPIO18 so we can read it with the ADC ***/
    if(DEBUG_TOGGLE == 1)
    {
        GpioDataRegs.GPATOGGLE.bit.GPIO18 = 1;      // Toggle the pin
    }

//--- Example: Toggle GPIO34 at a 0.5 sec rate (connected to the LED).
//             (1/50000 sec/sample)*(1 samples/int)*(x interrupts/toggle) = (0.5 sec/toggle)
//             ==> x = 25000
    if(GPIO34_count++ > 25000)                  // Toggle slowly to see the LED blink
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Toggle the pin
        GPIO34_count = 0;                       // Reset the counter
    }

//--- Write to DAC-B to create input to ADC-A0
    if(SINE_ENABLE == 1)
    {
        //sin_func = DacOffset + ((QuadratureTable[iQuadratureTable++] ^ 0x8000) >> 5);
        //DacOutput = adc_op + sin_func;
    }
    else
    {
//        DacOutput = DacOffset;
//        d_op = sin-x;
        d_op = sin-x;
        d_op = (d_op)+1.5;
//         d_op = sin(v_op);
//        DacOutput = adc_op1 + adc_op2;
        DacOutput = (Uint16)((d_op/3.3)*4096);
    }
    if(iQuadratureTable > (SINE_PTS - 1))       // Wrap the index
    {
        iQuadratureTable = 0;
    }
    DacbRegs.DACVALS.all = DacOutput;
}

/*** end of file *****************************************************/
