//###########################################################################
//
// FILE:   Example_2806xAdcSoc.c
//
// TITLE:  ADC Start of Conversion Example
//
//! \addtogroup f2806x_example_list
//! <h1> ADC Start of Conversion (adc_soc)</h1>
//!
//! This ADC example uses ePWM1 to generate a periodic ADC SOC - ADCINT1.
//! Two channels are converted, ADCINA4 and ADCINA2.
//!
//! \b Watch \b Variables \n
//! - x[10]    - Last 10 ADCRESULT0 values
//! - x[10]    - Last 10 ADCRESULT1 values
//! - Ch_sel - Current result number 0-9
//! - LoopCount       - Idle loop counter
//
//###########################################################################
// $TI Release: F2806x Support Library v2.04.00.00 $
// $Release Date: Thu Oct 18 15:47:20 CDT 2018 $
// $Copyright:
// Copyright (C) 2009-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "IQmathLib.h"
//#include "IQmathCPP.h"
#define offset 1.5
// Function Prototypes
//
__interrupt void adc_isr(void);
//void Adc_Config(void);

void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);
void error(void);

//
// Globals
//
Uint16 LoopCount,flag;
Uint16 Ch_sel;
Uint16 x[8],xdot[8], ff[4],i;
void dac_xmit(Uint16 a);
float cnvt_to_float(Uint16 a,float b);
float cnvt_to_Uint(float a,float b);
//
// Main
//


void main(void)
{
//    static Uint16 tempout = 0, tempxdot=0;
    static float tempff[4], tempx[8], temp_xdot[8];
    double j=0;
//    Uint16 sdata;  // send data
//    Uint16 rdata;  // received data
//    Uint16 i;
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example
    InitSpiaGpio();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT1 = &adc_isr;
   // PieVectTable.ADCINT1 = &adc_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example
    //
    InitAdc();  // For this example, init the ADC
    AdcOffsetSelfCal();

    //
    // Step 5. User specific code, enable interrupts:
    //

    //
    // Enable ADCINT1 in PIE
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    IER |= M_INT1;                     // Enable CPU Interrupt 1
    EINT;                              // Enable Global interrupt INTM
    ERTM;                              // Enable Global realtime interrupt DBGM

    LoopCount = 0;
    Ch_sel = 0;

    //
    // Configure ADC
    //
    EALLOW;

    spi_fifo_init();   // Initialize the SPI only
    spi_init();       // init SPI

    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode

    //
    // ADCINT1 trips after AdcResults latch
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcRegs.INTSEL1N2.bit.INT1E     = 1;  // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;  // Disable ADCINT1 Continuous mode

    //
    // setup EOC1 to trigger ADCINT1 to fire
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 8;

//    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 2;  // var1
//    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 4;  // var1
//    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 12;  // ff0
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 12;  // ff1
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 2;  // var1
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 4;  // var2
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 5;  // var3
    AdcRegs.ADCSOC4CTL.bit.CHSEL    = 6;  // var4
    AdcRegs.ADCSOC6CTL.bit.CHSEL    = 1;  // var6
    AdcRegs.ADCSOC7CTL.bit.CHSEL    = 8;  // var7
    AdcRegs.ADCSOC8CTL.bit.CHSEL    = 7;  // var8


    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 5;

    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC6CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC7CTL.bit.ACQPS    = 10;
    AdcRegs.ADCSOC8CTL.bit.ACQPS    = 10;

    // configure 39
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;


    //
        GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;
           GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;

    EDIS;

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;        // Select SOC from CMPA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;        // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA    = 0x0080;   // Set compare A value
    EPwm1Regs.TBPRD             = 600;  // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;        // count up and start

    //
    // Wait for ADC interrupt
    //
//    sdata = 0xFFF6;


    SpiaRegs.SPITXBUF = 0x8008;
    SpiaRegs.SPITXBUF = 0x900F;


    while(1){

        if(flag==1){// Included Files

            flag = 0;// Included Files

            tempx[0] = cnvt_to_float(x[0],offset);
            tempx[1] = cnvt_to_float(x[1],offset);
//            tempx[3] = cnvt_to_float(x[3],offset);

            tempff[1] = cnvt_to_float(ff[1],offset);

             if(j<1000){
                  temp_xdot[0] = -tempx[0] + 1;
                  temp_xdot[1] = -tempx[1];
                  j=j+1;
            }
            else{

        //        temp_dx0 = 5*(tempx-(tempx*tempx*tempx/3)-tempx1);
        //        temp_xdot[1] = 16*(1-(tempx[0]*tempx[0]))*tempx[1] - (tempx[0]);
        //        temp_xdot[0]=tempx[1];
        //        temp_dx0 = tempff-(tempx*tempx*tempx)+offset;
//                temp_xdot[3]=tempff[1];
                temp_xdot[0] = tempx[1];
                temp_xdot[1] = (- tempx[0] + ((tempx[0]*tempx[0]*tempx[0])/6) - ((tempx[0]*tempx[0]*tempx[0]*tempx[0]*tempx[0])/120)) - 0.1*tempx[1];
//                temp_xdot[1] =
            }
             xdot[1] = cnvt_to_Uint(temp_xdot[1],offset);
             xdot[0] = cnvt_to_Uint(temp_xdot[0],offset);
             dac_xmit((0x0FFF & xdot[0])|0x0000);
             dac_xmit((0x0FFF & xdot[1])|0x1000);
             dac_xmit((0x0FFF & ff[1])|0x2000);
             dac_xmit((0x0FFF & ff[1])|0x3000);
             dac_xmit((0x0FFF & ff[1])|0x5000);
             dac_xmit((0x0FFF & ff[1])|0x6000);
             dac_xmit((0x0FFF & ff[1])|0x7000);
        }
    }
}

float cnvt_to_float(Uint16 a,float b)
{
    float temp;
    temp = 3.3*(((float)a)/4096);
    temp = temp-b;
    return temp;
}

float cnvt_to_Uint(float a,float b)
{
    Uint16 temp;
    temp = (Uint16)(((a+b)*4096)/3.3);
    return temp;
}

void dac_xmit(Uint16 a)
{
    GpioDataRegs.GPASET.bit.GPIO25 = 1;
    SpiaRegs.SPITXBUF= a;
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
    for(i=0;i<20;i++){}
}

//
// adc_isr -
//
__interrupt void
adc_isr(void)
{


    flag = 1;
    //
    // If 20 conversions have been logged, start over
    //

    ff[1] = AdcResult.ADCRESULT0;
    x[0] = AdcResult.ADCRESULT1;
    x[1] = AdcResult.ADCRESULT2;
    x[2] = AdcResult.ADCRESULT3;
    x[3] = AdcResult.ADCRESULT4;
    x[5] = AdcResult.ADCRESULT6;
    x[6] = AdcResult.ADCRESULT7;
    x[7] = AdcResult.ADCRESULT8;

    if (x[0]<2048){
        //led high
        GpioDataRegs.GPBSET.bit.GPIO39 = 1;
    }
    else
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;
    }

    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}



void
error(void)
{
    __asm("     ESTOP0");                       // Test failed!! Stop!
    for (;;);
}

//
// spi_init -
//
void
spi_init()
{
    SpiaRegs.SPICCR.all =0x000F;  // Reset on, rising edge, 16-bit char bits
    //
    // Enable master mode, normal phase, enable talk, and SPI int disabled.
    //
    SpiaRegs.SPICTL.all =0x0006;

    SpiaRegs.SPIBRR =0x0002;
    SpiaRegs.SPICCR.all =0x009F;   // Relinquish SPI from Reset
    SpiaRegs.SPICCR.bit.SPICHAR = 15;

    SpiaRegs.SPIPRI.bit.FREE = 1;  // Set so breakpoints don't disturb xmission

}

//
// spi_xmit -
//
void
spi_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF=a;
}

//
// spi_fifo_init -
//
void
spi_fifo_init(void)
{
    //
    // Initialize SPI FIFO registers
    //
    SpiaRegs.SPIFFTX.all=0xE040;
    SpiaRegs.SPIFFRX.all=0x2044;
    SpiaRegs.SPIFFCT.all=0x0;
}

//
// End of File
//
