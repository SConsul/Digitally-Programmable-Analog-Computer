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
//! - Voltage1[10]    - Last 10 ADCRESULT0 values
//! - Voltage2[10]    - Last 10 ADCRESULT1 values
//! - ConversionCount - Current result number 0-9
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

//
// Function Prototypes
//
__interrupt void adc_isr(void);
void Adc_Config(void);

void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);
void error(void);

//
// Globals
//
Uint16 LoopCount,flag;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];

//
// Main
// 


void main(void)
{
    Uint16 sdata;  // send data
    Uint16 rdata;  // received data
    Uint16 i;
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
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example
    //
    InitAdc();  // For this example, init the ADC
    AdcOffsetSelfCal();
    spi_fifo_init();   // Initialize the SPI only
    spi_init();       // init SPI

    //
    // Step 5. User specific code, enable interrupts:
    //

    //
    // Enable ADCINT1 in PIE
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    IER |= M_INT1; 					   // Enable CPU Interrupt 1
    EINT;          					   // Enable Global interrupt INTM
    ERTM;          					   // Enable Global realtime interrupt DBGM

    LoopCount = 0;
    ConversionCount = 0;

    //
    // Configure ADC
    //
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode
    
    //
    // ADCINT1 trips after AdcResults latch
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;
    
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;  // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;  // Disable ADCINT1 Continuous mode
    
    //
    // setup EOC1 to trigger ADCINT1 to fire
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL 	= 1;
    
    AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 0;  // set SOC0 channel select to ADCINA0
    AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 1;  // set SOC1 channel select to ADCINA1
    
    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;
    
    //
    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts 
    // first then SOC1
    //
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;
    
    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;
    
    //
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //    
    AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;
    EDIS;

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA 	= 0x0080;	// Set compare A value
    EPwm1Regs.TBPRD 			= 0x3F00;	// Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE	= 0;		// count up and start

    //
    // Wait for ADC interrupt
    //
    sdata = 0x0000;

    for(;;)
    {
        //
        // Transmit data
        //
        if(flag == 1){
//            GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
            spi_xmit(sdata);
            flag = 0;
//            for(i=0;1<10;i++){}
            GpioDataRegs.GPASET.bit.GPIO19 = 1;

        }

        //
        // Wait until data is received
        //
//        while(SpiaRegs.SPIFFRX.bit.RXFFST !=1)
//                            {
//
//                            }

       // SpiaRegs.SPIFFTX.bit.TXFFST = 0;



        //
        //
        // Check against sent data
        //
        rdata = SpiaRegs.SPIRXBUF;
//        if(rdata != sdata)
//        {
//            error();
//        }

        sdata = 0xFFF6;
        LoopCount++;
    }
}

//
// adc_isr - 
//
__interrupt void
adc_isr(void)
{
    Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
    Voltage2[ConversionCount] = AdcResult.ADCRESULT1;
    
    flag = 1;
    //
    // If 20 conversions have been logged, start over
    //
    if(ConversionCount == 9)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
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

    SpiaRegs.SPIBRR =0x007F;
    SpiaRegs.SPICCR.all =0x009F;   // Relinquish SPI from Reset
    SpiaRegs.SPICCR.bit.SPICHAR = 11;

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

