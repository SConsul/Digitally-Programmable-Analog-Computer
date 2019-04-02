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

__interrupt void spiTxFifoIsr(void);
__interrupt void spiRxFifoIsr(void);
void delay_loop(void);
void spi_fifo_init(void);
void error();

void Adc_Config(void);

//
// Globals
//
Uint16 IdleCount;
Uint16 ConversionCount;
Uint16 x[8],x_dot[8];

Uint16 sdata[2];     // Send data buffer
Uint16 rdata[2];     // Receive data buffer

//
// Keep track of where we are in the data stream to check received data
//
Uint16 rdata_point;



//
// Main
// 
void main(void)
{
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
    //
    // Setup only the GP I/O only for SPI-A functionality
    //
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
    PieVectTable.SPIRXINTA = &spiRxFifoIsr;
    PieVectTable.SPITXINTA = &spiTxFifoIsr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize all the Device Peripherals:
    // This function is found in F2806x_InitPeripherals.c
    // InitPeripherals(); // Not required for this example
    //
    InitAdc();  // For this example, init the ADC
    AdcOffsetSelfCal();
    spi_fifo_init();   // Initialize the SPI only

    //
    // Step 5. User specific code, enable interrupts:
    //

    //
    // Enable ADCINT1 in PIE
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx1=1;     // Enable PIE Group 6, INT 1
    PieCtrlRegs.PIEIER6.bit.INTx2=1;     // Enable PIE Group 6, INT 2
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
    IER=0x20;                            // Enable CPU INT6
    IER |= M_INT1; 					   // Enable CPU Interrupt 1
    EINT;          					   // Enable Global interrupt INTM
    ERTM;          					   // Enable Global realtime interrupt DBGM

    IdleCount = 0;
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
    // setup EOC7 (last EOC) to trigger ADCINT1 to fire
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL 	= 7;
    
    // ________________________________________________________________
    // NEED TO CONFIRM WHAT CHANNELS WE ARE USING FOR THE 8 CONVERSIONS
    // ----------------------------------------------------------------
    AdcRegs.ADCSOC0CTL.bit.CHSEL 	= 0;  // set SOC0 channel select to ADCINA4
    AdcRegs.ADCSOC1CTL.bit.CHSEL 	= 1;  // set SOC1 channel select to ADCINA2
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;  // set SOC0 channel select to ADCINA4
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 4;  // set SOC1 channel select to ADCINA2
    AdcRegs.ADCSOC4CTL.bit.CHSEL    = 8;  // set SOC0 channel select to ADCINA4
    AdcRegs.ADCSOC5CTL.bit.CHSEL    = 9;  // set SOC1 channel select to ADCINA2
    AdcRegs.ADCSOC6CTL.bit.CHSEL    = 10;  // set SOC0 channel select to ADCINA4
    AdcRegs.ADCSOC7CTL.bit.CHSEL    = 12;  // set SOC1 channel select to ADCINA2
    

    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    //
    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts 
    // first then SOC1
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 5;

    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    //
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC6CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC7CTL.bit.ACQPS    = 6;
    EDIS;

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //

//    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;   // Set timer phase

    EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC on PRD event
    EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event
//    EPwm1Regs.CMPA.half.CMPA 	= 0x0010;	// Set compare A value
    EPwm1Regs.TBCTR = 0x0000;               // Clear timer counter
    EPwm1Regs.TBPRD = 1999;                 // Set timer period
    EPwm1Regs.TBCTL.bit.CTRMODE	= 0;		// count up and start

    SpiaRegs.SPITXBUF='A';     // first tx to initialize interrupt
    //
    // Keep updating send data buffer  not exactly correct
    //
    i=0;
    while(1){
        sdata[i] = x[i];
//        i++;
        if(i==8) i=0;
//        IdleCount++;
    }
}

//
// error -
//
void
error(void)
{
    __asm("     ESTOP0");  //Test failed!! Stop!
    for (;;);
}

//
// spi_fifo_init - Initialize SPI FIFO registers
//
void
spi_fifo_init()
{
    SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI

    SpiaRegs.SPICCR.all=0x001F;      // 16-bit character, Loopback mode
    SpiaRegs.SPICTL.all=0x0017; // Interrupt enabled, Master/Slave XMIT enabled
    SpiaRegs.SPISTS.all=0x0000;
    SpiaRegs.SPIBRR=0x0063;           // Baud rate
    SpiaRegs.SPIFFTX.all=0xC022;      // Enable FIFO's, set TX FIFO level to 4
    SpiaRegs.SPIFFRX.all=0x0022;      // Set RX FIFO level to 4
    SpiaRegs.SPIFFCT.all=0x00;
    SpiaRegs.SPIPRI.all=0x0010;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}

//
// adc_isr - 
// loop time should be smaller than the total sampling time for all 8 conversions
//
__interrupt void
adc_isr(void)
{
    // New values of state variables
    x[0] = AdcResult.ADCRESULT0;
    x[1] = AdcResult.ADCRESULT1;
    x[2] = AdcResult.ADCRESULT2;
    x[3] = AdcResult.ADCRESULT3;
    x[4] = AdcResult.ADCRESULT4;
    x[5] = AdcResult.ADCRESULT5;
    x[6] = AdcResult.ADCRESULT6;
    x[7] = AdcResult.ADCRESULT7;
    
    // Here itself compute differential and send it to DAC through SPI
    x_dot[0] = x[0];
    x_dot[1] = x[1];
    x_dot[2] = x[2];
    x_dot[3] = x[3];
    x_dot[4] = x[4];
    x_dot[5] = x[5];
    x_dot[6] = x[6];
    x_dot[7] = x[7];


    ConversionCount++;

    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    return;
}


//
// spiTxFifoIsr -
//
__interrupt void
spiTxFifoIsr(void)
{
    Uint16 i;
    for(i=0;i<2;i++)
    {
       SpiaRegs.SPITXBUF=0x0001;      // Send data
    }

//    for(i=0;i<2;i++)                    // Increment data for next cycle
//    {
//       sdata[i] = sdata[i] + 1;
//    }

    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ACK
}

//
// spiRxFifoIsr -
//
__interrupt void
spiRxFifoIsr(void)
{
    Uint16 i;
    for(i=0;i<2;i++)
    {
        rdata[i]=SpiaRegs.SPIRXBUF;     // Read data
    }
    for(i=0;i<2;i++)                    // Check received data
    {
        if(rdata[i] != x[i]) error();
    }
    rdata_point++;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

//
// End of File
//

