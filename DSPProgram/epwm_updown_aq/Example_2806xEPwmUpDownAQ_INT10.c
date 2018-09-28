//###########################################################################
// Description:
//! \addtogroup f2806x_example_list
//! <h1>ePWM Action Qualifier Module using up/down count (epwm_updown_aq)</h1>
//!
//! This example configures ePWM1, ePWM2, ePWM3 to produce an waveform with
//! independent modulation on EPWMxA and EPWMxB. The compare values CMPA 
//! and CMPB are modified within the ePWM's ISR. The TB counter is in up/down
//! count mode for this example. 
//! 
//! Monitor ePWM1-ePWM3 pins on an oscilloscope as described
//!
//! \b External \b Connections \n
//!  - EPWM1A is on GPIO0
//!  - EPWM1B is on GPIO1
//!  - EPWM2A is on GPIO2
//!  - EPWM2B is on GPIO3
//!  - EPWM3A is on GPIO4
//!  - EPWM3B is on GPIO5
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;   
}EPWM_INFO;

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);
void InitEPwm4Example(void);
void InitEPwm5Example(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
__interrupt void epwm4_isr(void);
__interrupt void epwm5_isr(void);
__interrupt void adc_isr1(void);
__interrupt void adc_isr2(void);
__interrupt void adc_isr3(void);
__interrupt void adc_isr4(void);
__interrupt void adc_isr5(void);
void update_compare(EPWM_INFO*);

// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;
EPWM_INFO epwm4_info;
EPWM_INFO epwm5_info;

Uint16 Current1BuffIndex, Current2BuffIndex, Current3BuffIndex, Current4BuffIndex, Voltage1BuffIndex;
Uint16 Current1[10];
Uint16 Current2[10];
Uint16 Current3[10];
Uint16 Current4[10];
Uint16 Voltage1[10];
Uint16 CurrentSetpointCount;
Uint16 VoltageSetpointCount;
const Uint16 CurrentHysteresisCount = 10;
const Uint16 VoltageHysteresisCount = 10;

#define ADC_usDELAY  1000L

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  500  // Period register
#define EPWM1_MAX_CMPA     450
#define EPWM1_MIN_CMPA      50
#define EPWM1_MAX_CMPB     450
#define EPWM1_MIN_CMPB      50

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

void main(void)
{
// Initialize Global Variables
   Current1BuffIndex = 0;
   Current2BuffIndex = 0;
   Current3BuffIndex = 0;
   Current4BuffIndex = 0;
   Voltage1BuffIndex = 0;
   CurrentSetpointCount = 1000;
   VoltageSetpointCount = 1000;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO: 
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example  

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3, ePWM4, ePWM5
// These functions are in the F2806x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
   InitEPwm4Gpio();
   InitEPwm5Gpio();
   
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();
   
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM1_INT = &epwm1_isr;
   PieVectTable.EPWM2_INT = &epwm2_isr;
   PieVectTable.EPWM3_INT = &epwm3_isr;
   PieVectTable.EPWM4_INT = &epwm4_isr;
   PieVectTable.EPWM5_INT = &epwm5_isr;
   PieVectTable.ADCINT1 = &adc_isr1;
   PieVectTable.ADCINT2 = &adc_isr2;
   PieVectTable.ADCINT3 = &adc_isr3;
   PieVectTable.ADCINT4 = &adc_isr4;
   PieVectTable.ADCINT5 = &adc_isr5;

   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// For this example, only initialize the ePWM

   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();
   InitEPwm2Example();
   InitEPwm3Example();
   InitEPwm4Example();
   InitEPwm5Example();

   // Initialize ADC start
   extern void DSP28x_usDelay(Uint32 Count);

   // *IMPORTANT*
   // The Device_cal function, which copies the ADC calibration values from TI reserved
   // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
   // Boot ROM. If the boot ROM code is bypassed during the debug process, the
   // following function MUST be called for the ADC to function according
   // to specification. The clocks to the ADC MUST be enabled before calling this
   // function.
   // See the device data manual and/or the ADC Reference
   // Manual for more information.

       EALLOW;
       SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
       (*Device_cal)();
       EDIS;

   // To powerup the ADC the ADCENCLK bit should be set first to enable
   // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
   // Before the first conversion is performed a 5ms delay must be observed
   // after power up to give all analog circuits time to power up and settle

   // Please note that for the delay function below to operate correctly the
   // CPU_RATE define statement in the F2806x_Examples.h file must
   // contain the correct CPU clock period in nanoseconds.
   EALLOW;
   AdcRegs.ADCCTL1.bit.ADCBGPWD  = 1;      // Power ADC BG
   AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
   AdcRegs.ADCCTL1.bit.ADCPWDN   = 1;      // Power ADC
   AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC
   AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG
   AdcRegs.ADCCTL1.bit.INTPULSEPOS	= 1;	// ADCINT one cycle before AdcResults latch
   AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;	// Enable non-overlap mode
   EDIS;

   DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

   EALLOW;
   AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1; //ADCCLK=SYSCLK/2
   EDIS;

   DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
   // Initialize ADC end


   // configures multiplexed pins as AIs -- start
   EALLOW;

	/* Configure ADC pins using AIO regs*/
	// This specifies which of the possible AIO pins will be Analog input pins.
	// NOTE: AIO1,3,5,7-9,11,13,15 are analog inputs in all AIOMUX1 configurations.
	// Comment out other unwanted lines.
	// 0,1 => AI is enabled. 2,3 => AI is disabled  **
    GpioCtrlRegs.AIOMUX1.bit.AIO2 = 0;    // Configure AIO2 for A2 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO4 = 0;    // Configure AIO4 for A4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO6 = 2;    // Configure AIO6 for A6 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO10 = 2;   // Configure AIO10 for B2 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO12 = 2;   // Configure AIO12 for B4 (analog input) operation
    GpioCtrlRegs.AIOMUX1.bit.AIO14 = 2;   // Configure AIO14 for B6 (analog input) operation

    EDIS;
    // configures multiplexed pins as AIs -- start

   //AdcOffsetSelfCal();
   //AdcChanConfig();
   
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
   EDIS;
   
// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWMINT1-5 in PIE:
   IER |= M_INT3;
// Enable CPU INT10 which is connected to ADCINT1-5 in PIE
   IER |= M_INT10; 						// Enable CPU Interrupt 10
//   IER |= M_INT1; 						// Enable CPU Interrupt 1 **

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-5
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx5 = 1;

// Enable ADC INTn in the PIE: Group 10 interrupt 1-5
// INT10.x not working ISRs corresponding to ADCINTx are just not called !!
   PieCtrlRegs.PIEIER10.bit.INTx1 = 1; //Group1 Interrupt 10 for ADCINT1
   PieCtrlRegs.PIEIER10.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER10.bit.INTx3 = 1;
   PieCtrlRegs.PIEIER10.bit.INTx4 = 1;
   PieCtrlRegs.PIEIER10.bit.INTx5 = 1;

// prog does not work if the two lines below are commented !!
//   PieCtrlRegs.PIEIER1.bit.INTx1 = 1; //Group1 Interrupt 1 for ADCINT1 **
//   PieCtrlRegs.PIEIER1.bit.INTx2 = 1; //Group1 Interrupt 1 for ADCINT2 **


// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Configure ADC
//   AdcChanConfig();

	EALLOW;

       AdcRegs.ADCSOC0CTL.bit.CHSEL= 0;   		// SOC0 ==> ADCINA0
       AdcRegs.ADCSOC1CTL.bit.CHSEL= 1;   		// SOC1 ==> ADCINA1
/*       AdcRegs.ADCSOC2CTL.bit.CHSEL= 3;   		// SOC0 ==> ADCINA3
       AdcRegs.ADCSOC3CTL.bit.CHSEL= 5;   		// SOC1 ==> ADCINA5
       AdcRegs.ADCSOC4CTL.bit.CHSEL= 7;   		// SOC0 ==> ADCINA7 */
       AdcRegs.ADCSOC2CTL.bit.CHSEL= 2;   		// SOC0 ==> ADCINA2   // now CH2 & 4 working!
       AdcRegs.ADCSOC3CTL.bit.CHSEL= 3;   		// SOC1 ==> ADCINA3
       AdcRegs.ADCSOC4CTL.bit.CHSEL= 4;   		// SOC0 ==> ADCINA4

	   AdcRegs.INTSEL1N2.bit.INT1E     = 1;	// Enabled ADCINT1
	   AdcRegs.INTSEL1N2.bit.INT2E     = 1;	// Enabled ADCINT2
	   AdcRegs.INTSEL3N4.bit.INT3E     = 1;	// Enabled ADCINT3
	   AdcRegs.INTSEL3N4.bit.INT4E     = 1;	// Enabled ADCINT4
	   AdcRegs.INTSEL5N6.bit.INT5E     = 1;	// Enabled ADCINT5

	   AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;	// Disable ADCINT1 Continuous mode
	   AdcRegs.INTSEL1N2.bit.INT2CONT  = 0;	// Disable ADCINT2 Continuous mode
	   AdcRegs.INTSEL3N4.bit.INT3CONT  = 0;	// Disable ADCINT3 Continuous mode
	   AdcRegs.INTSEL3N4.bit.INT4CONT  = 0;	// Disable ADCINT4 Continuous mode
	   AdcRegs.INTSEL5N6.bit.INT5CONT  = 0;	// Disable ADCINT5 Continuous mode

	   AdcRegs.INTSEL1N2.bit.INT1SEL 	= 0;    // setup EOC0 to trigger ADCINT1 to fire
	   AdcRegs.INTSEL1N2.bit.INT2SEL 	= 1;    // setup EOC1 to trigger ADCINT2 to fire
	   AdcRegs.INTSEL3N4.bit.INT3SEL 	= 2;    // setup EOC2 to trigger ADCINT3 to fire
	   AdcRegs.INTSEL3N4.bit.INT4SEL 	= 3;    // setup EOC3 to trigger ADCINT4 to fire
	   AdcRegs.INTSEL5N6.bit.INT5SEL 	= 4;    // setup EOC4 to trigger ADCINT5 to fire

	   AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;    // set SOC0 start trigger on EPWM1A
	   AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 7;    // set SOC1 start trigger on EPWM2A
	   AdcRegs.ADCSOC2CTL.bit.TRIGSEL 	= 9;    // set SOC2 start trigger on EPWM3A
	   AdcRegs.ADCSOC3CTL.bit.TRIGSEL 	= 11;    // set SOC3 start trigger on EPWM4A
	   AdcRegs.ADCSOC4CTL.bit.TRIGSEL 	= 13;    //set SOC4 start trigger on EPWM5A

	   AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;	// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC2CTL.bit.ACQPS 	= 6;	// set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC3CTL.bit.ACQPS 	= 6;	// set SOC3 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC4CTL.bit.ACQPS 	= 6;	// set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

	EDIS;

    DELAY_US(ADC_usDELAY);                  // Delay before converting ADC channels

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
      __asm("          NOP");
   }

} 

__interrupt void epwm1_isr(void)
{
   // Update the CMPA and CMPB values
   //update_compare(&epwm1_info);
   
   // Clear INT flag for this timer
   EPwm1Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

__interrupt void epwm2_isr(void)
{

   // Update the CMPA and CMPB values
   //update_compare(&epwm2_info);

   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm3_isr(void)
{
	//AdcRegs.ADCSOCFRC1.bit.SOC2=1; // force conversion on SOC2
   // Update the CMPA and CMPB values
   //update_compare(&epwm3_info);

   // Clear INT flag for this timer

	EPwm3Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm4_isr(void)
{

   // Update the CMPA and CMPB values
   //update_compare(&epwm3_info);

   // Clear INT flag for this timer
   EPwm4Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm5_isr(void)
{
	//AdcRegs.ADCSOCFRC1.bit.SOC4=1; // force conversion on SOC4
   // Update the CMPA and CMPB values
   //update_compare(&epwm3_info);

   // Clear INT flag for this timer
   EPwm5Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void  adc_isr1(void)
{
  Current1[Current1BuffIndex] = AdcResult.ADCRESULT0;
 // Current3[Current3BuffIndex] = AdcResult.ADCRESULT2;
//  Voltage1[Voltage1BuffIndex] = AdcResult.ADCRESULT4;

  // check and update EPWM1 levels .. switch states
  if (Current1[Current1BuffIndex]>(CurrentSetpointCount+CurrentHysteresisCount))
  {
	  EPwm1Regs.CMPA.half.CMPA = 2001;
	  EPwm1Regs.CMPB = 2001;
  }
  if (Current1[Current1BuffIndex]<(CurrentSetpointCount-CurrentHysteresisCount))
  {
	  EPwm1Regs.CMPA.half.CMPA = 0;
	  EPwm1Regs.CMPB = 0;
  }

/*
  // check and update EPWM3 levels .. switch states
  if ( Current3[Current3BuffIndex]>(CurrentSetpointCount+CurrentHysteresisCount))
  {
	  EPwm3Regs.CMPA.half.CMPA = 2001;
	  EPwm3Regs.CMPB = 2001;
  }
  if ( Current3[Current3BuffIndex]<(CurrentSetpointCount-CurrentHysteresisCount))
  {
	  EPwm3Regs.CMPA.half.CMPA = 0;
	  EPwm3Regs.CMPB = 0;
  }

  // check and update EPWM5 levels .. switch states
  if ( Voltage1[Voltage1BuffIndex]>(CurrentSetpointCount+CurrentHysteresisCount))
  {
	  EPwm5Regs.CMPA.half.CMPA = 2001;
	  EPwm5Regs.CMPB = 2001;
  }
  if ( Voltage1[Voltage1BuffIndex]<(CurrentSetpointCount-CurrentHysteresisCount))
  {
	  EPwm5Regs.CMPA.half.CMPA = 0;
	  EPwm5Regs.CMPB = 0;
  }
*/


  // If 20 conversions have been logged, start over
  if(Current1BuffIndex == 9)
  {
     Current1BuffIndex = 0;
  }
  else Current1BuffIndex++;

  /*
  // If 20 conversions have been logged, start over
  if(Current3BuffIndex == 9)
  {
     Current3BuffIndex = 0;
  }
  else Current3BuffIndex++;

  // If 20 conversions have been logged, start over
  if(Voltage1BuffIndex == 9)
  {
	  Voltage1BuffIndex = 0;
  }
  else Voltage1BuffIndex++;
*/

  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE
  //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}

__interrupt void  adc_isr2(void)
{
	Current2[Current2BuffIndex] = AdcResult.ADCRESULT1;
//	Current4[Current4BuffIndex] = AdcResult.ADCRESULT3;

	 // check and update EPWM2 levels .. switch states
	  if (Current2[Current2BuffIndex]>(CurrentSetpointCount+CurrentHysteresisCount))
	  {
		  EPwm2Regs.CMPA.half.CMPA = 2001;
		  EPwm2Regs.CMPB = 2001;
	  }
	  if (Current2[Current2BuffIndex]<(CurrentSetpointCount-CurrentHysteresisCount))
	  {
		  EPwm2Regs.CMPA.half.CMPA = 0;
		  EPwm2Regs.CMPB = 0;
	  }

/*
	  // check and update EPWM4 levels .. switch states
	  if ( Current4[Current4BuffIndex]>(CurrentSetpointCount+CurrentHysteresisCount))
	  {
		  EPwm4Regs.CMPA.half.CMPA = 2001;
		  EPwm4Regs.CMPB = 2001;
	  }
	  if ( Current4[Current4BuffIndex]<(CurrentSetpointCount-CurrentHysteresisCount))
	  {
		  EPwm4Regs.CMPA.half.CMPA = 0;
		  EPwm4Regs.CMPB = 0;
	  }
*/

  // If 20 conversions have been logged, start over
  if(Current2BuffIndex == 9)
  {
     Current2BuffIndex = 0;
  }
  else Current2BuffIndex++;

/*
  // If 20 conversions have been logged, start over
  if(Current4BuffIndex == 9)
  {
     Current4BuffIndex = 0;
  }
  else Current4BuffIndex++;
*/

  AdcRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;		//Clear ADCINT2 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE
  //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}


__interrupt void  adc_isr3(void)  // never called why?
{

	Current3[Current3BuffIndex] = AdcResult.ADCRESULT2;

  // If 20 conversions have been logged, start over
  if(Current3BuffIndex == 9)
  {
     Current3BuffIndex = 0;
  }
  else Current3BuffIndex++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT3 = 1;		//Clear ADCINT3 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE

  return;
}




__interrupt void  adc_isr4(void) // never called why?
{

	Current4[Current4BuffIndex] = AdcResult.ADCRESULT3;

  // If 20 conversions have been logged, start over
  if(Current4BuffIndex == 9)
  {
     Current4BuffIndex = 0;
  }
  else Current4BuffIndex++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT4 = 1;		//Clear ADCINT4 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE

  return;
}


__interrupt void  adc_isr5(void) // never called why?
{

	Voltage1[Voltage1BuffIndex] = AdcResult.ADCRESULT4;

  // If 20 conversions have been logged, start over
  if(Voltage1BuffIndex == 9)
  {
	  Voltage1BuffIndex = 0;
  }
  else Voltage1BuffIndex++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT5 = 1;		//Clear ADCINT2 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE

  return;
}


void InitEPwm1Example()
{

   // Setup TBCLK
   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;           // Set timer period 801 TBCLKs
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter


   EPwm1Regs.TBPHS.half.TBPHS = 0;           // Phase is 0
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;        // Sync Downstream module
   
   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = 250; //EPWM1_MIN_CMPA;     // Set compare A value
   EPwm1Regs.CMPB = 250; //EPWM1_MIN_CMPB; 	  // Set Compare B value
   
   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up


   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
   EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;            // Generate INT on 3rd event

   EPwm1Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
//   EPwm1Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
   EPwm1Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from Timebase counter equal to period TBCTR=TBPRD
   EPwm1Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are 
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA & 
   epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
   epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm1_info.EPwmRegHandle = &EPwm1Regs;          // Set the pointer to the ePWM module
   epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
   epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;       
   epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;    
   epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;       
   
}

void InitEPwm2Example()
{

   
   // Setup TBCLK
   EPwm2Regs.TBPRD = EPWM1_TIMER_TBPRD;           // Set timer period 801 TBCLKs

   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter
   
   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = 250; //EPWM2_MIN_CMPA;     // Set compare A value
   EPwm2Regs.CMPB = 250; //EPWM2_MIN_CMPB;               // Set Compare B value
   
   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up

   EPwm2Regs.TBPHS.half.TBPHS = 200;           // Phase is 200
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
   EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        //Sync Flow Through

   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz
   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // Set actions
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
   EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event   

   EPwm2Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
//   EPwm2Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
   EPwm2Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from Timebase counter equal to period TBCTR=TBPRD
   EPwm2Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are 
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA & 
   epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP;   // increasing CMPB
   epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
   epwm2_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
   epwm2_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   epwm2_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
   epwm2_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
   
}

void InitEPwm3Example(void)
{

   
   // Setup TBCLK
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm3Regs.TBPRD = EPWM1_TIMER_TBPRD;          // Set timer period

   EPwm3Regs.TBCTR = 0x0000;                     // Clear counter
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz

   EPwm3Regs.TBPHS.half.TBPHS = 400;           // Phase is 400
   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
   EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        //Sync Flow Through

   // Setup shadow register load on ZERO
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

  // Set Compare values
   EPwm3Regs.CMPA.half.CMPA = 250; //EPWM3_MIN_CMPA;    // Set compare A value
   EPwm3Regs.CMPB = 250; //EPWM3_MAX_CMPB;              // Set Compare B value
   
   // Set Actions
   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
   EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event   

   EPwm3Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
//   EPwm2Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
   EPwm3Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from Timebase counter equal to period TBCTR=TBPRD
   EPwm3Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are 
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA & 
   epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
   epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the ePWM module
   epwm3_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
   epwm3_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   epwm3_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
   epwm3_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
   
}


void InitEPwm4Example(void)
{


   // Setup TBCLK
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm4Regs.TBPRD = EPWM1_TIMER_TBPRD;          // Set timer period

   EPwm4Regs.TBCTR = 0x0000;                     // Clear counter
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz

   EPwm4Regs.TBPHS.half.TBPHS = 600;           // Phase is 600
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
   EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
   EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        //Sync Flow Through



   // Setup shadow register load on ZERO
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
   EPwm4Regs.CMPA.half.CMPA = 250; //EPwm4_MIN_CMPA;    // Set compare A value
   EPwm4Regs.CMPB = 250; //EPwm4_MAX_CMPB;              // Set Compare B value

   // Set Actions
   EPwm4Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm4Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
   EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Interrupt where we will change the Compare Values
   EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm4Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm4Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   EPwm4Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
//   EPwm2Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
   EPwm4Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from Timebase counter equal to period TBCTR=TBPRD
   EPwm4Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event

   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers

   epwm4_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
   epwm4_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
   epwm4_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm4_info.EPwmRegHandle = &EPwm4Regs;          // Set the pointer to the ePWM module
   epwm4_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
   epwm4_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   epwm4_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
   epwm4_info.EPwmMinCMPB = EPWM1_MIN_CMPB;

}

void InitEPwm5Example(void)
{


	   // Setup TBCLK
	   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
	   EPwm5Regs.TBPRD = EPWM1_TIMER_TBPRD;          // Set timer period

	   EPwm5Regs.TBCTR = 0x0000;                     // Clear counter
	   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
	   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz

	   EPwm5Regs.TBPHS.half.TBPHS = 200;           // Phase is 0
	   EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
	   EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;
	   EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        //Sync Flow Through


	   // Setup shadow register load on ZERO
	   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	  // Set Compare values
	   EPwm5Regs.CMPA.half.CMPA = 250; //EPwm5_MIN_CMPA;    // Set compare A value
	   EPwm5Regs.CMPB = 250; //EPwm5_MAX_CMPB;              // Set Compare B value

	   // Set Actions
	   EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
	   EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

	   EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	   EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

	   // Interrupt where we will change the Compare Values
	   EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	   EPwm5Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm5Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

	   EPwm5Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
	//   EPwm2Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
	   EPwm5Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from Timebase counter equal to period TBCTR=TBPRD
	   EPwm5Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event

	   // Information this example uses to keep track
	   // of the direction the CMPA/CMPB values are
	   // moving, the min and max allowed values and
	   // a pointer to the correct ePWM registers

	   epwm5_info.EPwm_CMPA_Direction = EPWM_CMP_UP;   // Start by increasing CMPA &
	   epwm5_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
	   epwm5_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	   epwm5_info.EPwmRegHandle = &EPwm5Regs;          // Set the pointer to the ePWM module
	   epwm5_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;        // Setup min/max CMPA/CMPB values
	   epwm5_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
	   epwm5_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
	   epwm5_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}


void update_compare(EPWM_INFO *epwm_info)
{
   

   // Every 10'th interrupt, change the CMPA/CMPB values
   if(epwm_info->EPwmTimerIntCount == 10)
   {
       epwm_info->EPwmTimerIntCount = 0;
       
       // If we were increasing CMPA, check to see if 
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
	   if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }
	   
	   // If we were decreasing CMPA, check to see if 
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
	   else
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }
	
	   // If we were increasing CMPB, check to see if 
       // we reached the max value.  If not, increase CMPB
       // else, change directions and decrease CMPB
	   if(epwm_info->EPwm_CMPB_Direction == EPWM_CMP_UP)
	   {
	       if(epwm_info->EPwmRegHandle->CMPB < epwm_info->EPwmMaxCMPB)
	       {
	          epwm_info->EPwmRegHandle->CMPB++;
	       }
	       else
	       {
	          epwm_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN;
	          epwm_info->EPwmRegHandle->CMPB--;
	       }
	   }

	   // If we were decreasing CMPB, check to see if 
       // we reached the min value.  If not, decrease CMPB
       // else, change directions and increase CMPB

	   else
	   {
	       if(epwm_info->EPwmRegHandle->CMPB == epwm_info->EPwmMinCMPB)
	       {
	          epwm_info->EPwm_CMPB_Direction = EPWM_CMP_UP;
	          epwm_info->EPwmRegHandle->CMPB++;
	       }
	       else
	       {
	          epwm_info->EPwmRegHandle->CMPB--;
	       }
	   }
   }
   else 
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}

//===========================================================================
// No more.
//===========================================================================
