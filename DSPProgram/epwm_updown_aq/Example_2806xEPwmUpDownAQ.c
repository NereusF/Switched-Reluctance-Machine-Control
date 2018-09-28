//###########################################################################
// Description:
//! \addtogroup f2806x_example_list
//! <h1>ePWM Action Qualifier Module using up/down count (epwm_updown_aq)</h1>
//! epwm_updown_aq is used as the base
//!
//!	Updated : 26-Apr-2016 -- changes have been tested - ramp, PI, rising edge
//!							speed calc, angle calc, angle based commut. >300prm
//!
//!
//! ePWM1, ePWM2, ePWM3, ePWM3 wth independent modulation on EPWMxA and EPWMxB
//! The compare values CMPA and CMPB are modified within the adc1_isr to turn
//! switches as per the set and actual values
//!
//! Soft chopping is implemented
//!
//! Current Setpoint can be by internal / watch-window setpoint or through pot
//! connected to the ASCINA5 input
//! 
//! Offset correction using correction counts to trim to zero.
//! Adjust Current1OffsetCount to get zero in Current1Amps
//!
//! Position, Speed, Commutation Logic is in the cpu_timer0_isr
//!
//! MainMode =0 => Motoring, operation on +ve inductance slope and regenrative
//!					braking on -ve inductance slope
//! MainMode =1 => Generation, NO operation on +ve inductance slope ONLY regenrative
//!					braking on -ve inductance slope
//!
//! Load power calc based on the chopper sw on/off times V^2/R
//!
//! Corrected RPM calibrated wrt Siemens Drive RPM readings
//!
//!
//! \b External \b Connections \n
//!  - EPWM1A is on GPIO0   A1
//!  - EPWM1B is on GPIO1   A2
//!  - EPWM2A is on GPIO2   B1
//!  - EPWM2B is on GPIO3	B2
//!  - EPWM3A is on GPIO4	C1
//!  - EPWM3B is on GPIO5	C2
//!  - EPWM4A is on GPIO6	D1
//!  - EPWM4B is on GPIO7	D2
//!  - EPWM5A is on GPIO8	Chopper Load
//!  - EPWM5B is on GPIO9
//!  - EPWM6A is on GPIO10	AO for speed (TBD)
//!
//!  - ADCINA0 is Ia
//!  - ADCINA1 is Ib
//!  - ADCINA2 is Ic
//!  - ADCINA3 is Id
//!  - ADCINA4 is Vgen
//!  - ADCINA5 is Speed / Current Setpoint ( for test )
//!
//!  - GPIO32 digital input for YELLOW SENSOR
//!  - GPIO33 digital input for GREEN SENSOR
//!
//!	-	ECAP1 is on GPIO24  input for YELLOW SENSOR ( not used due to bounce )
//!	-	ECAP2 is on GPIO25  input for GREEN SENSOR
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
extern void DSP28x_usDelay(Uint32 Count);

const Uint16 MainMode=1;   // 0=> motoring, 1=> regerative braking
// With MainMode=0=Motoring if actual speed < reference speed
// windings are energized during +ve inductance slope and energy is drawn from dc supply
// to increase the speed
// With MainMode=1=Regerative Braking if actual speed < reference speed
// windings are NOT energized during +ve inductance slope and NO energy is drawn from dc supply
// to increase the speed
Uint16 MPPTAuto=0; // 0=>manually enter RPMSpeedSetpoint from watch window
                   // 1=> use sw on GPIO27 in sync with sw for VFD Torque profile corresponding
			       // to wind speeds v1, v2 for MPPT. Active only for MainMode=1
const float LoadResistance=212.5; // Cold resistance 80W = 50E, 160W=25E
								  // hot resistance of 40W bulb = 1500E with 4 in paralled = 375E

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

struct CPUTIMER_VARS CpuTimer0;
struct CPUTIMER_VARS CpuTimer1;
struct CPUTIMER_VARS CpuTimer2;

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
//__interrupt void adc_isr2(void);
//__interrupt void adc_isr3(void);
//__interrupt void adc_isr4(void);
//__interrupt void adc_isr5(void);
__interrupt void cpu_timer0_isr(void);  //@ 100us, 0.1ms
__interrupt void cpu_timer1_isr(void);  //@1ms
__interrupt void cpu_timer2_isr(void);

void update_compare(EPWM_INFO*);

// Global variables used in this example

Uint16 Current1BuffIndex=0, Current2BuffIndex=0, Current3BuffIndex=0, Current4BuffIndex=0, Voltage1BuffIndex=0, SetpointIndex;
int Current1[10];
int Current2[10];
int Current3[10];
int Current4[10];
int Voltage1[10];
int SpeedCurrentSetpoint[10]; // for speed/current setpoint using pot connected to AI
float Current1OffsetCount=0, Current2OffsetCount=0, Current3OffsetCount=0, Current4OffsetCount=0, VoltageOffsetCount=120;
float Current1Actual[500], Current2Actual[500], Current3Actual[500], Current4Actual[500], VoltageActual[500]; // memory limit hit at 1000*5
Uint16 Current1ActualIndex=0, Current2ActualIndex=0, Current3ActualIndex=0, Current4ActualIndex=0, VoltageActualIndex=0;
Uint16 TempIndex=0;

float ChopperTARGETCount=10000;  // for EPWM sw freq of 10KHz, take avg power for 10s
float ChopperTOTALCount=1, ChopperONCount=0, ChopperOFFCount=0, ChopperONCountLastPeriod=0, ChopperOFFCountLastPeriod=0;
float ChopperDutyLastPeriod, ChopperDutyCurrentPeriod=0, ChopperDutyAvg=0;
float LoadPowerHyst=0.0, LoadPowerPWM=0.0001;

int PhaseAOnCount[10], PhaseBOnCount[10], PhaseCOnCount[10], PhaseDOnCount[10];
int PhaseAOnCountIndex=0, PhaseBOnCountIndex=0, PhaseCOnCountIndex=0, PhaseDOnCountIndex=0;

int i;
float RPMSpeedSetpoint=600, RPMSpeedSetpointInternal=0, CurrentSetpointAmpsRaw=0, RPMSpeedError=0, KpSpd=0.08, KpSpdGen=0.09 ;
float RPMCorrection=0, IntegralTermSpd=0, KiSpd=0.0008, KiSpdGen=-0.5; //KiSpd adjusted in motor mode to 0.0002
float RPMSpeedRampRate=20, RPMSpeedSetpointRampError=0;
Uint16 RampCount=0;

float FilteredVoltage=0, FilteredVoltageLast=0; // low pass filter terms
float VoltageErrorCount=0, KpVolt=0.5, KiVolt=0.5, IntegralTermVolt=0, VoltageControlCountFloat=2001.0; // chopper initially off
Uint16  VoltageControlCountInt=2001; // chopper initially off
int TestVoltage;
int VoltageCorrectionCount=0;

float Current1Amps[10];
float Current2Amps[10];
float Current3Amps[10];
float Current4Amps[10];
float Voltage1Volts[10];

Uint16 count=0; // to keep track of excited phase
int CurrentSetpointCount=0, CurrentSetpointCount1=0, CurrentSetpointCount2=0, CurrentSetpointCount3=0, CurrentSetpointCount4=0;
int CurrentSetpointCount1Old=0, CurrentSetpointCount2Old=0, CurrentSetpointCount3Old=0, CurrentSetpointCount4Old=0;

Uint16 Current1SetpointReachedRegen=0, Current2SetpointReachedRegen=0, Current3SetpointReachedRegen=0, Current4SetpointReachedRegen=0;
Uint16 Current1HighCount=0, Current2HighCount=0, Current3HighCount=0, Current4HighCount=0;
Uint16 HardChop1Count=0, SoftChop1Count=0;

float ChopperPWMDuty=0, ChopperPWMDutySummation=0, ChopperPWMDutySumCount=0;
float ChopperPWMDutyCurrentAvg=0, ChopperPWMDutyPreviousAvg=0, ChopperPWMDutyFinalAvg=0, VoltageControlCountFloatSample=0;
float GenBusVoltageSummation=0, GenBusVoltagePreviousAvg=0;

float CurrentSetpointAmps = 0;  // 3A or 3.5A reqd for open loop, 0 for closed loop speed control
float SpeedCurrentSetpointMean=0; //from ADCINA5
int VoltageSetpointCount=0;
float VoltageSetpointVolts=100; // 100V for gen 110V for motoring
float VoltageCorrectionVolts=17; // around 17V
int CurrentHysteresisCount;
float CurrentHysteresisAmps = 0.05; // 0.1A
int VoltageHysteresisCount;
float VoltageHysteresisVolts = 20.0; // 20V with PI control, 1V without PI control
float CurrentLimit=5;   // current limit of 5A

Uint16 Mode=0; // 0=> motoring, 1=> regerative braking
Uint16 YellowLEDSensor=0, GreenLEDSensor=0; // would be better to initilaize to illegal value 2(?) to wait for correct update from GPIO
Uint16 YellowLEDSensorOldValue=0, GreenLEDSensorOldValue=0;
Uint16 YellowRisingCount=0, GreenRisingCount=0;
Uint16 YellowFallingCount=0, GreenFallingCount=0;
Uint16 YellowRisingEdge, GreenRisingEdge;
Uint16 YellowFallingEdge, GreenFallingEdge;
Uint32 YellowFallingToRisingCount=65534, GreenFallingToRisingCount=65534; // initially at rest
Uint32 YellowRisingToFallingCount=65534, GreenRisingToFallingCount=65534;
Uint32 YellowOFFCount=65534, GreenOFFCount=65534; // 11 => min value to avoid noise
Uint32 YellowONCount=65534, GreenONCount=65534;   // 50001 => 1 rpm for initial rest condition
Uint16 YellowCount=65534, YellowRisingToRisingCount=65534;  // initially at rest
Uint16 GreenCount=65534, GreenRisingToRisingCount=65534;

float AbsoluteRotorPosition=0.0; // could also use a count 0-60deg => 0 to 60Deg_Count
float CalculatedRotorPosition=0.0;
float PosMax=0.0;
float PosMaxArray[1000];
Uint16 PosMaxArrayIndex=0;
Uint32 Position64=0.0;
float MeanCountFor60DegRotation=0.0;
float MeanTimeFor60DegRotation=0.0;
float MeanTimeFor360DegRotation=0.0;
float RPS=0.0;
float RPM=0.0, CorrectedRPM=0.0;
const float t_CPU_T0_ISR = 0.1e-3; //100us
Uint32 T0ISRTestCount=0;
Uint32 T0ISRTestCountHold=0;

Uint16 WindSpeedIn=0;
float WTLamda=8.1, WTRadius=2.44, GearRatio=3.0, WindSpeed=0.0, WindSpeedv1=7.89, WindSpeedv2=9.99, MPPTConstant=0.0;
// 7.89m/s => 750 rpm, 9.99m/s=> 950 rpm
const float PI=3.142857143;
Uint16 Scan1s=1000; // used with CPU_Timer1 counting @ 1ms. 1st cycle executed immediately

#define ADC_usDELAY  1000L

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  500  // Period register

Uint16 count5s=0;
Uint16 MPPTHillClimb=0;
Uint16 OldDir=0, Dir=0;
float MPPTSpeedError=0, OldRPMSpeedSetpoint=0;
float OldLoadPowerPWM=0.000; // small difference initially to take care of == condition
												 // initially in the MPPT loop

void main(void)
{

   if(MainMode==1) // for regenerative braking start directly at the set speed.
   {

	   RPMSpeedSetpointInternal=100;// set some initial target rpm corresponding to cut-in speed
   }
   else           // for motoring ramp from zero.
   {
	   RPMSpeedSetpointInternal=0;
   }

//
   MPPTConstant= (WTLamda*GearRatio*60.0)/(2.0*PI*WTRadius); // calc MPPT constant at start-up based on
                                                   // wind turbine parameters

// Initialize Global Variable Arrays
   for(TempIndex=0;TempIndex<500;TempIndex++)
   {
	   Current1Actual[TempIndex]=0;
	   Current2Actual[TempIndex]=0;
	   Current3Actual[TempIndex]=0;
	   Current4Actual[TempIndex]=0;
	   VoltageActual[TempIndex]=0;
   }

   for(TempIndex=0;TempIndex<10;TempIndex++)
   {
	   PhaseAOnCount[TempIndex]=0;
	   PhaseBOnCount[TempIndex]=0;
	   PhaseCOnCount[TempIndex]=0;
	   PhaseDOnCount[TempIndex]=0;
	   Current1[TempIndex]=0;
	   Current2[TempIndex]=0;
	   Current3[TempIndex]=0;
	   Current4[TempIndex]=0;
	   Voltage1[TempIndex]=0;
	   Current1Amps[TempIndex]=0;
	   Current2Amps[TempIndex]=0;
	   Current3Amps[TempIndex]=0;
	   Current4Amps[TempIndex]=0;
	   Voltage1Volts[TempIndex]=0;
	   SpeedCurrentSetpoint[TempIndex]=0;
   }



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
/*   PieVectTable.ADCINT2 = &adc_isr2;
   PieVectTable.ADCINT3 = &adc_isr3;
   PieVectTable.ADCINT4 = &adc_isr4;
   PieVectTable.ADCINT5 = &adc_isr5; */
   PieVectTable.TINT0 = &cpu_timer0_isr;
   PieVectTable.TINT1 = &cpu_timer1_isr;
   PieVectTable.TINT2 = &cpu_timer2_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

   //  initialize the Cpu Timers -- start
   InitCpuTimers();

// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 80MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 80, 100);//100us sampling for commutation
   ConfigCpuTimer(&CpuTimer1, 80, 1000);//1ms used for speed setpoint ramping
   ConfigCpuTimer(&CpuTimer2, 80, 1000000);

// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the
// below settings must also be updated.

   CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
   CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
   CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0


//  initialize the Cpu Timers -- end


//  initialize the ePWM -- start
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
   EDIS;

   InitEPwm1Example();
   InitEPwm2Example();
   InitEPwm3Example();
   InitEPwm4Example();
   InitEPwm5Example();
//  initialize the ePWM -- end


// Initialize GPIO inputs -- start
   EALLOW;
   //  GPIO-32 - PIN FUNCTION = Yellow Opto Sensor
   	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;	// 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
   	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;		// 1=OUTput,  0=INput
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;    // Enable pull-up on GPIO32
   //	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;	// uncomment if --> Set Low initially
   //	GpioDataRegs.GPBSET.bit.GPIO32 = 1;		// uncomment if --> Set High initially

   //--------------------------------------------------------------------------------------
   //  GPIO-33 - PIN FUNCTION = Green Opto Sensor
   	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;	// 0=GPIO,  1=I2C-SCL,  2=SYNCO,  3=ADCSOCB
   	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 0;		// 1=OUTput,  0=INput
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;    // Enable pull-up on GPIO32
   //	GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;	// uncomment if --> Set Low initially
   //	GpioDataRegs.GPBSET.bit.GPIO33 = 1;		// uncomment if --> Set High initially
   //--------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------
    //  GPIO-27 - PIN FUNCTION = Wind Speed input for Auto Mode
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;	// 0=GPIO,  1=I2C-SCL,  2=SYNCO,  3=ADCSOCB
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;		// 1=OUTput,  0=INput
	GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;    // Enable pull-up on GPIO27
    //	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;	// uncomment if --> Set Low initially
    //	GpioDataRegs.GPASET.bit.GPIO27 = 1;		// uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------

	// Set input qualifcation period for GPIO32 & GPIO33
	GpioCtrlRegs.GPBCTRL.bit.QUALPRD0=510;  // Qual period = T_SYSCLKOUT*4
	GpioCtrlRegs.GPBQSEL1.bit.GPIO32=2;   // 6 samples
	GpioCtrlRegs.GPBQSEL1.bit.GPIO33=2;   // 6 samples

	// Set input qualifcation period for GPIO27
	GpioCtrlRegs.GPACTRL.bit.QUALPRD3=510;  // Qual period = T_SYSCLKOUT*4
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27=2;   // 6 samples
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27=2;   // 6 samples

    //  GPIO-34 - PIN FUNCTION = LED for Experimenter Kit
	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;	// 0=GPIO,  1=COMP2OUT,  2=EMU1,  3=Resv
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;		// 1=OUTput,  0=INput
    //	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPBSET.bit.GPIO34 = 1;		// uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------

    EDIS;
// Initialize GPIO inputs -- end

// Initialize ADC -- start

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
// Initialize ADC -- end

   
// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWMINT1-5 in PIE:
   IER |= M_INT3;
// Enable CPU INT10 which is connected to ADCINT1-5 in PIE
//   IER |= M_INT10; 		// Enable CPU Interrupt 10
   IER |= M_INT1; 			// Enable CPU Interrupt 1 for ADCINT1
// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:
//  IER |= M_INT1;   // CPU-Timer 0 -- already enabled above
  IER |= M_INT13;  // CPU-Timer 1
  IER |= M_INT14; // CPU-Timer 2



// Enable EPWM INTn in the PIE: Group 3 interrupt 1-5
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx5 = 1;

// Enable ADC INTn in the PIE: Group 10 interrupt 1-5
// INT10.x not working ISRs corresponding to ADCINTx are just not called !!
/*   PieCtrlRegs.PIEIER10.bit.INTx1 = 1; //Group1 Interrupt 10 for ADCINT1
   PieCtrlRegs.PIEIER10.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER10.bit.INTx3 = 1;
   PieCtrlRegs.PIEIER10.bit.INTx4 = 1;
   PieCtrlRegs.PIEIER10.bit.INTx5 = 1;  */

// prog does not work if the two lines below are commented !!
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1; //Group1 Interrupt 1 for ADCINT1 **
   PieCtrlRegs.PIEIER1.bit.INTx2 = 1; //Group1 Interrupt 1 for ADCINT2 **

// Enable TINT0 in the PIE: Group 1 interrupt 7
 PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

// Configure ADC
//   AdcChanConfig();

	EALLOW;

       AdcRegs.ADCSOC0CTL.bit.CHSEL= 0;   		// SOC0 ==> ADCINA0
       AdcRegs.ADCSOC1CTL.bit.CHSEL= 1;   		// SOC1 ==> ADCINA1
       AdcRegs.ADCSOC2CTL.bit.CHSEL= 2;   		// SOC2 ==> ADCINA2   // now CH2 & 4 working!
       AdcRegs.ADCSOC3CTL.bit.CHSEL= 3;   		// SOC3 ==> ADCINA3
       AdcRegs.ADCSOC4CTL.bit.CHSEL= 4;   		// SOC4 ==> ADCINA4
       AdcRegs.ADCSOC5CTL.bit.CHSEL= 5;   		// SOC5 ==> ADCINA5

	   AdcRegs.INTSEL1N2.bit.INT1E     = 1;	// Enabled ADCINT1
/*	   AdcRegs.INTSEL1N2.bit.INT2E     = 1;	// Enabled ADCINT2
	   AdcRegs.INTSEL3N4.bit.INT3E     = 1;	// Enabled ADCINT3
	   AdcRegs.INTSEL3N4.bit.INT4E     = 1;	// Enabled ADCINT4
	   AdcRegs.INTSEL5N6.bit.INT5E     = 1;	// Enabled ADCINT5  */

	   AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;	// Disable ADCINT1 Continuous mode
/*	   AdcRegs.INTSEL1N2.bit.INT2CONT  = 0;	// Disable ADCINT2 Continuous mode
	   AdcRegs.INTSEL3N4.bit.INT3CONT  = 0;	// Disable ADCINT3 Continuous mode
	   AdcRegs.INTSEL3N4.bit.INT4CONT  = 0;	// Disable ADCINT4 Continuous mode
	   AdcRegs.INTSEL5N6.bit.INT5CONT  = 0;	// Disable ADCINT5 Continuous mode  */

	   AdcRegs.INTSEL1N2.bit.INT1SEL 	= 4;    // setup EOC4 to trigger ADCINT1 to fire
	   	   	   	   	   	   	   	   	   	   	   	// - do the round robin and let the last
	   	   	   	   	   	   	   	   	   	   	    //EOC fire ADCINT1
/*	   AdcRegs.INTSEL1N2.bit.INT2SEL 	= 1;    // setup EOC1 to trigger ADCINT2 to fire
	   AdcRegs.INTSEL3N4.bit.INT3SEL 	= 2;    // setup EOC2 to trigger ADCINT3 to fire
	   AdcRegs.INTSEL3N4.bit.INT4SEL 	= 3;    // setup EOC3 to trigger ADCINT4 to fire
	   AdcRegs.INTSEL5N6.bit.INT5SEL 	= 4;    // setup EOC4 to trigger ADCINT5 to fire */

	   // all SOCs triggered by EPWM1A - we let them do the round robin from SOC0->4
	   // we the use the last EOC4 to trigger the ADCINT1
	   AdcRegs.ADCSOC0CTL.bit.TRIGSEL 	= 5;    // set SOC0 start trigger on EPWM1A
	   AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 5;    // set SOC1 start trigger on EPWM1A
	   AdcRegs.ADCSOC2CTL.bit.TRIGSEL 	= 5;    // set SOC2 start trigger on EPWM1A
	   AdcRegs.ADCSOC3CTL.bit.TRIGSEL 	= 5;    // set SOC3 start trigger on EPWM1A
	   AdcRegs.ADCSOC4CTL.bit.TRIGSEL 	= 5;    // set SOC4 start trigger on EPWM1A
	   AdcRegs.ADCSOC5CTL.bit.TRIGSEL 	= 5;    // set SOC5 start trigger on EPWM1A


/*	   AdcRegs.ADCSOC1CTL.bit.TRIGSEL 	= 7;    // set SOC1 start trigger on EPWM2A
	   AdcRegs.ADCSOC2CTL.bit.TRIGSEL 	= 9;    // set SOC2 start trigger on EPWM3A
	   AdcRegs.ADCSOC3CTL.bit.TRIGSEL 	= 11;    // set SOC3 start trigger on EPWM4A
	   AdcRegs.ADCSOC4CTL.bit.TRIGSEL 	= 13;    //set SOC4 start trigger on EPWM5A  */

	   AdcRegs.ADCSOC0CTL.bit.ACQPS 	= 6;	// set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC1CTL.bit.ACQPS 	= 6;	// set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC2CTL.bit.ACQPS 	= 6;	// set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC3CTL.bit.ACQPS 	= 6;	// set SOC3 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC4CTL.bit.ACQPS 	= 6;	// set SOC4 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	   AdcRegs.ADCSOC5CTL.bit.ACQPS 	= 6;	// set SOC5 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

	EDIS;

    DELAY_US(ADC_usDELAY);                  // Delay before converting ADC channels

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;)
   {
      __asm("          NOP");
      // low priority processing in the infinite loop - background tasks

      // for speed/current setpoint using pot connected to AI
      SpeedCurrentSetpointMean = (SpeedCurrentSetpoint[0]+SpeedCurrentSetpoint[1]+SpeedCurrentSetpoint[2]+
    		  SpeedCurrentSetpoint[3]+SpeedCurrentSetpoint[4]+SpeedCurrentSetpoint[5]+SpeedCurrentSetpoint[6]+
			  SpeedCurrentSetpoint[7]+SpeedCurrentSetpoint[8]+SpeedCurrentSetpoint[9])/10;

//______________consider moving speed control to 0.1ms cpu_timer0 ISR__________________________
      // speed loop moved to cpu_timer0_isr



//__________________________consider moving inside cpu_timer0 ISR_________________

    	// limit voltage setpoint that may be changed fom the watch-window
    	if(VoltageSetpointVolts<50) // min voltage setpoint is 50V
    	{
    		VoltageSetpointVolts=50.0;
    	}
    	if(VoltageSetpointVolts>180) // max voltage setpoint is 150V
    	{
    		VoltageSetpointVolts=180.0;
    	}

      // convert setpoint and hysteresis bands from amps / volts to counts
      CurrentSetpointCount = (Uint16)(((2482*CurrentSetpointAmps)/15)+1241); // use hardcode or watch-window
      //CurrentSetpointCount = (Uint16)(1241.0+((827.0/4096.0)*SpeedCurrentSetpointMean)); // use ADCINA5 input
      CurrentHysteresisCount = (Uint16)((2482*CurrentHysteresisAmps)/15);
      VoltageSetpointCount = (Uint16)(((2482*VoltageSetpointVolts)/350)+1241);
      VoltageHysteresisCount = (Uint16)((2482*VoltageHysteresisVolts)/350);
      VoltageCorrectionCount = (Uint16)((2482*VoltageCorrectionVolts)/350);

      // convert sensed signals from counts to amps and volts
      Current1Amps[Current1BuffIndex]= (((float)(Current1[Current1BuffIndex])+Current1OffsetCount-1241.0)*15.0)/2482.0;
      Current2Amps[Current2BuffIndex]= (((float)(Current2[Current2BuffIndex])+Current2OffsetCount-1241.0)*15.0)/2482.0;
      Current3Amps[Current3BuffIndex]= (((float)(Current3[Current3BuffIndex])+Current3OffsetCount-1241.0)*15.0)/2482.0;
      Current4Amps[Current4BuffIndex]= (((float)(Current4[Current4BuffIndex])+Current4OffsetCount-1241.0)*15.0)/2482.0;

      // power calculations are now moved into the adc_isr
     // LoadPowerHyst = ((Voltage1Volts[Voltage1BuffIndex]*Voltage1Volts[Voltage1BuffIndex])/LoadResistance)*ChopperDutyAvg ;
     //LoadPowerPWM = ((VoltageSetpointVolts*VoltageSetpointVolts)/LoadResistance)*ChopperPWMDutyFinalAvg ;

   }
} 

__interrupt void epwm1_isr(void)
{
   // Clear INT flag for this timer
   EPwm1Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

__interrupt void epwm2_isr(void)
{
   // Clear INT flag for this timer
   EPwm2Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm3_isr(void)
{
   // Clear INT flag for this timer
	EPwm3Regs.ETCLR.bit.INT = 1;
   
   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm4_isr(void)
{
   // Clear INT flag for this timer
   EPwm4Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm5_isr(void)
{
   // Clear INT flag for this timer
   EPwm5Regs.ETCLR.bit.INT = 1;

   // Acknowledge this interrupt to receive more interrupts from group 3
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void  adc_isr1(void) // called every EPWM1A PRD event => 10KHz
{
  Current1[Current1BuffIndex] = AdcResult.ADCRESULT0;
  Current2[Current2BuffIndex] = AdcResult.ADCRESULT1;
  Current3[Current3BuffIndex] = AdcResult.ADCRESULT2;
  Current4[Current4BuffIndex] = AdcResult.ADCRESULT3;
  Voltage1[Voltage1BuffIndex] = AdcResult.ADCRESULT4;
  SpeedCurrentSetpoint[SetpointIndex] = AdcResult.ADCRESULT5; // for speed/current setpoint using pot connected to AI

  if (Mode==0) // motoring mode
  {
	  // check and update EPWM1 levels .. switch states
	  if(CurrentSetpointCount1!=0)
	  {

			  if ((Current1[Current1BuffIndex]+Current1OffsetCount)<(CurrentSetpointCount1-CurrentHysteresisCount))
			  {   // turn on switches
				  EPwm1Regs.CMPA.half.CMPA = 0;
				  EPwm1Regs.CMPB = 0;
			  }

			  if ((Current1[Current1BuffIndex]+Current1OffsetCount)>(CurrentSetpointCount1+CurrentHysteresisCount))
			  {   // turn off upper switch - soft chop
				  EPwm1Regs.CMPA.half.CMPA = 2001;
				  EPwm1Regs.CMPB = 0;
			  }
	  }
	  if(CurrentSetpointCount1==0)
	  {   // turn off both switches
		  EPwm1Regs.CMPA.half.CMPA = 2001;
		  EPwm1Regs.CMPB = 2001;
	  }

	 // check and update EPWM2 levels .. switch states
	  // turn on switches
	  if(CurrentSetpointCount2!=0)
	  {
		  if ((Current2[Current2BuffIndex]+Current2OffsetCount)<(CurrentSetpointCount2-CurrentHysteresisCount))
		  {
			  EPwm2Regs.CMPA.half.CMPA = 0;
			  EPwm2Regs.CMPB = 0;
		  }
		  // turn off upper switch - soft chopping
		  if ((Current2[Current2BuffIndex]+Current2OffsetCount)>(CurrentSetpointCount2+CurrentHysteresisCount))
		  {
			  EPwm2Regs.CMPA.half.CMPA = 2001;
			  EPwm2Regs.CMPB = 0;
		  }
	  }
	  if(CurrentSetpointCount2==0)
	  {   // turn off both switches
		  EPwm2Regs.CMPA.half.CMPA = 2001;
		  EPwm2Regs.CMPB = 2001;
	  }
	  // check and update EPWM3 levels .. switch states
	  // turn on switches
	  if(CurrentSetpointCount3!=0)
	  {
		  if ((Current3[Current3BuffIndex]+Current3OffsetCount)<(CurrentSetpointCount3-CurrentHysteresisCount))
		  {
			  EPwm3Regs.CMPA.half.CMPA = 0;
			  EPwm3Regs.CMPB = 0;
		  }
		  // turn off upper switch - soft chopping
		  if ((Current3[Current3BuffIndex]+Current3OffsetCount)>(CurrentSetpointCount3+CurrentHysteresisCount))
		  {
			  EPwm3Regs.CMPA.half.CMPA = 2001;
			  EPwm3Regs.CMPB = 0;
		  }
	  }
	  if(CurrentSetpointCount3==0)
	  {  // turn off both the switches
		  EPwm3Regs.CMPA.half.CMPA = 2001;
		  EPwm3Regs.CMPB = 2001;
	  }
	  // check and update EPWM4 levels .. switch states
	  if(CurrentSetpointCount4!=0)
	  {
		  // turn on switches
		  if ((Current4[Current4BuffIndex]+Current4OffsetCount)<(CurrentSetpointCount4-CurrentHysteresisCount))
		  {
			  EPwm4Regs.CMPA.half.CMPA = 0;
			  EPwm4Regs.CMPB = 0;
		  }
		  // turn off upper switch -- soft chopping
		  if ((Current4[Current4BuffIndex]+Current4OffsetCount)>(CurrentSetpointCount4+CurrentHysteresisCount))
		  {
			  EPwm4Regs.CMPA.half.CMPA = 2001;
			  EPwm4Regs.CMPB = 0;
		  }
	  }
	  if(CurrentSetpointCount4==0)
	  {   // turn off both the switches
		  EPwm4Regs.CMPA.half.CMPA = 2001;
		  EPwm4Regs.CMPB = 2001;
	  }
  }

  if (Mode==1) // regeneration mode -- start
  {
	  // check and update EPWM1 levels .. switch states
	  if(CurrentSetpointCount1!=0)
	  {
		  if ((Current1[Current1BuffIndex]+Current1OffsetCount)<(CurrentSetpointCount1-CurrentHysteresisCount))
		  {
			 if(Current1SetpointReachedRegen==0)
			  {
				  // turn on switches
				  EPwm1Regs.CMPA.half.CMPA = 0;
				  EPwm1Regs.CMPB = 0;
				  HardChop1Count++;
			  }
			 if(Current1SetpointReachedRegen==1)
			  {
				  // turn on lower switch only, soft chop
				  EPwm1Regs.CMPA.half.CMPA = 2001;
				  EPwm1Regs.CMPB = 0;
				  SoftChop1Count++;
			  }
		  }

		  if ((Current1[Current1BuffIndex]+Current1OffsetCount)>(CurrentSetpointCount1+CurrentHysteresisCount))
		  {   // turn off both switches
			  EPwm1Regs.CMPA.half.CMPA = 2001;
			  EPwm1Regs.CMPB = 2001;
			  Current1HighCount++;
			  if (Current1HighCount>3) // qualification to avoid noise
			  {
				  Current1SetpointReachedRegen=1;
			  }
		  }
	  }

	  if(CurrentSetpointCount1==0)
	  {   // turn off both switches
		  EPwm1Regs.CMPA.half.CMPA = 2001;
		  EPwm1Regs.CMPB = 2001;
		  Current1SetpointReachedRegen=0;
		  Current1HighCount=0;
		  HardChop1Count=0;
		  SoftChop1Count=0;
	  } //  EPWM1 end..


	  // check and update EPWM2 levels .. switch states
	  if(CurrentSetpointCount2!=0)
	  {
		  if ((Current2[Current2BuffIndex]+Current2OffsetCount)<(CurrentSetpointCount2-CurrentHysteresisCount))
		  {
			  if(Current2SetpointReachedRegen==0)
			  {
				  // turn on switches
				  EPwm2Regs.CMPA.half.CMPA = 0;
				  EPwm2Regs.CMPB = 0;
			  }
			 if(Current2SetpointReachedRegen==1)
			  {
				  // turn on lower switch only, soft chop
				  EPwm2Regs.CMPA.half.CMPA = 2001;
				  EPwm2Regs.CMPB = 0;
			  }
		  }

		  if ((Current2[Current2BuffIndex]+Current2OffsetCount)>(CurrentSetpointCount2+CurrentHysteresisCount))
		  {   // turn off both switches
			  EPwm2Regs.CMPA.half.CMPA = 2001;
			  EPwm2Regs.CMPB = 2001;
			  Current2HighCount++;
			  if (Current2HighCount>3)// qualification to avoid noise
			  {
				  Current2SetpointReachedRegen=1;
			  }
		  }
	  }

	  if(CurrentSetpointCount2==0)
	  {   // turn off both switches
		  EPwm2Regs.CMPA.half.CMPA = 2001;
		  EPwm2Regs.CMPB = 2001;
		  Current2SetpointReachedRegen=0;
		  Current2HighCount=0;
	  }  //  EPWM2 end..

	  // check and update EPWM3 levels .. switch states
	  if(CurrentSetpointCount3!=0)
	  {
		  if ((Current3[Current3BuffIndex]+Current3OffsetCount)<(CurrentSetpointCount3-CurrentHysteresisCount))
		  {
			  if(Current3SetpointReachedRegen==0)
			  {
				  // turn on switches
				  EPwm3Regs.CMPA.half.CMPA = 0;
				  EPwm3Regs.CMPB = 0;
			  }
			  if(Current3SetpointReachedRegen==1)
			  {
				  // turn on lower switch only, soft chop
				  EPwm3Regs.CMPA.half.CMPA = 2001;
				  EPwm3Regs.CMPB = 0;
			  }
		  }

		  if ((Current3[Current3BuffIndex]+Current3OffsetCount)>(CurrentSetpointCount3+CurrentHysteresisCount))
		  {   // turn off both switches
			  EPwm3Regs.CMPA.half.CMPA = 2001;
			  EPwm3Regs.CMPB = 2001;
			  Current3HighCount++;
			  if (Current3HighCount>3)// qualification to avoid noise
			  {
				  Current3SetpointReachedRegen=1;
			  }
		  }
	  }

	  if(CurrentSetpointCount3==0)
	  {   // turn off both switches
		  EPwm3Regs.CMPA.half.CMPA = 2001;
		  EPwm3Regs.CMPB = 2001;
		  Current3SetpointReachedRegen=0;
		  Current3HighCount=0;
	  }  //  EPWM3 end..

	  // check and update EPWM4 levels .. switch states
	 	  if(CurrentSetpointCount4!=0)
	 	  {
	 		  if ((Current4[Current4BuffIndex]+Current4OffsetCount)<(CurrentSetpointCount4-CurrentHysteresisCount))
	 		  {
	 			  if(Current4SetpointReachedRegen==0)
	 			  {
	 				  // turn on switches
	 				  EPwm4Regs.CMPA.half.CMPA = 0;
	 				  EPwm4Regs.CMPB = 0;
	 			  }
	 			  if(Current4SetpointReachedRegen==1)
	 			  {
	 				  // turn on lower switch only, soft chop
	 				  EPwm4Regs.CMPA.half.CMPA = 2001;
	 				  EPwm4Regs.CMPB = 0;
	 			  }
	 		  }

	 		  if ((Current4[Current4BuffIndex]+Current4OffsetCount)>(CurrentSetpointCount4+CurrentHysteresisCount))
	 		  {   // turn off both switches
	 			  EPwm4Regs.CMPA.half.CMPA = 2001;
	 			  EPwm4Regs.CMPB = 2001;
				  Current4HighCount++;
				  if (Current4HighCount>3)// qualification to avoid noise
				  {
					  Current4SetpointReachedRegen=1;
				  }
	 		  }
	 	  }

	 	  if(CurrentSetpointCount4==0)
	 	  {   // turn off both switches
	 		  EPwm4Regs.CMPA.half.CMPA = 2001;
	 		  EPwm4Regs.CMPB = 2001;
	 		  Current4SetpointReachedRegen=0;
	 		 Current4HighCount=0;
	 	  }  //  EPWM4 end..

  }// regeneration mode -- end


  // log the actual measured current for phase A
  Current1ActualIndex++;
  if (Current1ActualIndex>=500)
  {
	  Current1ActualIndex=0;
  }
  Current1Actual[Current1ActualIndex] = (((float)(Current1[Current1BuffIndex])+Current1OffsetCount-1241.0)*15.0)/2482.0;


  // log the actual measured voltage
  VoltageActualIndex++;
  if (VoltageActualIndex>=500)
  {
	  VoltageActualIndex=0;
  }
  VoltageActual[VoltageActualIndex] = (((float)(Voltage1[Voltage1BuffIndex])+VoltageOffsetCount-1241.0)*350.0)/2482.0;

  //Voltage PI Control --------------------------------start----------------------
  //FilteredVoltage=FilteredVoltageLast+((100e-6)/(10e-3)*((float)Voltage1[Voltage1BuffIndex]-FilteredVoltageLast)); // won't work for us
  VoltageErrorCount=(float)(VoltageSetpointCount-VoltageCorrectionCount-Voltage1[Voltage1BuffIndex]); // actual
  //VoltageErrorCount=(float)(TestVoltage-VoltageSetpointCount); // for simulation only
  if ((VoltageControlCountInt>25)&&(VoltageControlCountInt<475)) // 100,11900
  {
	  IntegralTermVolt=IntegralTermVolt+(VoltageErrorCount*0.1e-3);
  }
  // 500=> fully OFF, 0=> fully ON
  VoltageControlCountFloat=500.0+((VoltageErrorCount*KpVolt)+(IntegralTermVolt*KiVolt)); //2000
  VoltageControlCountInt=(Uint16)VoltageControlCountFloat;

  if(VoltageControlCountInt>475) // avoid excessive switching near the off position //1900
  {
	  VoltageControlCountInt=500;
  }
  if (VoltageControlCountInt<25) //100
  {
	  VoltageControlCountInt=0;  // avoid excessive switching near the on position
  }

  EPwm5Regs.CMPA.half.CMPA = VoltageControlCountInt;
  EPwm5Regs.CMPB = VoltageControlCountInt;

  // for power calc
  VoltageControlCountFloatSample=(float)VoltageControlCountInt;

  //Voltage PI Control --------------------------------end----------------------

// maintain Hysteresis control at a higher voltage level than PI
// eg. PI at 100V  and Hys. at 80-120V
 //Voltage Hysteresis Control ---------------------------start----------------------
  // check and update EPWM5 levels .. switch states
  // turn off switch if voltage less than the set level
  if (Voltage1[Voltage1BuffIndex]<(VoltageSetpointCount-VoltageHysteresisCount))
  {
	  EPwm5Regs.CMPA.half.CMPA = 2001;
	  EPwm5Regs.CMPB = 2001;
	  VoltageControlCountFloatSample =500; // for power calc //2000
	  ChopperOFFCount++;
  }
  // turn on switches if voltage greater than the set level
  if (Voltage1[Voltage1BuffIndex]>(VoltageSetpointCount+VoltageHysteresisCount))
  {
	  EPwm5Regs.CMPA.half.CMPA = 0;
	  EPwm5Regs.CMPB = 0;
	  VoltageControlCountFloatSample =0; // for power calc
	  ChopperONCount++;
  }

  // power calc for the PWM chopper and Hysteresis

    ChopperPWMDuty=1-(VoltageControlCountFloatSample/500); //2000
    ChopperPWMDutySummation=ChopperPWMDutySummation+ChopperPWMDuty;  //(0.0-1.0) .. 10000 times
    GenBusVoltageSummation=GenBusVoltageSummation+VoltageActual[VoltageActualIndex]; // ~ 100..10000 times  .. fits in float type
    ChopperPWMDutySumCount++;
      if(ChopperPWMDutySumCount>=10000)
    {
  	  ChopperPWMDutyPreviousAvg=ChopperPWMDutySummation/10000.0;
  	  GenBusVoltagePreviousAvg=GenBusVoltageSummation/10000.0;
  	  LoadPowerPWM = ((GenBusVoltagePreviousAvg*GenBusVoltagePreviousAvg)/LoadResistance)*(ChopperPWMDutyPreviousAvg*ChopperPWMDutyPreviousAvg);
  	  ChopperPWMDutySumCount=0;
  	  ChopperPWMDutySummation=0;
  	  GenBusVoltageSummation=0;
    }
    // weighed avg between the previous average duty cycle and current average duty cycle
    //ChopperPWMDutyFinalAvg=(ChopperPWMDutyPreviousAvg*(20000-ChopperPWMDutySumCount)/10000)+(ChopperPWMDutyCurrentAvg*ChopperPWMDutySumCount/10000)/2;
    ChopperPWMDutyFinalAvg=ChopperPWMDutyPreviousAvg;


/*    // old calc ..replaced by combined PWM and Hysteresis Calc above...
  //Power calc for the hysteresis chopper -- overriden by the PWM chopper
  ChopperTOTALCount=ChopperONCount+ChopperOFFCount;
  if (ChopperTOTALCount>=ChopperTARGETCount) // count the on and off times for 1000@10KHz=100ms
  {
	  ChopperONCountLastPeriod=ChopperONCount; // update counters for background computation
	  ChopperOFFCountLastPeriod=ChopperOFFCount;
	  ChopperDutyLastPeriod=(ChopperONCount)/(ChopperTARGETCount);
	  ChopperONCount=0;
	  ChopperOFFCount=0;
  }
  ChopperDutyCurrentPeriod=(ChopperONCount)/(ChopperTOTALCount);
  // ensure equal weightage to the counts of the earlier and current periods
  ChopperDutyAvg=((ChopperDutyLastPeriod*((2*ChopperTARGETCount)-ChopperTOTALCount)/ChopperTARGETCount)+
		  (ChopperDutyCurrentPeriod*ChopperTOTALCount/ChopperTARGETCount))/2;
*/

  //Voltage Hysteresis Control ---------------------------end----------------------

  // If 10 conversions have been logged, start over
  if(Current1BuffIndex >= 9)
  {
	  Current1BuffIndex = 0;
  }
  else Current1BuffIndex++;

	// If 10 conversions have been logged, start over
	if(Current2BuffIndex >= 9)
	{
	Current2BuffIndex = 0;
	}
	else Current2BuffIndex++;

  // If 10 conversions have been logged, start over
  if(Current3BuffIndex >= 9)
  {
     Current3BuffIndex = 0;
  }
  else Current3BuffIndex++;

	// If 10 conversions have been logged, start over
	if(Current4BuffIndex >= 9)
	{
	Current4BuffIndex = 0;
	}
	else Current4BuffIndex++;


  // If 10 conversions have been logged, start over
  Voltage1BuffIndex++;
  if(Voltage1BuffIndex >= 9)
  {
	  Voltage1BuffIndex = 0;
  }
  //else Voltage1BuffIndex++;

  // If 10 conversions have been logged, start over
  if(SetpointIndex >= 9)
  {
	  SetpointIndex = 0;
  }
  else SetpointIndex++;



  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;   // Acknowledge interrupt to PIE
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}   // adc_isr end..


/*
__interrupt void cpu_timer0_isr(void) // open loop w/o sensor logic
// CurrentSetpointAmps should be set to 3.5A
{

	YellowLEDSensor = GpioDataRegs.GPBDAT.bit.GPIO32;
	GreenLEDSensor =  GpioDataRegs.GPBDAT.bit.GPIO33;
	if((YellowLEDSensor==1) && (YellowLEDSensorOldValue==0))
	{
		YellowRisingCount=YellowRisingCount+1;
	}
	YellowLEDSensorOldValue = YellowLEDSensor;

  // convert setpoint and hysteresis bands from amps / volts to counts
  CurrentSetpointCount = (Uint16)((2482*CurrentSetpointAmps)/15)+1241;
  CurrentHysteresisCount = (Uint16)((2482*CurrentHysteresisAmps)/15);
  VoltageSetpointCount = (Uint16)((2482*VoltageSetpointVolts)/350)+1241;
  VoltageHysteresisCount = (Uint16)((2482*VoltageHysteresisVolts)/350);

	if (count == 0)
	{
		CurrentSetpointCount1=CurrentSetpointCount;	// turn on phase A
	}
	if (count == 1)
	{
		CurrentSetpointCount3=0;		// turn off phase C
	}
	if (count == 2)
	{
		CurrentSetpointCount2=CurrentSetpointCount;	// turn on phase B
	}
	if (count == 3)
	{
		CurrentSetpointCount1=0;	// turn off phase A
	}
	if (count == 4)
	{
		CurrentSetpointCount4=CurrentSetpointCount; // turn on phase D
	}
	if (count == 5)
	{
		CurrentSetpointCount2=0;		// turn off phase B
	}
	if (count == 6)
	{
		CurrentSetpointCount3=CurrentSetpointCount; // turn on phase C
	}
	if (count == 7)
	{
		CurrentSetpointCount4=0;	// turn off phase D
	}

	count++;
	if (count == 8)
	{
		count = 0;
	}

	CpuTimer0.InterruptCount++;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
*/

__interrupt void cpu_timer0_isr(void) // GPIO sensor logic  @1ms
{
	YellowLEDSensor = GpioDataRegs.GPBDAT.bit.GPIO32;
	GreenLEDSensor =  GpioDataRegs.GPBDAT.bit.GPIO33;

	if((YellowLEDSensor==1) && (YellowLEDSensorOldValue==0) && (YellowOFFCount>10))// yellow rising edge
	{
		YellowRisingCount=YellowRisingCount+1;
		YellowRisingEdge=1;    // should be turned off appropriately

		YellowFallingToRisingCount=YellowOFFCount; //load the count
		YellowRisingToRisingCount=YellowCount;  // rising edge to rising edge count

		YellowOFFCount=0;
		YellowCount=0;
		//AbsoluteRotorPosition=15; // yellow rising edge => 15deg - cannot be trusted
	}
	if((YellowLEDSensor==0) && (YellowLEDSensorOldValue==1) && (YellowONCount>10) )// yellow falling edge
	{
		YellowFallingCount=YellowFallingCount+1;
		YellowFallingEdge=1;      // should be turned off appropriately

		YellowRisingToFallingCount=YellowONCount; //load the count

		YellowONCount=0;
		//AbsoluteRotorPosition=45; // yellow falling edge => 45deg - cannot be trusted
	}
	YellowLEDSensorOldValue = YellowLEDSensor;

	if (YellowLEDSensor==1)
	{
		YellowONCount++;
		YellowFallingEdge=0;  // reset here if not handled elsewhere
	}
	if (YellowLEDSensor==0)
	{
		YellowOFFCount++;
		YellowRisingEdge=0;  // reset here if not handled elsewhere
	}
	YellowCount++;
	if (YellowCount>=5100) // do not wrap around to zero but hold the largest count
	{
		YellowCount=5101;  //=> very slow speed or stationary
		RPS=0;
		RPM=0;			   // force the RPM to 0 so that we can use the <100rpm commutation logic
		CorrectedRPM=0;
	}


	if((GreenLEDSensor==1) && (GreenLEDSensorOldValue==0) && (GreenOFFCount>10))// Green rising edge
	{
		GreenRisingCount=GreenRisingCount+1;
		GreenRisingEdge=1;    // should be turned off appropriately

		GreenFallingToRisingCount=GreenOFFCount; //load the count
		GreenRisingToRisingCount=GreenCount;

		GreenOFFCount=0;
		GreenCount=0;

		// AbsoluteRotorPosition=0; // Green rising edge => 0deg ~ 3 deg after adjusting the sensor
		PosMaxArray[PosMaxArrayIndex]=CalculatedRotorPosition;
		CalculatedRotorPosition=0; // start from zero ( or maybe 3 deg )
		PosMaxArrayIndex++;
		if(PosMaxArrayIndex>=1000)
		{
			PosMaxArrayIndex=0;
		}
	}
	if((GreenLEDSensor==0) && (GreenLEDSensorOldValue==1) && (GreenONCount>10) )// Green falling edge
	{
		GreenFallingCount=GreenFallingCount+1;
		GreenFallingEdge=1;      // should be turned off appropriately

		GreenRisingToFallingCount=GreenONCount; //load the count

		GreenONCount=0;
		//AbsoluteRotorPosition=30; // Green falling edge => 30deg - cannot be trusted
	}
	GreenLEDSensorOldValue = GreenLEDSensor;

	if (GreenLEDSensor==1)
	{
		GreenONCount++;
		GreenFallingEdge=0;  // reset here if not handled elsewhere
	}
	if (GreenLEDSensor==0)
	{
		GreenOFFCount++;
		GreenRisingEdge=0;  // reset here if not handled elsewhere
	}
	GreenCount++;
	if (GreenCount>=5100) // do not wrap around to zero but hold the largest count
	{
		GreenCount=5101;  //=> very slow speed or stationary
		RPS=0;
		RPM=0;			   // force the RPM to 0 so that we can use the <100rpm commutation logic
		CorrectedRPM=0;
	}

	// RPM calculation & Speed Loop
	// mean time to move 60 deg. Rising edge to Rising edge considered
	// Rising to Falling & Falling to rising are not equal so not considered for 30 deg rotation
	if ((YellowRisingEdge==1)||(GreenRisingEdge==1)) //calc speed here
	{
			MeanCountFor60DegRotation=((float)YellowRisingToRisingCount+(float)GreenRisingToRisingCount)/2;

			MeanTimeFor60DegRotation = MeanCountFor60DegRotation*t_CPU_T0_ISR;
			MeanTimeFor360DegRotation = MeanTimeFor60DegRotation*6; // use *12 when working with 30 deg
			if (MeanTimeFor360DegRotation>1e-3) // check for divide by zero
			{
				RPS = 1/MeanTimeFor360DegRotation;
			}
			RPM = RPS*60;
			CorrectedRPM=RPM*1.12;
			YellowRisingEdge=0; // clear the edge notification after calc rpm
			YellowFallingEdge=0;
			GreenRisingEdge=0; // clear the edge notification after calc rpm
			GreenFallingEdge=0;

			//we should move the speed loop here. this is where we have the correct speed and this is where
			// we should take the control action.
			//Speed Control start______________________________________________________________________________
			//Speed Control end______________________________________________________________________________
	}// RPM calculation & Speed Loop .. end.

//speed loop start
	  //Speed Control
		/*if (Mode==1) // speed correction for regeneration mode
		{
	      RPMCorrection=(RPMSpeedSetpointInternal*0.147)-100; // beacuse the controlled speed is always around 60-20rpm above the setpoint
		} */
		RPMSpeedError=RPMSpeedSetpointInternal-RPMCorrection-CorrectedRPM; // force RPMSpeedSetpointInternal to lower value to correct 60-20 rpm offset in speed
		if((CurrentSetpointAmpsRaw>0)&&(CurrentSetpointAmpsRaw<CurrentLimit)) // if the current ref calculated by the PI controller is larger than limit
		{										// disable integral accumulation of error to prevent intergal windup
			IntegralTermSpd = IntegralTermSpd + (RPMSpeedError*t_CPU_T0_ISR); // may need to adjust
			//Integral Time increment = t_CPU_T0_ISR*(YellowRisingToRisingCount+GreenRisingToRisingCount)/2
		}
		if(Mode==0)
		{
			CurrentSetpointAmpsRaw=(RPMSpeedError*KpSpd)+(IntegralTermSpd*KiSpd); // motor mode eq
			if ((RPMSpeedSetpoint<0.5)&&(RPMSpeedError<-40))
			{
					CurrentSetpointAmpsRaw=0;
			}
		}

		if(Mode==1)
		{
			CurrentSetpointAmpsRaw=(RPMSpeedError*KpSpdGen)+(IntegralTermSpd*KiSpdGen); //  gen mode eq
		}



		if(MainMode==0)  // machine mainly running as a motor
		{
			if (RPMSpeedError<-100)
			{
				Mode=1; // set mode to regenerative braking
			}
			else
			{
				Mode=0; // set mode to motoring
			}


			if (Mode==0) // motoring mode
			{
				if (CurrentSetpointAmpsRaw>CurrentLimit)
					{
						CurrentSetpointAmps=CurrentLimit;
					}
				else if ((CurrentSetpointAmpsRaw>0.0001) && (CurrentSetpointAmpsRaw<=CurrentLimit))
					{
						CurrentSetpointAmps=CurrentSetpointAmpsRaw;
					}
				else
					{
					CurrentSetpointAmps = 0.0;
					}
			}

			if (Mode==1) // regenerative braking mode
			{
				if (CurrentSetpointAmpsRaw<-CurrentLimit)
					{
						CurrentSetpointAmps=CurrentLimit;
					}
				else if ((CurrentSetpointAmpsRaw<-0.0001) && (CurrentSetpointAmpsRaw>=-CurrentLimit))
					{
						CurrentSetpointAmps=-CurrentSetpointAmpsRaw;
					}
				else
					{
					CurrentSetpointAmps = 0.0;
					}
			}
		}

		if(MainMode==1)  // machine mainly running as a generator
		{
			Mode=1; // set mode to regenerative braking
			// there is no "Motoring" mode here

			if (Mode==1) // regenerative braking mode
			{
				if (CurrentSetpointAmpsRaw<-CurrentLimit)
					{
						CurrentSetpointAmps=CurrentLimit;
					}
				else if ((CurrentSetpointAmpsRaw<-0.0001) && (CurrentSetpointAmpsRaw>=-CurrentLimit))
					{
						CurrentSetpointAmps=-CurrentSetpointAmpsRaw;
					}
				else
					{
					CurrentSetpointAmps = 0.0; // in case actual speed falls below the reference speed
					}
			}
		}
//speed loop end

	CalculatedRotorPosition=CalculatedRotorPosition+(RPM*60*(0.1e-3)/10); // 1. check with only RPM
																				   // 2. check w/o the div by 10
	// Commutation Logic

	if (Mode==0) // motoring mode
	{
		if(CorrectedRPM<100.0) // use sensor edges for phase commutation
		{    // updated with latest as per drawing 26 Apr
			if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==0))// --ph B aligned posn
			{
				//turn off  phases not assoicated with this position
				CurrentSetpointCount2=0;	// turn off phase B
				CurrentSetpointCount3=0;  // turn off phase C

				if ((PhaseAOnCount[PhaseAOnCountIndex]<=5000)&&    // initially turn on the nearly aligned phase
						(PhaseDOnCount[PhaseDOnCountIndex]<=0))   // not yet tried the next phase
				{
					CurrentSetpointCount1=CurrentSetpointCount; // turn on phase A
					CurrentSetpointCount1Old=CurrentSetpointCount1;
					PhaseAOnCount[PhaseAOnCountIndex]++;
					if (PhaseAOnCount[PhaseAOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseAOnCount[PhaseAOnCountIndex]=5001;
					}
				}

				if ((PhaseAOnCount[PhaseAOnCountIndex]>5000)||   // still in this position after 0.5s
						(PhaseDOnCount[PhaseDOnCountIndex]>0))  // OR now trying out phase D
				{
					CurrentSetpointCount1=0;  // turn off phase A
					CurrentSetpointCount4=CurrentSetpointCount; // try energizing phase D
					CurrentSetpointCount4Old=CurrentSetpointCount4;
					PhaseDOnCount[PhaseDOnCountIndex]++;
					if (PhaseDOnCount[PhaseDOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseDOnCount[PhaseDOnCountIndex]=5001;
					}
				}
			}

			if ((GreenLEDSensor== 1)&&(YellowLEDSensor==0))//-- ph A aligned posn
			{
				//turn off  phases not assoicated with this position
				CurrentSetpointCount1=0;		// turn off phase A
				CurrentSetpointCount2=0;	    // turn off phase B

				if ((PhaseDOnCount[PhaseDOnCountIndex]<=5000)&& // initially turn on the nearly aligned phase
					(PhaseCOnCount[PhaseCOnCountIndex]<=0))    // not yet tried the next phase
				{
					CurrentSetpointCount4=CurrentSetpointCount;	// turn on phase D
					CurrentSetpointCount4Old=CurrentSetpointCount4;
					PhaseDOnCount[PhaseDOnCountIndex]++;
					if (PhaseDOnCount[PhaseDOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseDOnCount[PhaseDOnCountIndex]=5001;
					}
				}

				if ((PhaseDOnCount[PhaseDOnCountIndex]>5000)|| // still in this position after 0.5s
						(PhaseCOnCount[PhaseCOnCountIndex]>0))  // OR now trying out phase C
				{
					CurrentSetpointCount4=0;  // turn off phase D
					CurrentSetpointCount3=CurrentSetpointCount; // try energizing phase C
					CurrentSetpointCount3Old=CurrentSetpointCount3;
					PhaseCOnCount[PhaseCOnCountIndex]++;
					if (PhaseCOnCount[PhaseCOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseCOnCount[PhaseCOnCountIndex]=5001;
					}
				}
			}

			if ((GreenLEDSensor== 1)&&(YellowLEDSensor==1)) //-- phase D aligned posn
			{
				//turn off  phases not assoicated with this position
				CurrentSetpointCount4=0;	    // turn off phase D
				CurrentSetpointCount1=0;        // turn off phase A

				if ((PhaseCOnCount[PhaseCOnCountIndex]<=5000)&& // initially turn on the nearly aligned phase
						(PhaseBOnCount[PhaseBOnCountIndex]<=0))   // not yet tried the next phase
				{
					CurrentSetpointCount3=CurrentSetpointCount;	// turn on phase C
					CurrentSetpointCount3Old=CurrentSetpointCount3;
					PhaseCOnCount[PhaseCOnCountIndex]++;
					if (PhaseCOnCount[PhaseCOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseCOnCount[PhaseCOnCountIndex]=5001;
					}
				}

				if ((PhaseCOnCount[PhaseCOnCountIndex]>5000)|| // still in this position after 0.5s
						(PhaseBOnCount[PhaseBOnCountIndex]>0))  // OR now trying out phase B
				{
					CurrentSetpointCount3=0;  // turn off phase C
					CurrentSetpointCount2=CurrentSetpointCount; // try energizing phase B
					CurrentSetpointCount2Old=CurrentSetpointCount2;
					PhaseBOnCount[PhaseBOnCountIndex]++;
					if (PhaseBOnCount[PhaseBOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseBOnCount[PhaseBOnCountIndex]=5001;
					}
				}

			}

			if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==1)) //-- phase C aligned posn
			{
				//turn off  phases not assoicated with this position
				CurrentSetpointCount3=0;	// turn off phase C
				CurrentSetpointCount4=0;	// turn off phase D

				if ((PhaseBOnCount[PhaseBOnCountIndex]<=5000)&& // initially turn on the nearly aligned phase
						(PhaseAOnCount[PhaseAOnCountIndex]<=0))   // not yet tried the next phase
				{
					CurrentSetpointCount2=CurrentSetpointCount;	// turn on phase B
					CurrentSetpointCount2Old=CurrentSetpointCount2;
					PhaseBOnCount[PhaseBOnCountIndex]++;
					if (PhaseBOnCount[PhaseBOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseBOnCount[PhaseBOnCountIndex]=5001;
					}
				}

				if ((PhaseBOnCount[PhaseBOnCountIndex]>5000)|| // still in this position after 0.5s
						(PhaseAOnCount[PhaseAOnCountIndex]>0))  // OR now trying out phase A
				{
					CurrentSetpointCount2=0;  // turn off phase B
					CurrentSetpointCount1=CurrentSetpointCount; // try energizing phase A
					CurrentSetpointCount1Old=CurrentSetpointCount1;
					PhaseAOnCount[PhaseAOnCountIndex]++;
					if (PhaseAOnCount[PhaseAOnCountIndex]>5000) // limit max value to prevent wrap around to -ve
					{
						PhaseAOnCount[PhaseAOnCountIndex]=5001;
					}
				}


			}

			if ( (CurrentSetpointCount1==0)&&(CurrentSetpointCount1Old !=0)) // catch falling edge of A
			{
			PhaseAOnCountIndex++;
			if (PhaseAOnCountIndex>=9)
				{
				PhaseAOnCountIndex=0;
				}
			PhaseAOnCount[PhaseAOnCountIndex]=0; // prepare next location for next cycle
			CurrentSetpointCount1Old=0;
			}

			if ((CurrentSetpointCount2==0)&&(CurrentSetpointCount2Old !=0)) // catch falling edge of B
			{
			PhaseBOnCountIndex++;
			if (PhaseBOnCountIndex>=9)
				{
				PhaseBOnCountIndex=0;
				}
			PhaseBOnCount[PhaseBOnCountIndex]=0;
			CurrentSetpointCount2Old=0;
			}

			if ((CurrentSetpointCount3==0)&&(CurrentSetpointCount3Old !=0)) // catch falling edge of C
			{
			PhaseCOnCountIndex++;
			if (PhaseCOnCountIndex>=9)
				{
				PhaseCOnCountIndex=0;
				}
			PhaseCOnCount[PhaseCOnCountIndex]=0;
			CurrentSetpointCount3Old=0;
			}

			if ((CurrentSetpointCount4==0)&&(CurrentSetpointCount4Old !=0)) // catch falling edge of D
			{
			PhaseDOnCountIndex++;
			if (PhaseDOnCountIndex>=9)
				{
				PhaseDOnCountIndex=0;
				}
			PhaseDOnCount[PhaseDOnCountIndex]=0;
			CurrentSetpointCount4Old=0;
			}


		} // motoring < 100 rpm end...

		else  //speed > 100 rpm use speed estimation for phase commutation ( motoring )
		{     //  commutation logic updated 26 Apr
			if ((CalculatedRotorPosition>=0) && (CalculatedRotorPosition<15)) // Phase C from 0 to 15 deg
			//if ((GreenLEDSensor== 1)&&(YellowLEDSensor==0))//-- ph A aligned posn ->( dec. inductance slope)
			{   // updated
				CurrentSetpointCount3=CurrentSetpointCount;	// turn on phase C
				CurrentSetpointCount3Old=CurrentSetpointCount3;
				PhaseCOnCount[PhaseCOnCountIndex]++;
				CurrentSetpointCount4=0;		// turn off phase D
				if ( CurrentSetpointCount4Old !=0) // catch falling edge of D
				{
				PhaseDOnCountIndex++;
				if (PhaseDOnCountIndex>=9)
					{
					PhaseDOnCountIndex=0;
					}
				PhaseDOnCount[PhaseDOnCountIndex]=0;
				CurrentSetpointCount4Old=0;
				}
			}
			if ((CalculatedRotorPosition>=15) && (CalculatedRotorPosition<30)) // Phase B from 15 to 30 deg
			//if ((GreenLEDSensor== 1)&&(YellowLEDSensor==1)) //-- phase D aligned posn ->( dec. inductance slope)
			{   // updated
				CurrentSetpointCount2=CurrentSetpointCount;	// turn on phase B
				CurrentSetpointCount2Old=CurrentSetpointCount2;
				PhaseBOnCount[PhaseBOnCountIndex]++;
				CurrentSetpointCount3=0;	    // turn off phase C
				if ( CurrentSetpointCount3Old !=0) // catch falling edge of C
				{
				PhaseCOnCountIndex++;
				if (PhaseCOnCountIndex>=9)
					{
					PhaseCOnCountIndex=0;
					}
				PhaseCOnCount[PhaseCOnCountIndex]=0;
				CurrentSetpointCount3Old=0;
				}
			}
			if ((CalculatedRotorPosition>=30) && (CalculatedRotorPosition<45)) // Phase A from 30 to 45 deg
			//if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==1)) //-- phase C aligned posn -> ( dec. inductance slope)
			{   // updated
				CurrentSetpointCount1=CurrentSetpointCount; // turn on phase A
				CurrentSetpointCount1Old=CurrentSetpointCount1;
				PhaseAOnCount[PhaseAOnCountIndex]++;
				CurrentSetpointCount2=0;	// turn off phase B
				if ( CurrentSetpointCount2Old !=0) // catch falling edge of B
				{
				PhaseBOnCountIndex++;
				if (PhaseBOnCountIndex>=9)
					{
					PhaseBOnCountIndex=0;
					}
				PhaseBOnCount[PhaseBOnCountIndex]=0;
				CurrentSetpointCount2Old=0;
				}
			}

			if ((CalculatedRotorPosition>=45) && (CalculatedRotorPosition<60)) // Phase D from 45 to 60 deg
			//if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==0))// --ph B aligned posn -> ( dec. inductance slope)
			{   //updated
				CurrentSetpointCount4=CurrentSetpointCount; // turn on phase D
				CurrentSetpointCount4Old=CurrentSetpointCount4;
				PhaseDOnCount[PhaseDOnCountIndex]++;
				CurrentSetpointCount1=0;	// turn off phase A
				if ( CurrentSetpointCount1Old !=0) // catch falling edge of A
				{
				PhaseAOnCountIndex++;
				if (PhaseAOnCountIndex>=9)
					{
					PhaseAOnCountIndex=0;
					}
				PhaseAOnCount[PhaseAOnCountIndex]=0; // prepare next location for next cycle
				CurrentSetpointCount1Old=0;
				}
			}
		} // motoring > 200 rpm end...
	}// motoring end..

	if (Mode==1)// regerative braking
	{
		if(CorrectedRPM<200.0) // use sensor edges for phase commutation
		{   // updated with latest logic 26 Apr
			if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==0))// --ph B aligned posn
			{
				CurrentSetpointCount2=CurrentSetpointCount; // turn on phase B
				CurrentSetpointCount2Old=CurrentSetpointCount2;
				PhaseBOnCount[PhaseBOnCountIndex]++;
				CurrentSetpointCount3=0;	// turn off phase C
				if ( CurrentSetpointCount3Old !=0) // catch falling edge of C
				{
				PhaseCOnCountIndex++;
				if (PhaseCOnCountIndex>=9)
					{
					PhaseCOnCountIndex=0;
					}
				PhaseCOnCount[PhaseCOnCountIndex]=0; // prepare next location for next cycle
				CurrentSetpointCount3Old=0;
				}
			}

			if ((GreenLEDSensor== 1)&&(YellowLEDSensor==0))//-- ph A aligned posn
			{
				CurrentSetpointCount1=CurrentSetpointCount;	// turn on phase A
				CurrentSetpointCount1Old=CurrentSetpointCount1;
				PhaseAOnCount[PhaseAOnCountIndex]++;
				CurrentSetpointCount2=0;		// turn off phase B
				if ( CurrentSetpointCount2Old !=0) // catch falling edge of B
				{
				PhaseBOnCountIndex++;
				if (PhaseBOnCountIndex>=9)
					{
					PhaseBOnCountIndex=0;
					}
				PhaseBOnCount[PhaseBOnCountIndex]=0;
				CurrentSetpointCount2Old=0;
				}
			}

			if ((GreenLEDSensor== 1)&&(YellowLEDSensor==1)) //-- phase D aligned posn
			{
				CurrentSetpointCount4=CurrentSetpointCount;	// turn on phase D
				CurrentSetpointCount4Old=CurrentSetpointCount4;
				PhaseDOnCount[PhaseDOnCountIndex]++;
				CurrentSetpointCount1=0;	    // turn off phase A
				if ( CurrentSetpointCount1Old !=0) // catch falling edge of A
				{
				PhaseAOnCountIndex++;
				if (PhaseAOnCountIndex>=9)
					{
					PhaseAOnCountIndex=0;
					}
				PhaseAOnCount[PhaseAOnCountIndex]=0;
				CurrentSetpointCount1Old=0;
				}
			}

			if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==1)) //-- phase C aligned posn
			{
				CurrentSetpointCount3=CurrentSetpointCount; // turn on phase C
				CurrentSetpointCount3Old=CurrentSetpointCount3;
				PhaseCOnCount[PhaseCOnCountIndex]++;
				CurrentSetpointCount4=0;	// turn off phase D
				if ( CurrentSetpointCount4Old!=0) // catch falling edge of D
				{
				PhaseDOnCountIndex++;
				if (PhaseDOnCountIndex>=9)
					{
					PhaseDOnCountIndex=0;
					}
				PhaseDOnCount[PhaseDOnCountIndex]=0;
				CurrentSetpointCount4Old=0;
				}
			}
		} // regen < 200 rpm end..

		else //speed > 200 rpm use speed estimation for phase commutation ( regenerative braking )
		{    // logic updated on 26 Apr
			if ((CalculatedRotorPosition>=0) && (CalculatedRotorPosition<15)) // Phase A from 0 to 15 deg
			//if ((GreenLEDSensor== 1)&&(YellowLEDSensor==0))//-- ph A aligned posn -> ( unaligned posn )
			{   //updated
				CurrentSetpointCount1=CurrentSetpointCount;	// turn on phase A
				CurrentSetpointCount1Old=CurrentSetpointCount1;
				PhaseAOnCount[PhaseAOnCountIndex]++;
				CurrentSetpointCount2=0;		// turn off phase B
				if ( CurrentSetpointCount2Old !=0) // catch falling edge of B
				{
				PhaseBOnCountIndex++;
				if (PhaseBOnCountIndex>=9)
					{
					PhaseBOnCountIndex=0;
					}
				PhaseBOnCount[PhaseBOnCountIndex]=0;
				CurrentSetpointCount2Old=0;
				}
			}

			if ((CalculatedRotorPosition>=15) && (CalculatedRotorPosition<30)) // Phase D from 15 to 30 deg
			//if ((GreenLEDSensor== 1)&&(YellowLEDSensor==1)) //-- phase D aligned posn -> ( unaligned posn )
			{   // updated
				CurrentSetpointCount4=CurrentSetpointCount;	// turn on phase D
				CurrentSetpointCount4Old=CurrentSetpointCount4;
				PhaseDOnCount[PhaseDOnCountIndex]++;
				CurrentSetpointCount1=0;	    // turn off phase A
				if ( CurrentSetpointCount1Old !=0) // catch falling edge of A
				{
				PhaseAOnCountIndex++;
				if (PhaseAOnCountIndex>=9)
					{
					PhaseAOnCountIndex=0;
					}
				PhaseAOnCount[PhaseAOnCountIndex]=0;
				CurrentSetpointCount1Old=0;
				}
			}

			if ((CalculatedRotorPosition>=30) && (CalculatedRotorPosition<45)) // Phase C from 30 to 45 deg
			//if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==1)) //-- phase C aligned posn -> ( unaligned posn )
				{   //updated
					CurrentSetpointCount3=CurrentSetpointCount; // turn on phase C
					CurrentSetpointCount3Old=CurrentSetpointCount3;
					PhaseCOnCount[PhaseCOnCountIndex]++;
					CurrentSetpointCount4=0;	// turn off phase D
					if ( CurrentSetpointCount4Old!=0) // catch falling edge of D
					{
					PhaseDOnCountIndex++;
					if (PhaseDOnCountIndex>=9)
						{
						PhaseDOnCountIndex=0;
						}
					PhaseDOnCount[PhaseDOnCountIndex]=0;
					CurrentSetpointCount4Old=0;
					}
				}
			if ((CalculatedRotorPosition>=45) && (CalculatedRotorPosition<60)) // Phase B from 45 to 60 deg
			//if  ((GreenLEDSensor== 0)&&(YellowLEDSensor==0))// --ph B aligned posn-> ( unaligned posn )
				{
					CurrentSetpointCount2=CurrentSetpointCount; // turn on phase B
					CurrentSetpointCount2Old=CurrentSetpointCount2;
					PhaseBOnCount[PhaseBOnCountIndex]++;
					CurrentSetpointCount3=0;	// turn off phase C
					if ( CurrentSetpointCount3Old !=0) // catch falling edge of C
					{
					PhaseCOnCountIndex++;
					if (PhaseCOnCountIndex>=9)
						{
						PhaseCOnCountIndex=0;
						}
					PhaseCOnCount[PhaseCOnCountIndex]=0; // prepare next location for next cycle
					CurrentSetpointCount3Old=0;
					}
				}
		}// regen >200 rpm end..

	}// regen brake mode end..

	// loop time test
	T0ISRTestCount++;
	if(T0ISRTestCount>=10000)
	{
	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	//Toggle GPIO34 (LD2)
	T0ISRTestCountHold=T0ISRTestCount;
	T0ISRTestCount=0;
	}

	CpuTimer0.InterruptCount++;
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


__interrupt void cpu_timer1_isr(void)  // called every 1ms
{

	Scan1s++;
	if(Scan1s>=1000)  // 1s scan
	{
		Scan1s=0;
		if((MainMode==1)&&(MPPTAuto==1)) // scan the wind speed i/p if Generator and MPPTAuto mode
		{
			WindSpeedIn = GpioDataRegs.GPADAT.bit.GPIO27;
			if (WindSpeedIn==0)
			{
				WindSpeed=WindSpeedv1; //750 rpm
			}
			if (WindSpeedIn==1)
			{
				WindSpeed=WindSpeedv2; //950 rpm
			}
			//RPMSpeedSetpoint= (WTLamda*WindSpeed*60.0)/(2.0*PI*WTRadius); // expensive calc
			RPMSpeedSetpoint= MPPTConstant*WindSpeed; // less expensive calc.
		}
	}


	RampCount++;
	if(RampCount>=RPMSpeedRampRate)// RPMSpeedRampRate=20 by default => 1rpm/20ms => 50rpm/sec
	{
		 // Speed ramp up
		 RPMSpeedSetpointRampError = RPMSpeedSetpoint-RPMSpeedSetpointInternal;
		 if(RPMSpeedSetpointRampError>1)
		  {
			  RPMSpeedSetpointInternal=RPMSpeedSetpointInternal+1;
		  }
		 if(RPMSpeedSetpointRampError>0.1)
		  {
			  RPMSpeedSetpointInternal=RPMSpeedSetpointInternal+0.1;
		  }
		 // Speed ramp down
		 if(RPMSpeedSetpointRampError<-1)
		  {
			  RPMSpeedSetpointInternal=RPMSpeedSetpointInternal-1;
		  }
		 if(RPMSpeedSetpointRampError<0.1)
		  {
			  RPMSpeedSetpointInternal=RPMSpeedSetpointInternal-0.1;
		  }
		 RampCount=0;
	}

	count5s++;
	if(count5s>=10000) // 5s scan
	{
		count5s=0;
		if(MPPTHillClimb==1)
		{
			MPPTAuto=0;
		}
		if((MPPTHillClimb==1)&&(CorrectedRPM>500))
				{
					MPPTSpeedError=RPMSpeedSetpoint-CorrectedRPM;

					if((RPMSpeedSetpoint>=1050)||(MPPTSpeedError>=40)) // speed high, on the right of MPP
																	   // not enough WT power to maintain set speed
					{
						OldRPMSpeedSetpoint=RPMSpeedSetpoint;
						RPMSpeedSetpoint=RPMSpeedSetpoint-50;//move towards the left
						//OldLoadPowerPWM=LoadPowerPWM;
						OldLoadPowerPWM=0; // force leftward movement
						OldDir=Dir;
						Dir=0; // decreasing speed
					}
					else if((RPMSpeedSetpoint<=650)||(MPPTSpeedError<=-80))// speed low, on the left of MPP
						                                              // not enough SRG braking power to maintain set speed (may not be the case in actual WT)
					{
						OldRPMSpeedSetpoint=RPMSpeedSetpoint;
						RPMSpeedSetpoint=RPMSpeedSetpoint+50; //move towards the right
						//OldLoadPowerPWM=LoadPowerPWM;
						OldLoadPowerPWM=0; // force rightward movement
						OldDir=Dir;
						Dir=1; // increasing speed
					}

					else
					{
						if((RPMSpeedSetpoint>OldRPMSpeedSetpoint)// for speed within range-executed during 1st MPPTHillClimb Cycle
							&&(LoadPowerPWM>OldLoadPowerPWM))// power increases with increase in speed
															 // Left of peak and moving right towards peak
						{
							OldRPMSpeedSetpoint=RPMSpeedSetpoint;
							RPMSpeedSetpoint=RPMSpeedSetpoint+50;// move towards the right
							OldLoadPowerPWM=LoadPowerPWM;
							OldDir=Dir;
							Dir=1; // increasing speed
						}
						else if((RPMSpeedSetpoint>OldRPMSpeedSetpoint)
							&&(LoadPowerPWM<OldLoadPowerPWM))// power decreases with increase in speed
															 // right of peak and moving right away from peak
						{
							OldRPMSpeedSetpoint=RPMSpeedSetpoint;
							RPMSpeedSetpoint=RPMSpeedSetpoint-50;//move towards the left
							OldLoadPowerPWM=LoadPowerPWM;
							OldDir=Dir;
							Dir=0; // decreasing speed
						}
						else if((RPMSpeedSetpoint<OldRPMSpeedSetpoint)
							&&(LoadPowerPWM>OldLoadPowerPWM))// power increases with decrease in speed
															 // right of peak and moving left towards peak
						{
							OldRPMSpeedSetpoint=RPMSpeedSetpoint;
							RPMSpeedSetpoint=RPMSpeedSetpoint-50;//move towards the left
							OldLoadPowerPWM=LoadPowerPWM;
							OldDir=Dir;
							Dir=0; // decreasing speed
						}
						else if((RPMSpeedSetpoint<OldRPMSpeedSetpoint)
							&&(LoadPowerPWM<OldLoadPowerPWM))// power decreases with decrease in speed
															 // left of peak and moving left away from peak
						{
							OldRPMSpeedSetpoint=RPMSpeedSetpoint;
							RPMSpeedSetpoint=RPMSpeedSetpoint+50;// move towards the right
							OldLoadPowerPWM=LoadPowerPWM;
							OldDir=Dir;
							Dir=1; // increasing speed
						}
						else if((LoadPowerPWM>1)&&(LoadPowerPWM==OldLoadPowerPWM))// power same with change in speed
															 // continue in the same direction
						{
							OldRPMSpeedSetpoint=RPMSpeedSetpoint;
							if(Dir==0)
								{
								RPMSpeedSetpoint=RPMSpeedSetpoint-50;//move towards the left
								}
							if(Dir==1)
								{
								RPMSpeedSetpoint=RPMSpeedSetpoint+50;//move towards the right
								}
							OldLoadPowerPWM=LoadPowerPWM;
							OldDir=Dir;
						}
					}
				}//end MPPTHillClimb

	}// end 5s scan



	EALLOW;
	CpuTimer1.InterruptCount++;
	EDIS;
}

__interrupt void cpu_timer2_isr(void)
{
   EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
   EDIS;
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
   // 10MHz => 0.1us TBPRD*2=500*2=1000. 10MHz/1000=10KHz

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

   EPwm2Regs.TBPHS.half.TBPHS = 0;           // Phase is 200
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
   // EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        //Sync Flow Through

   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz
   // 10MHz => 0.1us TBPRD*2=500*2=1000. 10MHz/1000=10KHz

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

}

void InitEPwm3Example(void)
{


	   // Setup TBCLK
	   EPwm3Regs.TBPRD = EPWM1_TIMER_TBPRD;           // Set timer period 801 TBCLKs

	   EPwm3Regs.TBCTR = 0x0000;                      // Clear counter

	   // Set Compare values
	   EPwm3Regs.CMPA.half.CMPA = 250; //EPWM2_MIN_CMPA;     // Set compare A value
	   EPwm3Regs.CMPB = 250; //EPWM2_MIN_CMPB;               // Set Compare B value

	   // Setup counter mode
	   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up

	   EPwm3Regs.TBPHS.half.TBPHS = 0;           // Phase is 200
	   EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
	   // EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
	   EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        //Sync Flow Through

	   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
	   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz
	   // 10MHz => 0.1us TBPRD*2=500*2=1000. 10MHz/1000=10KHz

	   // Setup shadowing
	   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
	   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	   // Set actions
	   EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
	   EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

	   EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	   EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

	   // Interrupt where we will change the Compare Values
	   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	   EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	   EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

	   EPwm3Regs.ETSEL.bit.SOCAEN	= 1;		// Enable SOC on A group
	//   EPwm3Regs.ETSEL.bit.SOCASEL	= 4;		// Select SOC from CMPA on upcount
	   EPwm3Regs.ETSEL.bit.SOCASEL	= 2;		// Select SOC from Timebase counter equal to period TBCTR=TBPRD
	   EPwm3Regs.ETPS.bit.SOCAPRD 	= 1;		// Generate pulse on 1st event

}


void InitEPwm4Example(void)
{


   // Setup TBCLK
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
   EPwm4Regs.TBPRD = EPWM1_TIMER_TBPRD;          // Set timer period

   EPwm4Regs.TBCTR = 0x0000;                     // Clear counter
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz
   // 10MHz => 0.1us TBPRD*2=500*2=1000. 10MHz/1000=10KHz

   EPwm4Regs.TBPHS.half.TBPHS = 0;           // Phase is 600
   EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
   // EPwm4Regs.TBCTL.bit.PHSDIR = TB_DOWN;
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

}

void InitEPwm5Example(void)
{


	   // Setup TBCLK
	   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
	   EPwm5Regs.TBPRD = EPWM1_TIMER_TBPRD;          // Set timer period

	   EPwm5Regs.TBCTR = 0x0000;                     // Clear counter
	   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV4;       // Clock ratio to SYSCLKOUT
	   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV2;			// = SYSCLKOUT /(4*2) = 80MHz/8=10MHz
	   // 10MHz => 0.1us TBPRD*2=500*2=1000. 10MHz/1000=10KHz

	   EPwm5Regs.TBPHS.half.TBPHS = 0;           // Phase is 0
	   EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Slave Mode
	   // EPwm5Regs.TBCTL.bit.PHSDIR = TB_UP;
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
}



//---------------------------------------------------------------------------
// InitCpuTimers:
//---------------------------------------------------------------------------
// This function initializes all three CPU timers to a known state.
//
void InitCpuTimers(void)
{
    // CPU Timer 0
	// Initialize address pointers to respective timer registers:
	CpuTimer0.RegsAddr = &CpuTimer0Regs;
	// Initialize timer period to maximum:
	CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;
	// Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer0Regs.TPR.all  = 0;
	CpuTimer0Regs.TPRH.all = 0;
	// Make sure timer is stopped:
	CpuTimer0Regs.TCR.bit.TSS = 1;
	// Reload all counter register with period value:
	CpuTimer0Regs.TCR.bit.TRB = 1;
	// Reset interrupt counters:
	//CpuTimer0.InterruptCount = 0;


// Initialize address pointers to respective timer registers:
	CpuTimer1.RegsAddr = &CpuTimer1Regs;
	CpuTimer2.RegsAddr = &CpuTimer2Regs;
	// Initialize timer period to maximum:
	CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;
	CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
	CpuTimer1Regs.TPR.all  = 0;
	CpuTimer1Regs.TPRH.all = 0;
	CpuTimer2Regs.TPR.all  = 0;
	CpuTimer2Regs.TPRH.all = 0;
    // Make sure timers are stopped:
	CpuTimer1Regs.TCR.bit.TSS = 1;
	CpuTimer2Regs.TCR.bit.TSS = 1;
	// Reload all counter register with period value:
	CpuTimer1Regs.TCR.bit.TRB = 1;
	CpuTimer2Regs.TCR.bit.TRB = 1;
	// Reset interrupt counters:
	//CpuTimer1.InterruptCount = 0;
	//CpuTimer2.InterruptCount = 0;

}

//---------------------------------------------------------------------------
// ConfigCpuTimer:
//---------------------------------------------------------------------------
// This function initializes the selected timer to the period specified
// by the "Freq" and "Period" parameters. The "Freq" is entered as "MHz"
// and the period in "uSeconds". The timer is held in the stopped state
// after configuration.
//
void ConfigCpuTimer(struct CPUTIMER_VARS *Timer, float Freq, float Period)
{
	Uint32 	PeriodInClocks;

	// Initialize timer period:
	Timer->CPUFreqInMHz = Freq;
	Timer->PeriodInUSec = Period;
	PeriodInClocks = (long) (Freq * Period);
	Timer->RegsAddr->PRD.all = PeriodInClocks - 1; // Counter decrements PRD+1 times each period

	// Set pre-scale counter to divide by 1 (SYSCLKOUT):
	Timer->RegsAddr->TPR.all  = 0;
	Timer->RegsAddr->TPRH.all  = 0;

	// Initialize timer control register:
	Timer->RegsAddr->TCR.bit.TSS = 1;      // 1 = Stop timer, 0 = Start/Restart Timer
	Timer->RegsAddr->TCR.bit.TRB = 1;      // 1 = reload timer
	Timer->RegsAddr->TCR.bit.SOFT = 0;
	Timer->RegsAddr->TCR.bit.FREE = 0;     // Timer Free Run Disabled
	Timer->RegsAddr->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt

	// Reset interrupt counter:
	Timer->InterruptCount = 0;

}



//===========================================================================
// No more.
//===========================================================================
