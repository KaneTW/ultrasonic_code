//#############################################################################
//
// FILE:   epwm_ex9_dma.c
//
// TITLE:  ePWM Using DMA.
//
//! \addtogroup driver_example_list
//! <h1>ePWM DMA</h1>
//!
//! This example configures ePWM1 and DMA as follows:
//!  - ePWM1 is set up to generate PWM waveforms
//!  - DMA5 is set up to update the CMPAHR, CMPA, CMPBHR and CMPB every period
//!    with the next value in the configuration array. This allows the user to
//!    create a DMA enabled fifo for all the CMPx and CMPxHR registers to
//!    generate unconventional PWM waveforms.
//!  - DMA6 is set up to update the TBPHSHR, TBPHS, TBPRDHR and TBPRD every
//!    period with the next value in the configuration array.
//!  - Other registers such as AQCTL can be controlled through the DMA as well
//!    by following the same procedure. (Not used in this example)
//!
//! \b External \b Connections \n
//! - GPIO0 EPWM1A
//! - GPIO1 EPWM1B
//!
//! \b Watch \b Variables \n
//! - None.
//
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.07.00.00 $
// $Release Date: Sun Sep 29 07:34:54 CDT 2019 $
// $Copyright:
// Copyright (C) 2013-2019 Texas Instruments Incorporated - http://www.ti.com/
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
//#############################################################################



//
// Included Files
//
#include "driverlib.h"
#include "device.h"

#include "she_pwm_coefficients.h"
#include "SFO_V8.h"

//
// Defines
//
#define EPWM_TIMER_TBPRD 2500
#define TRANSFER    (2*ANGLES)
#define BURST       6              // 4 words per transfer
#define PI          3.14159265

//
// Globals
//

uint16_t compareConfigs[TRANSFER*BURST] = {
//  TBPRDHR ,  TBRPRD  ,  CMPAHR  ,  CMPA    ,  CMPBHR  ,  CMPB   ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
    0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff  ,  0xffff ,
};


uint16_t status;
int MEP_ScaleFactor;
volatile uint32_t ePWM[3] = {0, EPWM1_BASE, EPWM2_BASE};

// Place buffers in GSRAM
#pragma DATA_SECTION(compareConfigs,    "ramgs0");

//
// Function Prototypes
//
void initDMA(void);
void initEPWM(uint32_t base);

__interrupt void dmaCh5ISR(void);
__interrupt void dmaCh6ISR(void);
__interrupt void epwm1ISR(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    while(status == SFO_INCOMPLETE)
    {
        status = SFO();
    }

    //
    // Assign the interrupt service routines to ePWM interrupts
    //
    Interrupt_register(INT_EPWM1, &epwm1ISR);
    Interrupt_register(INT_DMA_CH5, &dmaCh5ISR);

    //
    // Configure GPIO0/1 as ePWM1A/1B pins respectively
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);


    //
    // CHANGE XBAR inputs from using GPIO0
    // if EPWM SYNCIN is enabled, EXTSYNCIN1 and EXTSYNCIN2 will use
    // GPIO0 (which is the output of EPWM1).
    // Pick and unused GPIO
    //
    XBAR_setInputPin(XBAR_INPUT5, 50);

    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Select DMA as the secondary master for the Peripherals
    //
    SysCtl_selectSecMaster(1, 1);

    initDMA();
    initEPWM(EPWM1_BASE);


    float frequency = 40000;
    int ampIdx = 99;

    float fundamentalAngularPeriod = 50e6 / (2 * PI * frequency);

    const float* selected = phaseAngles[ampIdx];

    int i;
    for (i = 0; i < ANGLES; i ++) {
        float angle = selected[i*2] * fundamentalAngularPeriod;
        uint16_t angleInt = (uint16_t) angle;

        float period = selected[i*2 + 1] * fundamentalAngularPeriod;
        uint16_t periodInt = (uint16_t) period;

        compareConfigs[i*BURST + 1] = periodInt;
        compareConfigs[i*BURST + 3] = angleInt;
        compareConfigs[(i + ANGLES)*BURST + 1] = periodInt;
        compareConfigs[(i + ANGLES)*BURST + 5] = angleInt;

        angle = (angle - angleInt)*65536;
        angleInt = (uint16_t) angle;

        period = (period - periodInt)*65536;
        periodInt = (uint16_t) period;

        compareConfigs[i*BURST + 0] = periodInt;
        compareConfigs[i*BURST + 2] = angleInt;
        compareConfigs[(i + ANGLES)*BURST + 0] = periodInt;
        compareConfigs[(i + ANGLES)*BURST + 4] = angleInt;

    }

   //
   // Enable sync and clock to PWM
   //
   SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);


   // Enable ePWM interrupts
   //
   Interrupt_enable(INT_EPWM1);
   Interrupt_enable(INT_DMA_CH5);

   //
   // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
   //
   EINT;
   ERTM;

   EALLOW;

    DMA_startChannel(DMA_CH5_BASE);

    for(;;)
    {
        //
        // Call the scale factor optimizer lib function SFO()
        // periodically to track for any change due to temp/voltage.
        // This function generates MEP_ScaleFactor by running the
        // MEP calibration module in the HRPWM logic. This scale
        // factor can be used for all HRPWM channels. The SFO()
        // function also updates the HRMSTEP register with the
        // scale factor value.
        //
        status = SFO(); // in background, MEP calibration module
                        // continuously updates MEP_ScaleFactor
    }
}


//
// DMA setup channels.
//
void initDMA()
{
    //
    // Initialize DMA
    //
    DMA_initController();

    //
    // DMA CH5
    //
    DMA_configAddresses(DMA_CH5_BASE, (uint16_t *)(EPWM1_BASE + HRPWM_O_TBPRDHR),
                        compareConfigs);
    DMA_configBurst(DMA_CH5_BASE, BURST, 1, 1);
    DMA_configTransfer(DMA_CH5_BASE, TRANSFER, 1, 1-BURST);
    DMA_configMode(DMA_CH5_BASE, DMA_TRIGGER_EPWM1SOCA, DMA_CFG_ONESHOT_DISABLE |
                   DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT);

    //
    // Configure DMA Ch5 interrupts
    //
    DMA_setInterruptMode(DMA_CH5_BASE, DMA_INT_AT_END);
    DMA_enableInterrupt(DMA_CH5_BASE);
    DMA_enableTrigger(DMA_CH5_BASE);
}

//
// DMA Channel 5 ISR
//
__interrupt void dmaCh5ISR(void)
{

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
    return;
}

//
// DMA Channel 6 ISR
//
__interrupt void dmaCh6ISR(void)
{

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
    return;
}


//
// epwm1ISR - ePWM 1 ISR
//
__interrupt void epwm1ISR(void)
{
    //
    // Un-comment below to check the status of each register after CTR=0
    //
    //ESTOP0;

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}


void initEPWM(uint32_t base)
{
    EPWM_setEmulationMode(base, EPWM_EMULATION_STOP_AFTER_FULL_CYCLE);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(base, EPWM_TIMER_TBPRD);
    EPWM_setPhaseShift(base, 0U);
    EPWM_setTimeBaseCounter(base, 0U);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(base,
                                EPWM_COUNTER_COMPARE_A,
                                EPWM_TIMER_TBPRD/2);
    EPWM_setCounterCompareValue(base,
                                EPWM_COUNTER_COMPARE_B,
                                EPWM_TIMER_TBPRD/2);

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(base);

    //
    // Set up shadowing
    //
    //EPWM_setCounterCompareShadowLoadMode(base,
    //                                     EPWM_COUNTER_COMPARE_A,
    //                                     EPWM_COMP_LOAD_ON_CNTR_ZERO);
    //EPWM_setCounterCompareShadowLoadMode(base,
    //                                     EPWM_COUNTER_COMPARE_B,
    //                                     EPWM_COMP_LOAD_ON_CNTR_ZERO);

    EPWM_disableCounterCompareShadowLoadMode(base,  EPWM_COUNTER_COMPARE_A);
    EPWM_disableCounterCompareShadowLoadMode(base,  EPWM_COUNTER_COMPARE_B);

    //
    // Set actions
    //

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);


    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_TOGGLE,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_TOGGLE,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    //HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_A, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_A, HRPWM_MEP_DUTY_PERIOD_CTRL);


    //HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_FALLING_EDGE);
    HRPWM_setMEPEdgeSelect(base, HRPWM_CHANNEL_B, HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE);
    HRPWM_setMEPControlMode(base, HRPWM_CHANNEL_B, HRPWM_MEP_DUTY_PERIOD_CTRL);


    //
    // Enable auto-conversion logic.
    //
    HRPWM_enableAutoConversion(base);
    HRPWM_enablePeriodControl(base);


    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 1st event
    //
    EPWM_setInterruptSource(base, EPWM_INT_TBCTR_U_CMPA);
    EPWM_enableInterrupt(base);
    EPWM_setInterruptEventCount(base, 1U);

    EPWM_enableADCTrigger(base, EPWM_SOC_A);
    EPWM_setADCTriggerSource(base,
                             EPWM_SOC_A,
                             EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(base,
                                    EPWM_SOC_A,
                                       1);
    EPWM_clearADCTriggerFlag(base,
                             EPWM_SOC_A);

}

