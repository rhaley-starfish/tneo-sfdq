//
// Created by Russell Haley on 2021-05-22.
//
/*
Copyright (c) 2017-2018 STARFISH PRODUCT ENGINEERING INC.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

//*************************************************
//includes
//*************************************************
//#include <cat24C256.h>
#include <math.h>
#include <ad7172.h>
#include <adcManager.h>
#include <bootloader.h>
#include <byteQ.h>
#include <comms.h>
#include <dac.h>
#include <encoder.h>
#include <error.h>
#include <fastcode.h>
#include <fastcodeUtil.h>
#include <feedbackControl.h>
#include <i2c.h>
#include <pac1710.h>
#include <packetReceiver.h>
#include <portExpanderManager.h>
#include <ports.h>
#include <powerOutputs.h>
#include <pwm.h>
#include <sfdqPackets.h>
#include <sfm3019.h>
#include <sigGen.h>
#include <spi.h>
#include <stdbool.h>
#include <stepperMotor.h>
#include <stm32f4xx.h>
#include <stm32f4xx_iwdg.h>
#include <stm32f4xx_rcc.h>
#include <sys/_stdint.h>
#include <system_stm32f4xx.h>
#include <thermistors.h>
#include <vRailMonitor.h>
#include <backupRam.h>
#include <appData.h>
#include <lsm6ds3.h>
#include <eeprom.h>
#include <statusLeds.h>

#include <max31865.h>
#include <amt22.h>

#include "os-tasks.h"
#include "task_consumer.h"
//***************************************
//From TNEO
//-- system frequency
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define SYS_FREQ           168000000L

//-- kernel ticks (system timer) frequency
#define SYS_TMR_FREQ       1000

//-- system timer period (auto-calculated)
#define SYS_TMR_PERIOD              \
   (SYS_FREQ / SYS_TMR_FREQ)

//-- idle task and interrupt stack sizes, in words. Allocate with convenience
//-- macro TN_STACK_ARR_DEF()
#define IDLE_TASK_STACK_SIZE          (TN_MIN_STACK_SIZE + 32)
#define INTERRUPT_STACK_SIZE          (TN_MIN_STACK_SIZE + 64)
TN_STACK_ARR_DEF(idle_task_stack, IDLE_TASK_STACK_SIZE);
TN_STACK_ARR_DEF(interrupt_stack, INTERRUPT_STACK_SIZE);


//-- task stack size
#define  TASK_PRODUCER_STACK_SIZE      (TN_MIN_STACK_SIZE + 96)
TN_STACK_ARR_DEF(task_producer_stack, TASK_PRODUCER_STACK_SIZE);
//-- highest priority
#define  TASK_PRODUCER_PRIORITY        0

//***************************************

//#include "state_machine_mockup.h"

//*************************************************
//defines
//*************************************************
#define APP_DATA_LENGTH 16
#define SFDQ_BAUD 115200
#define UART_BYTE_BUFFER_LENGTH 1000
#define EEPROM_RAM_SIZE 1556

//*************************************************
//Types
//*************************************************


//*************************************************
//SFDQ Variables
//*************************************************

static struct TN_Task task_producer = {};

static ByteQ hostInQ;	//note: by default a ByteQ is 128 bytes long
static ByteQ hostOutQ;
static uint8_t hostInBuffer[UART_BYTE_BUFFER_LENGTH];
static uint8_t hostOutBuffer[UART_BYTE_BUFFER_LENGTH];

static ByteQ debugInQ;
static ByteQ debugOutQ;
static uint8_t debugInBuffer[UART_BYTE_BUFFER_LENGTH];
static uint8_t debugOutBuffer[UART_BYTE_BUFFER_LENGTH];


static uint32_t wdtTimerReg = 0;
static uint32_t blinkTimerReg = 0;
static uint32_t feedbackTimerReg = 0;
static uint32_t commBlinkTimerReg = 0;
//static uint32_t currentSenseTimerReg = 0;
//
//static uint8_t currentSenseAddress[4] = {0x4c, 0x4d, 0x4e, 0x4f};
//static float currentSenseValue[4];
//static float voltageSenseValue[4];
static uint32_t currentSenseIndex = 0;


static uint32_t m_eepromShadowRam[EEPROM_RAM_SIZE];


//*************************************************
//Application Specific Variables
//*************************************************

//this section is only for use when the SFDQ firmware is customized for a specific project
static bool commOverride = false;//used to make the SFDQ comm protocol over-ride the custom code
static uint32_t commOverrideTimerReg = 0;
static uint32_t commTimeoutTimerReg = 0;

static float m_txAppData[APP_DATA_LENGTH];//data to send to the host
static float m_rxAppData[APP_DATA_LENGTH];//data sent to us from host

//static uint32_t cat24TimerReg = 0;
//static uint32_t cat24DataReg = 0;

static volatile uint32_t run_index = 0;

static struct TN_EventGrp que_procon_events;


//*************************************************
//function prototypes
//*************************************************
static void initClocks(void);
void print(char* msg);


void idle_task_callback (void)
{
}

/**
 * inits all clocks
 * must be run without interrupt running.
 */
void initClocks(void){
    //turn on clocks for GPIO and peripherals
    uint32_t mask = 0;
    mask |= RCC_AHB1Periph_GPIOA;
    mask |= RCC_AHB1Periph_GPIOB;
    mask |= RCC_AHB1Periph_GPIOC;
    mask |= RCC_AHB1Periph_GPIOD;
    mask |= RCC_AHB1Periph_DMA1;
    mask |= RCC_AHB1Periph_DMA2;
    RCC_AHB1PeriphClockCmd(mask, ENABLE);
    mask = 0;
    mask |= RCC_APB1Periph_PWR;
    mask |= RCC_APB1Periph_WWDG;
    mask |= RCC_APB1Periph_TIM3;
    mask |= RCC_APB1Periph_TIM3;
    mask |= RCC_APB1Periph_TIM2;
    mask |= RCC_APB1Periph_TIM7;
    mask |= RCC_APB1Periph_SPI2;
    mask |= RCC_APB1Periph_DAC;
    mask |= RCC_APB1Periph_I2C1;
    mask |= RCC_APB1Periph_I2C2;
    mask |= RCC_APB1Periph_I2C3;

#ifdef SFDQ_HW_REV_X5
    mask |= RCC_APB1Periph_USART3;
#endif
    RCC_APB1PeriphClockCmd(mask, ENABLE);


    mask = 0;
    mask |= RCC_APB2Periph_ADC1;
    mask |= RCC_APB2Periph_SYSCFG;
    mask |= RCC_APB2Periph_USART1;

#ifdef SFDQ_HW_REV_X3
    mask |= RCC_APB2Periph_USART6;
#endif
    RCC_APB2PeriphClockCmd(mask, ENABLE);



    // blocking code - possibly not optimal, however it is only used at program startup, plus the status bits are almost certainly clear at program startup
    while (IWDG->SR & (IWDG_FLAG_PVU | IWDG_FLAG_RVU)){
        // must ensure that (wait until) these flag bits are cleared before we can write to the following registers
    }
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_256);//divides LSI clock down to typical 125Hz
    //IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetReload(32);//cannot be bigger than 0x0fff. 16 sets timeout to 128ms. (32 sets to 256ms)
    //IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
    IWDG_Enable();
}

void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    while(1)
    {
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
//*************************************************
//code
//*************************************************
void hw_init(void)
{

    SystemCoreClockUpdate();

    // DO THIS THIRD
    // (our initialization)
    // Initialize everything else
    initClocks();
    initPins();

    //init new byte queues.
    initByteQ(&hostInQ, hostInBuffer, UART_BYTE_BUFFER_LENGTH);
    initByteQ(&hostOutQ, hostOutBuffer, UART_BYTE_BUFFER_LENGTH);
    initByteQ(&debugInQ, debugInBuffer, UART_BYTE_BUFFER_LENGTH);
    initByteQ(&debugOutQ, debugOutBuffer, UART_BYTE_BUFFER_LENGTH);

    //use these next two lines for the old wiring of uarts.
    //This is NOT compatible with using the bootloader

#ifdef SFDQ_HW_REV_X5


    initComms(USART3_DEV,  &debugInQ, &debugOutQ, 115200, true);
    initComms(USART1_DEV, &hostInQ, &hostOutQ, SFDQ_BAUD, true);



#else
    initComms(USART1_DEV, &debugInQ, &debugOutQ, 115200);
    initComms(USART6_DEV, &hostInQ, &hostOutQ, SFDQ_BAUD);
#endif


    initPacketReciever(&hostInQ);
    initBaudrateUpdater(USART1_DEV);


    initThermistors();
    initSteppers();
    initFeedback();


    spiInit();
    dacInit();

//    if(SET == RCC_GetFlagStatus(RCC_FLAG_IWDGRST)){
//        logError(WATCHDOG_RESET_ERROR, 0);
//    } else {
//        logError(RESET_ERROR, 0);
//    }


#ifdef SFDQ_HW_REV_X5


    powerOutputInit(0, PWM_TIMER3, PWM_CH1);
    powerOutputInit(1, PWM_TIMER3, PWM_CH2);
    powerOutputInit(2, PWM_TIMER3, PWM_CH3);
    powerOutputInit(3, PWM_TIMER3, PWM_CH4);



#else
    powerOutputInit(0, PWM_TIMER3, PWM_CH4);
    powerOutputInit(1, PWM_TIMER3, PWM_CH1);
    powerOutputInit(2, PWM_TIMER2, PWM_CH4);
    powerOutputInit(3, PWM_TIMER3, PWM_CH3);

    setPin(HB_EN_1_PIN, true);
    setPin(HB_EN_2_PIN, true);
    setPin(HB_EN_3_PIN, true);
    setPin(HB_EN_4_PIN, true);
#endif


    sigGenInit();
    i2cInit(I2C1_DEV);
    i2cInit(I2C2_DEV);
    i2cInit(I2C3_DEV);
    initialiseFastCode(20000);
    pac1710Init(I2C1_DEV);
    initEncoder();
    vRailMonitorInit();
    adcManagerInit();

//    volatile uint32_t i = 0; //Slow code
//	logError(TEST_ERROR);


    spiInit(SPI2_DEV);
//	amt22Config(1, SPI2_DEV, GPIO_B10_PIN, true, 42, true, 1, 0);


    setPin(GPIO_LED101_PIN, true);
    setPin(GPIO_LED101_PIN_2, true);

    //setup I2C pins
    setPinDirection(GPIO_B9_PIN, PIN_AF1);
    setPinDirection(GPIO_C12_PIN, PIN_AF1);

}
// Debug print
void print(char* msg){
    if(msg){
        while(*msg){
            putByte(&debugOutQ, *msg);
            msg++;
        }
    }
}


void SysTick_Handler(void){
//    static uint32_t hack = 0;
    dontPutAnyFastCodeBeforeThisFunction();
    fastcode();
//    ++hack;
//    if(hack == 1000) {
//        tn_tick_int_processing();
//        hack = 0;
//    }
    dontPutAnyFastCodeAfterThisFunction();

}

static void appl_init(void)
{

    //-- init common application objects
    os_tasks_init();

    //-- create all the rest application tasks:

    //-- create the consumer task {{{
    {
        task_consumer_create();

        //-- wait until consumer task initialized
        SYSRETVAL_CHECK(
                tn_eventgrp_wait(
                        queue_procon_eventgrp_get(),
                        QUE_PROCON_FLAG__TASK_CONSUMER_INIT,
                        TN_EVENTGRP_WMODE_AND,
                        TN_NULL,
                        TN_WAIT_INFINITE
                )
        );
    }
    // }}}

}

/**
 * This is the specific code to the application project.
 * It will only run when comms is not present.
 */
static void doAppSpecificCode(void){
    // All the Morair-specific slow code lives here.
//    do_hw_mockup_states();
    int i;

    for (i = 0; i < 3/*pins count*/;i++){
        //-- Wait before sending message
        SYSRETVAL_CHECK_TO( tn_task_sleep(2500) );

        enum E_TaskConsPin pin_num = 0;

        //-- determine pin_num
        switch (i){
            case 0:
                pin_num = TASK_CONS_PIN__0;
                break;
            case 1:
                pin_num = TASK_CONS_PIN__1;
                break;
            case 2:
                pin_num = TASK_CONS_PIN__2;
                break;
            default:
                //-- should never be here
                SOFTWARE_BREAK();
                break;
        }

        //-- Send the message to consumer
        if (!task_consumer_msg_send(
                TASK_CONS_CMD__PIN_TOGGLE,
                pin_num
        )
                )
        {
            //-- failed to send the message
        }
    }
}

static void task_producer_body(void *par) {
    //-- in this particular application, producer task is the first application
    //   task that is started, so, we should perform all the app initialization
    //   here, and then start other tasks. All of this is done in the appl_init().
    appl_init();

    //-- cry that producer task has initialized
    SYSRETVAL_CHECK(
            tn_eventgrp_modify(
                    queue_procon_eventgrp_get(),
                    TN_EVENTGRP_OP_SET,
                    QUE_PROCON_FLAG__TASK_PRODUCER_INIT
            )
    );

    //-- at this point, application is completely initialized, and we can
    //   get to business: enter endless loop and repeatedly send
    //   messages to the consumer
    for (;;) {


        configStatusLeds(GPIO_LED101_PIN, GPIO_LED100_PIN);

        ++run_index;
        if (slowTimer(&wdtTimerReg, 1000)) {

            IWDG_ReloadCounter();

        }
        if (slowTimer(&blinkTimerReg, 500000)) {
//            toggleHeartbeatStatusLed();

        }

        if (slowTimer(&feedbackTimerReg, getFeedbackSampleTimeUs())) {
            calcFeedbackOutputs();
        }
        if (slowTimer(&commBlinkTimerReg, 20000)) {
            setCommsStatusLed(false);
        }

        //init dac because there's no harm in it
        dacInit();

        if (decodeSfdqPacket()) {

            //there was a new packet

            if (!wasPacketJustAQuery()) {
                resetSlowTimer(&commOverrideTimerReg);
                commOverride = true;
            }

            resetSlowTimer(&commTimeoutTimerReg);
            resetSlowTimer(&commBlinkTimerReg);
            setCommsStatusLed(true);//turn LED on

            sendSfdqPacket(&hostOutQ);

        }


//if no comms for 5 seconds then make sure we're at 115200 baud
        if (slowTimer(&commTimeoutTimerReg, 5000000)) {
            if (getBaudrateUpdate() != 115200) {
                updateBaudrate(115200);
            }
        }


        if (slowTimer(&commOverrideTimerReg, 1000000)) {
            commOverride = false;
        }

        if (!commOverride) {
            //if comms is not ongoing then control the system autonomously
            doAppSpecificCode();
        } else {
            initAppData(m_txAppData, APP_DATA_LENGTH, m_rxAppData, APP_DATA_LENGTH);

            setTxAppData(0, getFastloopMaxTime() * 1e6);
//			setTxAppData(1, getSfm3019Temperature());
//			setTxAppData(2, getSfm3019Volume());
//			setTxAppData(3, getI2cResetTimeoutCount(I2C1_DEV)+getI2cResetTimeoutCount(I2C2_DEV));
//			setTxAppData(4, getI2cBusyTimeoutCount(I2C1_DEV)+getI2cBusyTimeoutCount(I2C2_DEV));
//			setTxAppData(5, getSfm3019Status());
            setTxAppData(6, M_PI);

        }

//		if(slowTimer(&cat24TimerReg, 100000)){
//			queueCat24ReadWords(I2C2_DEV, 0b10100000, 12, &cat24DataReg, 0);
////			updateTca9534a(I2C1_DEV, 0x39, 0b11010100, 0xff, &tca9534Inputs, &tca9534Status1);
//
//
//
//		}
        portExpanderSlowCode();
//		tca9534SlowCode();
//		cat24C256SlowCode();
        eepromSlowCode();
//		sfm3019SlowCode(I2C1_DEV, 20000, 0.0f, GPIO_A11_PIN);


        ad7172SlowCode(SPI2_DEV, GPIO_B12_PIN);
        lsm6ds3SlowCode(0.5);
        currentSenseIndex = 0;
        adcManagerSlowCode();
        vRailMonitorSlowCode();
        bootloaderSlowCode();
//		ads131a0xConfig(true, SPI2_DEV, GPIO_B12_PIN, GPIO_A11_PIN);
//		ads131a0xControl(1000000);


        max31865SlowCode(0.5);
        amt22SlowCode();

        //check the current sensor. If we get a reading then increment to the next chip
//		if(pac1710SlowCode(currentSenseAddress[currentSenseIndex], &currentSenseTimerReg, &(currentSenseValue[currentSenseIndex]), &(voltageSenseValue[currentSenseIndex]))){
//			++currentSenseIndex;
//			if(currentSenseIndex >= 4){
//				currentSenseIndex = 0;
//			}
//		}

    }
}

void init_task_create(void) {

    SYSRETVAL_CHECK(
            tn_task_create(
                    &task_producer,
                    task_producer_body,
                    TASK_PRODUCER_PRIORITY,
                    task_producer_stack,
                    TASK_PRODUCER_STACK_SIZE,
                    TN_NULL,
                    (TN_TASK_CREATE_OPT_START)
            )
    );

}


int main(void) {

    hw_init();
//	cat24TestData = 0x31415926;
//	queueCat24WriteWords(12, &cat24TestData, &cat24Status);

//	setupEepromAsCat24(m_eepromShadowRam, EEPROM_RAM_SIZE, I2C2_DEV, 0);
    setupEepromAsNothing(EEPROM_RAM_SIZE, m_eepromShadowRam);

    initAppData(m_txAppData, APP_DATA_LENGTH, m_rxAppData, APP_DATA_LENGTH);

    //-- create application events
    //   (see enum E_QueExampleFlag in the header)
    SYSRETVAL_CHECK(tn_eventgrp_create(&que_procon_events, (0)));

    tn_sys_start(
            idle_task_stack,
            IDLE_TASK_STACK_SIZE,
            interrupt_stack,
            INTERRUPT_STACK_SIZE,
            init_task_create,
            idle_task_callback
    );
}
