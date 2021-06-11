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
#include "stm32f4xx.h"
#include "ports.h"
#include "os-tasks.h"
#include "task_consumer.h"
#include "task_producer.h"
#include "fastcodeUtil.h"
#include "fastcode-tneo-wrapper.h"
#include "slowcode-tneo-wrapper.h"
#include "slowcode.h"
#include "eeprom.h"
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


static uint32_t m_eepromShadowRam[EEPROM_RAM_SIZE];

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
void hw_init(void) {

    SystemCoreClockUpdate();

    // DO THIS THIRD
    // (our initialization)
    // Initialize everything else
    initClocks();
    initPins();
    init_sfdq();
}

void SysTick_Handler(void){
    dontPutAnyFastCodeBeforeThisFunction();
        tn_tick_int_processing();
    dontPutAnyFastCodeAfterThisFunction();
}
//
//static void appl_init(void)
//{
//
//    //-- init common application objects
//    os_tasks_init();
//
//    //-- create all the rest application tasks:
//
//    //-- create the consumer task {{{
//    {
//        task_consumer_create();
//
//        //-- wait until consumer task initialized
//        SYSRETVAL_CHECK(
//                tn_eventgrp_wait(
//                        queue_procon_eventgrp_get(),
//                        QUE_PROCON_FLAG__TASK_CONSUMER_INIT,
//                        TN_EVENTGRP_WMODE_AND,
//                        TN_NULL,
//                        TN_WAIT_INFINITE
//                )
//        );
//    }
//    // }}}
//
//}

void init_task_create(void) {
    //Start Fast code at high priority
    //start slow code at lower priority
    task_producer_create();
    task_fastcode_create();
    task_slowcode_create();

}


int main(void) {

    hw_init();
//	cat24TestData = 0x31415926;
//	queueCat24WriteWords(12, &cat24TestData, &cat24Status);

//	setupEepromAsCat24(m_eepromShadowRam, EEPROM_RAM_SIZE, I2C2_DEV, 0);
    setupEepromAsNothing(EEPROM_RAM_SIZE, m_eepromShadowRam);

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
