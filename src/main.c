//
// main.c
//

#include <stdio.h>
//#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "tn.h"
#include "queue_example.h"

//-- system frequency
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define SYS_FREQ           168000000L

//-- kernel ticks (system timer) frequency
#define SYS_TMR_FREQ       1000

//-- system timer period (auto-calculated)
#define SYS_TMR_PERIOD              \
   (SYS_FREQ / SYS_TMR_FREQ)



//-- idle task stack size, in words
#define IDLE_TASK_STACK_SIZE          (TN_MIN_STACK_SIZE + 32)

//-- interrupt stack size, in words
#define INTERRUPT_STACK_SIZE          (TN_MIN_STACK_SIZE + 64)

//-- stack sizes of user tasks
#define TASK_A_STK_SIZE    (TN_MIN_STACK_SIZE + 96)
#define TASK_B_STK_SIZE    (TN_MIN_STACK_SIZE + 96)
#define TASK_C_STK_SIZE    (TN_MIN_STACK_SIZE + 96)

//-- user task priorities
#define TASK_A_PRIORITY    7
#define TASK_B_PRIORITY    6
#define TASK_C_PRIORITY    5

JOYState_TypeDef JoyState = JOY_NONE;

//
//#define LED0 (1 << 12)
//#define LED1 (1 << 13)
//#define LED2 (1 << 14)
//#define LED3 (1 << 15)


/*******************************************************************************
 *    DATA
 ******************************************************************************/

//-- Allocate arrays for stacks: stack for idle task
//   and for interrupts are the requirement of the kernel;
//   others are application-dependent.
//
//   We use convenience macro TN_STACK_ARR_DEF() for that.

TN_STACK_ARR_DEF(idle_task_stack, IDLE_TASK_STACK_SIZE);
TN_STACK_ARR_DEF(interrupt_stack, INTERRUPT_STACK_SIZE);

TN_STACK_ARR_DEF(task_a_stack, TASK_A_STK_SIZE);
TN_STACK_ARR_DEF(task_b_stack, TASK_B_STK_SIZE);
TN_STACK_ARR_DEF(task_c_stack, TASK_C_STK_SIZE);



//-- task structures

struct TN_Task task_a;
struct TN_Task task_b;
struct TN_Task task_c;



/*******************************************************************************
 *    ISRs
 ******************************************************************************/

/**
 * system timer ISR
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
    tn_tick_int_processing();
}
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}


/*******************************************************************************
 *    FUNCTIONS
 ******************************************************************************/


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

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows:
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    HAL_StatusTypeDef ret = HAL_OK;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 360;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 6;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Activate the OverDrive to reach the 180 MHz Frequency */
    ret = HAL_PWREx_EnableOverDrive();
    if(ret != HAL_OK)
    {
        while(1) { ; }
    }
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

}

/*
 * Needed for STM debug printf
 */
int fputc(int c, FILE *stream)
{
    return ITM_SendChar(c);
}

void appl_init(void);

void task_a_body(void *par)
{
    //-- this is a first created application task, so it needs to perform
    //   all the application initialization.
    appl_init();

    //-- and then, let's get to the primary job of the task
    //   (job for which task was created at all)
    for(;;)
    {
//        if (GPIOD->ODR & LED1){
//            GPIOD->BSRRH = LED1;
//        } else {
//            GPIOD->BSRRL = LED1;
//        }
//        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        printf("task a\n");
//        BSP_LED_On()
        tn_task_sleep(500);

    }
}

void task_b_body(void *par)
{
    for(;;)
    {

//        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
////            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//            BSP_LED_Off(LED2);
//            tn_task_sleep(50);
//        } else {
////            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//            BSP_LED_On(LED2);
//            tn_task_sleep(50);
//        }
        uint32_t bstate = BSP_PB_GetState(BUTTON_USER);
        if(bstate == 1)
        {
            for(int i = 0; i < 3; i++) {
                BSP_LED_On(LED2);
                tn_task_sleep(50);
                BSP_LED_Off(LED2);
                tn_task_sleep(50);
            }
        }
//        JoyState = BSP_JOY_GetState();
//        //        Blink_LED(JoyState);
//        if(JoyState == JOY_LEFT)
//        {
//            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//            tn_task_sleep(100);
//            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//        }
        HAL_Delay(6);
        tn_task_sleep(1000);
    }
}

void task_c_body(void *par)
{
    for(;;)
    {
        for(int i = 0; i < 3; i++) {
            BSP_LED_On(LED2);
            tn_task_sleep(50);
            BSP_LED_Off(LED2);
            tn_task_sleep(50);
        }
        //printf("task c\n");
        tn_task_sleep(1303);
    }
}

/**
 * Hardware init: called from main() with interrupts disabled
 */
void hw_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    //-- init system timer
    SysTick_Config(SYS_TMR_PERIOD);
    /* Configure the system clock to 180 MHz */
    SystemClock_Config();

    /* GPIO Ports Clock Enable */
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    __HAL_RCC_GPIOH_CLK_ENABLE();

    LED2_GPIO_CLK_ENABLE(); //__HAL_RCC_GPIOA_CLK_ENABLE()
    USER_BUTTON_GPIO_CLK_ENABLE(); //__HAL_RCC_GPIOC_CLK_ENABLE()
//    /*##-2- Configure PA05 IO in output push-pull mode to drive external LED ###*/
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    Button_TypeDef button = (Button_TypeDef)USER_BUTTON_PIN;
    ButtonMode_TypeDef mode =  BUTTON_MODE_GPIO;
    BSP_PB_Init(button, mode);

    /* STM32F4xx HAL library initialization:
     - Configure the Flash prefetch, instruction and Data caches
     - Configure the Systick to generate an interrupt each 1 msec
     - Set NVIC Group Priority to 4
     - Global MSP (MCU Support Package) initialization
   */
    HAL_Init();
}

/**
 * Application init: called from the first created application task
 */
void appl_init(void)
{

    //-- configure LED port pins
//    {
//        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enable Port D clock
//
//        //-- Set pin 12, 13, 14, 15 as general purpose output mode (pull-push)
//        GPIOD->MODER |= (0
//                         | GPIO_MODER_MODER12_0
//                         | GPIO_MODER_MODER13_0
//                         | GPIO_MODER_MODER14_0
//                         | GPIO_MODER_MODER15_0
//        ) ;
//
//        // GPIOD->OTYPER |= 0; //-- No need to change - use pull-push output
//
//        GPIOD->OSPEEDR |= (0
//                           | GPIO_OSPEEDER_OSPEEDR12 // 100MHz operations
//                           | GPIO_OSPEEDER_OSPEEDR13
//                           | GPIO_OSPEEDER_OSPEEDR14
//                           | GPIO_OSPEEDER_OSPEEDR15
//        );
//
//        GPIOD->PUPDR = 0; // No pull up, no pull down
//    }

    //-- initialize various on-board peripherals, such as
    //   flash memory, displays, etc.
    //   (in this sample project there's nothing to init)

    //-- initialize various program modules
    //   (in this sample project there's nothing to init)


    //-- create all the rest application tasks
    tn_task_create(
            &task_b,
            task_b_body,
            TASK_B_PRIORITY,
            task_b_stack,
            TASK_B_STK_SIZE,
            NULL,
            (TN_TASK_CREATE_OPT_START)
    );

    tn_task_create(
            &task_c,
            task_c_body,
            TASK_C_PRIORITY,
            task_c_stack,
            TASK_C_STK_SIZE,
            NULL,
            (TN_TASK_CREATE_OPT_START)
    );
}

//-- idle callback that is called periodically from idle task
void idle_task_callback (void)
{
}

////-- create first application task(s)
//void init_task_create(void)
//{
//    //-- task A performs complete application initialization,
//    //   it's the first created application task
//    tn_task_create(
//            &task_a,                   //-- task structure
//            task_a_body,               //-- task body function
//            TASK_A_PRIORITY,           //-- task priority
//            task_a_stack,              //-- task stack
//            TASK_A_STK_SIZE,           //-- task stack size (in words)
//            NULL,                      //-- task function parameter
//            TN_TASK_CREATE_OPT_START   //-- creation option
//    );
//
//}



int main(void)
{

    //-- unconditionally disable interrupts
    tn_arch_int_dis();

    //-- init hardware
    hw_init();

    //-- call to tn_sys_start() never returns
    tn_sys_start(
            idle_task_stack,
            IDLE_TASK_STACK_SIZE,
            interrupt_stack,
            INTERRUPT_STACK_SIZE,
            init_task_create,
            idle_task_callback
    );

    //-- unreachable
    return 1;

}

#pragma clang diagnostic pop