/**
 *  Copyright (c) 2008-20120 Enphase Energy, Inc. All rights reserved.
 *
 *  @file bmu_init.c
 *  @brief bmu_init .
 *
 *  This is to initialize the BMU peripherals.
 *
 *  @author Enphase Solar Energy Pvt. Ltd.
 *  @author Prajwal Bhagwat
 *  @bug No known bugs.
 */


#include <app/defines.h>
#include <platform/gpio.h>
#include <platform/uart.h>
#include <app/bmu_init.h>
#include <safety/watchdog.h>
#include <safety/stm32f4xx_STLstartup.h>

#define PLLN_FACTOR      192        /* For Clk freq of 96 MHz and external osc = 8 MHz */
#define PLLQ_FACTOR      4          /* For USB clock of 48 MHz*/
#define MHZ_DIVISOR      1000000u

uint32_t bmu_init_status = 0;

static void initialize_hardware(void);
static void systemclock_config(void);

/** @brief BMU initialize.
 *    This function Calls multiple functions related to system initialization
 *	 	after power up. Sets Main oscillator to operate @ 120MHz,
 *	 	second oscillator @ 32.768KHz for RTC,  Initialize pins as
 *	 	GPIOs/ADCs/SPI/CAN/UART/PWM & assigns default states..
 *
 * @param None.
 * @return None
 */

void bmu_init(void)
{
   initialize_hardware();

   __enable_irq();		// Bootloader

   /* GPIO Initialization */
   if(bmu_gpio_init() != BMU_noerr)
   {
	   bmu_init_status |= (uint32_t)(1 << (uint8_t)gpio_bit);
   }

   /* Uart Initialization */
   if(debug_uart_init() != BMU_noerr)
   {
	   bmu_init_status |= (uint32_t)(1 << (uint8_t)uart_bit);
   }

   uart_print("\n\rEncharge BMU Application !!!\r");
   uart_print("\n\rSystem clock: %u MHz", (HAL_RCC_GetHCLKFreq()/MHZ_DIVISOR));

   /*I2C initialization to be added*/

}


static void initialize_hardware(void)
{
  /* Initialize the HAL Library; it must be the first function
  * to be executed before the call of any HAL function.
  */
  HAL_Init();

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  systemclock_config();

  /* Call the CSMSIS system clock routine to store the clock frequency
  * in the SystemCoreClock global RAM location.
  */
  SystemCoreClockUpdate();
}


/**
 * @brief  System Clock Configuration
 * @param  None
 * @retval None
 */
static void  systemclock_config(void)
{

  RCC_OscInitTypeDef osc_init;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
  * device is clocked below the maximum system frequency, to update the
  * voltage scaling value regarding system frequency refer to product
  * datasheet.
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*
   *  Enable HSE Oscillator and activate PLL with HSE as source.
   */
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc_init.HSEState = RCC_HSE_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /*
   *  This assumes the HSE_VALUE is a multiple of 1 MHz.
  */
  osc_init.PLL.PLLM = (HSE_VALUE/MHZ_DIVISOR);
  osc_init.PLL.PLLN = PLLN_FACTOR;
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;   /* 96 MHz */
  osc_init.PLL.PLLQ = PLLQ_FACTOR; /* To make USB work. */
  osc_init.PLL.PLLState = RCC_PLL_ON;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init;
  /*  Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
   *  clocks dividers
   */
  clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;


  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV4;  /* 24 MHz */
  clk_init.APB2CLKDivider = RCC_HCLK_DIV2;  /* 48 MHz */

  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5);
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}
