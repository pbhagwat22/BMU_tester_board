/**
 *  Copyright (c) 2008-2018 Enphase Energy, Inc. All rights reserved.
 *
 *  @file gpio.c
 *  @brief GPIO handler.
 *
 *  This contains GPIO configuration and initialization
 *
 *  @author Enphase Solar Energy Pvt. Ltd.
 *  @author Gokulnath Kuppusamy.
 *  @bug No known bugs.
 */

#include <app/defines.h>
#include <app/pin.h>
#include <platform/gpio.h>


/* Local function declaration */
static void uart_gpio_init(void);
static void app_output_gpio_init(void);
static void i2c_gpio_init(void);


/** @brief bmu_gpio_init
 *         This function Initialize the GPIO pins.
 * @param  None.
 * @return bmu_errno_e  [out] 'BMU_noerr', if no error;
 *                            otherwise gives appropriate error.
 */

bmu_errno_e bmu_gpio_init(void)
{
	bmu_errno_e errno = BMU_noerr;

	/* Enable GPIO Peripheral clock for ports A-E */

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	__GPIOE_CLK_ENABLE();

   /* Initializing GPIO based on functions */
	app_output_gpio_init();
	uart_gpio_init();
	i2c_gpio_init();


	return errno;
}

/** @brief bmu_gpio_write
 *         This function writes to GPIO output
 * @param  bmu_gpio_out_pins_e  [in] - GPIO pin name
 * @param  bmu_gpio_out_value_e [in] - Value to be written
 * @return bmu_errno_e          [out] 'BMU_noerr', if no error;
 *                                     otherwise gives appropriate error.
 */

bmu_errno_e bmu_gpio_write(bmu_gpio_out_pins_e pin_name,
                           bmu_gpio_out_value_e value)
{
  GPIO_PinState gpio_value;
  bmu_errno_e errno = BMU_noerr;

  if( value == HIGH)
  {
    gpio_value = GPIO_PIN_SET;
  }
  else
  {
    gpio_value = GPIO_PIN_RESET;
  }
  /* Based on Pin name appropriate Pin is SET or RESET based on input value */
  switch (pin_name)
  {
    case BMU_DAC1_CH1:
      HAL_GPIO_WritePin(BMU_DAC_PORT,BMU_DAC1_1CH_RST,gpio_value);
      break;
    case BMU_DAC1_CH2 :
      HAL_GPIO_WritePin(BMU_DAC_PORT,BMU_DAC1_2CH_RST,gpio_value);
      break;

    case BMU_DAC2_CH1 :
      HAL_GPIO_WritePin(BMU_DAC_PORT,BMU_DAC2_1CH_RST,gpio_value);
      break;

    case BMU_DAC2_CH2:
      HAL_GPIO_WritePin(BMU_DAC_PORT,BMU_DAC2_2CH_RST,gpio_value);
      break;

    default:
      errno = BMU_invalid_param_err;
      break;
  }
  return errno;
}


/** @brief uart_gpio_init
 *         This function Initialize the UART GPIO pins.
 * @param  None.
 * @return None
 */

static void uart_gpio_init(void)
{
	GPIO_InitTypeDef gpio_init;

  /* Uart pin configuration */
	gpio_init.Mode      = GPIO_MODE_AF_PP;
	gpio_init.Pull      = GPIO_NOPULL;
	gpio_init.Speed     = GPIO_SPEED_HIGH;
	gpio_init.Alternate = GPIO_AF7_USART2;
	gpio_init.Pin       = BMU_UART1_TX | BMU_UART1_RX;

  /* Port A Pin 9 configured as Uart_tx and
   * Port A pin 10 configured as Uart_rx
   */
	HAL_GPIO_Init(BMU_UART1_PORT, &gpio_init);
}

static void i2c_gpio_init(void)
{
	GPIO_InitTypeDef gpio_init;

  /* i2c pin configuration */
	gpio_init.Mode      = GPIO_MODE_AF_PP;
	gpio_init.Pull      = GPIO_NOPULL;
	gpio_init.Speed     = GPIO_SPEED_HIGH;

	gpio_init.Alternate = GPIO_AF4_I2C1;
	gpio_init.Pin       = BMU_I2C1_SCL | BMU_I2C1_SDA;
	HAL_GPIO_Init(BMU_I2C1_PORT, &gpio_init);

	gpio_init.Alternate = GPIO_AF4_I2C3;
	gpio_init.Pin       = BMU_I2C3_SCL;
	HAL_GPIO_Init(BMU_I2C3_SCL_PORT, &gpio_init);
	gpio_init.Pin       = BMU_I2C3_SDA;
	HAL_GPIO_Init(BMU_I2C3_SDA_PORT, &gpio_init);


}

/** @brief app_output_gpio_init
 *         This function Initialize the Application GPIO output pins
 * @param  None.
 * @return None
 */
static void app_output_gpio_init(void)
{
#if 0
  GPIO_InitTypeDef gpio_init;

	/*Configure Pin in OUTPUT mode, PULL DOWN and HIGH (50 MHz) speed */
	gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull  = GPIO_PULLDOWN;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;

	/* configuring all pins to be output and pulled low
	 */

	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOA, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOB, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOC, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOD, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOE, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOF, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
	HAL_GPIO_Init(GPIOG, &gpio_init);
	gpio_init.Pin   = GPIO_PIN_All;
#endif

}





