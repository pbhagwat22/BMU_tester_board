/**
 *  Copyright (c) 2008-2018 Enphase Energy, Inc. All rights reserved.
 *
 *  @file uart.c
 *  @brief UART handler.
 *
 *  This file initializes the uart module
 *
 *  @author Enphase Solar Energy Pvt. Ltd.
 *  @author Gokulnath Kuppusamy.
 *  @bug No known bugs.
 */

#include <platform/uart.h>
#include <app/defines.h>
#include <string.h>

void USART1_IRQHandler(void);

static uint8_t buffer[DEBUG_UART_BUFSIZE];
static uint8_t rec_buffer[2][DEBUG_UART_BUFSIZE];
static UART_HandleTypeDef uart_handle;
static uint16_t rx_tail_idx = 0;
static uint8_t rec_byte, rx_buff_idx;
static uint8_t cmd_idx , num_cmd , cmd_len;


#if ENABLE_UART_PRINT
/** @brief print_msg
 *         This function transfer the messages to UART
 *         if debug prints are enabled.
 *
 * @param  None
 * @return None
 */
void print_msg(const char * format, ... )
{
	va_list ap;
	int n;
	/* will overwrite the buffer otherwise, (needs a timeout and error handling*/
	while (uart_handle.gState != HAL_UART_STATE_READY);
	va_start(ap, format);
	n = vsnprintf ((char*)buffer, DEBUG_UART_BUFSIZE, format, ap);
	va_end(ap);
	HAL_UART_Transmit_IT(&uart_handle, (uint8_t*)buffer, (uint16_t)n);
}
#else
/** @brief This function will not transfer the messages to UART if debug prints are disabled.
 * This function will not transfer the messages to UART if debug prints are disabled
 * @param None.
 * @return None
 */
    void print_msg( const char* format, ... )
    {
    }
#endif

static inline void add_to_rx_buffer(uint8_t rx_data)
{

  /* Find the index in RX buffer where the next byte goes to */
  if (rx_data!='\n')
  {
	  rec_buffer[rx_buff_idx][rx_tail_idx] = rx_data;
	  rx_tail_idx=rx_tail_idx+1;

  }
  else
  {
	  rec_buffer[rx_buff_idx][rx_tail_idx] = '\0';
	  cmd_idx=rx_buff_idx;
	  cmd_len=rx_tail_idx;
	  num_cmd++;
	  rx_buff_idx=!rx_buff_idx;
	  rx_tail_idx=0;

  }

}
/** @brief USART1_IRQHandler
 *          USART1_IRQHandler Transfers the configured channel details
 * @param  None.
 * @return None
 */

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&uart_handle);

    HAL_UART_Receive_IT(&uart_handle, (uint8_t*)&rec_byte, 1);

    add_to_rx_buffer((uint8_t) rec_byte);

}



/**
 * @brief  Retrieves read_len number of bytes from UART RX buffer,
 *         if available. If read_len is more than available bytes,
 *         then only available number of bytes are copied.
 *         This is a non-blocking call. SHOULD not call from
 *         interrupt context.
 * @param  none
 * @retval number of bytes copied.
 */
bool UART_get_rx_cmd(uint8_t * read_buf)
{
	if (num_cmd>0)
	{
		memcpy(rec_buffer[cmd_idx], read_buf, cmd_len);
		num_cmd--;
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief  Flushes the RX buffer.
 * @param  none
 * @retval none
 */
void UART_flush_rx(void)
{

}

/** @brief DebugUARTInit
 *         This function configures the Debug UART port
 *         with the following parameters.
 *           Baud rate 11500,
 *           Data bits 8,
 *           Parity    No,
 *           Stop bits 1
 * @param  None.
 * @return bmu_errno_e  [out] 'BMU_noerr', if no error;
 *                            otherwise gives appropriate error.
 */

bmu_errno_e debug_uart_init(void)
{
	bmu_errno_e errno = BMU_noerr;

  __USART1_CLK_ENABLE();

  uart_handle.Instance        = USART1;
  uart_handle.Init.BaudRate   = BMU_UART_BAUD_RATE;
  uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
  uart_handle.Init.StopBits   = UART_STOPBITS_1;
  uart_handle.Init.Parity     = UART_PARITY_NONE;
  uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  uart_handle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&uart_handle) == HAL_OK)
  {
	  HAL_NVIC_SetPriority(USART1_IRQn, UART_INTRPT_PRIORITY,
			                             UART_INTRPT_SUB_PRIOTITY);
	  HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else
  {
	  errno = BMU_uart_init_err;
  }

  return errno;
}
