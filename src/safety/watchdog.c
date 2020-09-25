/*
 * watchdog.c
 *
 *  Created on: 03-Oct-2019
 *      Author: gkuppusamy
 */
#include <safety/watchdog.h>

static IWDG_HandleTypeDef IWDG_handle;

#define LSI_FREQ_KHZ            (32)
#define BMU_WATCHDOG_PRESCALER  (256)
#define MIN_WATCHDOG_TIMEOUT_MS (50)
#define MAX_WATCHDOG_TIMEOUT_MS (32768)

/**@brief This function initializes the Watchdog timer
 *
 * @param  timeout_ms  [in]  - Watchdog timer period in milli seconds
 * @return bmu_errno_e [out] - "BMU_noerr" , if no error
 *                                     else appropriate error no
 *
 */
bmu_errno_e watchdog_init(uint32_t timeout_ms)
{
	bmu_errno_e errno = BMU_noerr;
	/* check timer period is valid*/
	if((timeout_ms < MIN_WATCHDOG_TIMEOUT_MS) ||
			(timeout_ms > MAX_WATCHDOG_TIMEOUT_MS))
	{
	   errno = BMU_invalid_param_err;
	}
	if(errno == BMU_noerr)
	{
		uint32_t reload = (timeout_ms * LSI_FREQ_KHZ / BMU_WATCHDOG_PRESCALER);

		IWDG_handle.Init.Prescaler = IWDG_PRESCALER_256;
		IWDG_handle.Init.Reload = reload;
		IWDG_handle.Instance = IWDG;

		if(HAL_IWDG_Init(&IWDG_handle) != HAL_OK)
		{
			errno = BMU_watchdog_init_err;
		}
	}
	return(errno);
}

/**@brief This function resets the watchdog timer
 *
 * @param  None
 * @return None
 *
 */
void watchdog_rekick(void)
{
	HAL_IWDG_Refresh(&IWDG_handle);
}
