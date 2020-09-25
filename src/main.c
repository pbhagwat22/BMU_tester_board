/**
 *  Copyright (c) 2008-2018 Enphase Energy, Inc. All rights reserved.
 *
 *  @file main.c
 *  @brief Main handler.
 *
 *  This file calls the initialization  sequence and application logic.
 *
 *  @author Enphase Solar Energy Pvt. Ltd.
 *  @author Gokulnath Kuppusamy.
 *  @bug No known bugs.
 */


#include <app/defines.h>
#include <platform/uart.h>
#include <app/bmu_errno.h>
#include <app/bmu_init.h>


uint8_t read_buf[10];


void main(void)
{


   /* initializing the BMU */
   bmu_init();


   /* if no error during initialization do application logic */
   if(bmu_init_status == 0)
   {
	  uart_print("\n\rApplication Initialization Successful...\r");
//	  uart_print("\n\rStarting the BMU Application v%d.%d.%d.....\r",
//			                 SOFTWARE_MAJOR_VERSION,
//							 SOFTWARE_MINOR_VERSION,
//							 SOFTWARE_BUILD_NUMBER);
   }
   else/* Error during Initialization */
   {

      //fault_handler(INIT_FAIL,BMU_noerr,bmu_init_status);
	  uart_print("\n\rApplication Initialization Failed with status %ld !!! \n\r",
			                                                  bmu_init_status);
   }

   while(1)
   {

	   if (UART_get_rx_cmd(read_buf))
	   {
		   uart_print("\n %s \n",read_buf);
	   }





   }


}/* end of main loop */

#pragma GCC diagnostic pop
