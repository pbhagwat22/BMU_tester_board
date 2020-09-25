/**
  ******************************************************************************
  * @file stm32f4xx_STLstartup.c 
  * @author  MCD Application Team
  * @version  V1.0.0
  * @date  Apr-2013
  * @brief  This file contains all calls to the routines to be
  *                      executed at start.
  ******************************************************************************
  * @copyright
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  */ 


/* Includes ------------------------------------------------------------------*/
#include <safety/stm32f4xx_STLstartup.h>
#include <safety/stm32f4xx_STLparam.h>
#include <stm32f412vx.h>


/** @addtogroup STM32F4xxSelfTestLib_src
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
  typedef enum
  {
    ERROR = 0U,
    SUCCESS = !ERROR
  } ErrorStatus;

/* Private define ------------------------------------------------------------*/
#define DO_NOW
#define SWITCH_ON_PLL
#define FLASH_MEMORY_START_TEST
#define CONTROL_FLOW_CHK
#define RAM_START_TEST
#define RAM_CRC_REF_CHCK
#define FREQ_TEST_START
#define FLASH_MEMORY_RUN_TEST
#define LAST_FLOW_CONTROL_START_TEST

/* Private macro -------------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/
uint32_t tmpCC4_last;	/* Frequency start test: keep last TIM5/Chn4 captured value */
uint32_t RomTest;       /* Used for Flash Runtime CRC check */
uint32_t RamTest;       /* Used for RAM Runtime check */
uint32_t tmpreg;
uint32_t lse_rdy_wait_prd;
extern unsigned int _sCLASS_B_RAM;
extern unsigned int _eCLASS_B_RAM;
extern unsigned int _sCLASS_B_RAM_REV;
extern unsigned int _eCLASS_B_RAM_REV;


  /* Temporary RAM buffer used during transparent RAM run-time test */
  uint32_t RunTimeRamBuf[RT_RAM_BLOCKSIZE] __attribute__((section(".RUN_TIME_RAM_BUF")));

  /* RAM pointer for RAM run-time tests */
   uint32_t *p_RunTimeRamChk        __attribute__((section(".RUN_TIME_RAM_PNT")));
   uint32_t *p_RunTimeRamChkInv     __attribute__((section(".RUN_TIME_RAM_PNT")));

   /* Counter for verifying correct program execution at start */
    uint32_t StartTestResult             __attribute__((section(".CLASS_B_RAM")));
    uint32_t StartTestResultInv          __attribute__((section(".CLASS_B_RAM_REV")));

  /* Counter for verifying correct program execution at start */
   uint32_t CtrlFlowCnt             __attribute__((section(".CLASS_B_RAM")));
   uint32_t CtrlFlowCntInv          __attribute__((section(".CLASS_B_RAM_REV")));

  /* Counter for verifying correct program execution in interrupt */
   uint32_t ISRCtrlFlowCnt          __attribute__((section(".CLASS_B_RAM")));
   uint32_t ISRCtrlFlowCntInv       __attribute__((section(".CLASS_B_RAM_REV")));

  /* LSI period measurement at TIM5 IRQHandler */
   uint32_t PeriodValue           __attribute__((section(".CLASS_B_RAM")));
   uint32_t PeriodValueInv        __attribute__((section(".CLASS_B_RAM_REV")));

  /* Indicates to the main routine a 100ms tick */
   volatile uint32_t LSEPeriodFlag      __attribute__((section(".CLASS_B_RAM")));
   volatile uint32_t LSEPeriodFlagInv   __attribute__((section(".CLASS_B_RAM_REV")));

  /* Stores the Control flow counter from one main loop to the other */
   uint32_t LastCtrlFlowCnt         __attribute__((section(".CLASS_B_RAM")));
   uint32_t LastCtrlFlowCntInv      __attribute__((section(".CLASS_B_RAM_REV")));

  /* Pointer to FLASH for crc16 run-time tests */
   uint8_t *p_RunCrc16Chk           __attribute__((section(".CLASS_B_RAM")));
   uint8_t *p_RunCrc16ChkInv        __attribute__((section(".CLASS_B_RAM_REV")));

  /* Pointer to FLASH for crc32 run-time tests */
   uint32_t *p_RunCrc32Chk          __attribute__((section(".CLASS_B_RAM")));
   uint32_t *p_RunCrc32ChkInv       __attribute__((section(".CLASS_B_RAM_REV")));

/* Reference 32-bit CRC for run-time tests */
   uint32_t RefCrc32                __attribute__((section(".CLASS_B_RAM")));
   uint32_t RefCrc32Inv             __attribute__((section(".CLASS_B_RAM_REV")));

/* Private function prototypes -----------------------------------------------*/
void STL_StartUp(void);
uint32_t CRC_CalcBlockCrc(const uint32_t *p, uint32_t len);
void FailSafePOR(StartTestResult_t test_result);
void TIM5_IRQHandler(void);
void CRC_Init(void);
void CRC_SetIDRegister(uint8_t IDValue);
ErrorStatus STL_FullRamMarchC(void);
uint32_t CRC_GetCRC(void);
RunRamResult_t STL_TranspMarchC(void);
ClassBTestStatus STL_crc32Run(void);
void safety_init(void);
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/**
  * @brief  Contains the Fail Safe routine executed in case of
  *   failure detected during one of the POR self-test routines
  * @param  : None
  * @retval : None
  */
void FailSafePOR(StartTestResult_t test_result)
{
   /* early exit of startup test: record test result in RAM  */
   StartTestResult = (uint32_t)test_result;
   StartTestResultInv = ~StartTestResult;
   /* restart the application */
   GotoCompilerStartUp()   /* reset handler */

   /*  // Used during debug
   SysTick->CTRL= 0u;   // disable SystTick & its interrupt
  while(1)
  {  // Just stay here
  }
  */
}

/******************************************************************************/
/**
  * @brief  Contains the very first test routines executed right after
  *   the reset
  * @param  : None
  *   Flash interface initialized, Systick timer ON (2ms timebase)
  * @retval : None
  */
 void STL_StartUp(void)
{
	 uint32_t TimeOut;
	 ErrorStatus Result = ERROR;
	 StartTestResult_t test_result = START_TEST_PASS;

	/* Select correct vector table address */
	#if BANK_0_SELECTED == 1		// IBL Bootloader
	  SCB->VTOR = 0x08020000;
	#endif
	#if BANK_1_SELECTED == 1
	  SCB->VTOR = 0x08040000;
	#endif
	/* Ensure address change is complete before advancing */
	__DSB();
	__ISB();

	#ifdef SWITCH_ON_PLL
  /*--------------------------------------------------------------------------*/
  /*-------------------- Switch ON PLL for high MHz operation ------------------*/
  /*--------------------------------------------------------------------------*/
  /* No Control flow check here (not safety critical) */
  /* Switch on the PLL to speed-up Flash and RAM tests */
   /* Config system clock */
   RCC->CFGR &= ~RCC_CFGR_HPRE; 		/* Clear HPRE[3:0] bits */
   RCC->CFGR |= RCC_CFGR_HPRE_DIV1; 	/* Set HPRE[3:0] bits according to RCC_SYSCLK value */
   RCC->CFGR &= ~RCC_CFGR_PPRE2; 		/* Clear PPRE2[2:0] bits */
   RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;    /* Set PPRE2[2:0] bits according to RCC_HCLK value */
   RCC->CFGR &= ~RCC_CFGR_PPRE1; 		/*!< PRE2[2:0] bits (APB2 prescaler) */
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;    /*!< HCLK divided by 4 */

   FLASH->ACR |= FLASH_ACR_PRFTEN; 		/* Enable Prefetch Buffer */
   /* RCC PLL Config */
   RCC->CR &= ~RCC_CR_PLLON;   			/* First Disable PLL */
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_4;
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
   RCC->PLLCFGR |= 204u<<6;
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
   RCC->PLLCFGR |= 0x00<<16;
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
   RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
   RCC->PLLCFGR |= 0x05<<24;
   RCC->CR |= RCC_CR_PLLON;   			/* Done, now Enable PLL */

   TimeOut = PLL_LOCK_TIMEOUT;

   do { TimeOut--; }
   while( ( (RCC->CR&RCC_CR_PLLRDY) == 0u)  && (TimeOut != 0u));   /* Wait till PLL is ready */
   if (TimeOut == 0u)   // debugger shows TimeOut is at 29776 here from 30000
   {
     Result = ERROR;     /* Clock switch failure */
   }
   else
   {
	   TimeOut = CLOCK_SWITCH_TIMEOUT;
	   FLASH->ACR &= ~0x0Fu;  					/* Clear */
	   FLASH->ACR |= FLASH_ACR_LATENCY_3WS;  	/* Increase flash latency before switch to fast clock */
	   RCC->CFGR  &= ~RCC_CFGR_SW ; 			/* Clear clock source bits */
	   RCC->CFGR  |= RCC_CFGR_SW_PLL ; 			/* Select PLL as system clock source */
	   /* Wait till PLL is used as system clock source */
	   do{ TimeOut--; }
	   while(((RCC->CFGR & RCC_CFGR_SWS)!= RCC_CFGR_SWS_PLL)  && (TimeOut != 0u));
	   if (TimeOut == 0u) // debugger shows TimeOut is at 999 here from 1000
	   {
	     Result = ERROR;     /* Clock switch failure */
	   }
	   else
	   {
	     Result = SUCCESS;
	   }
   }
  if (Result != SUCCESS)
  {
	  test_result = START_TEST_FAIL_PLL;
	  FailSafePOR(test_result);
  }
#endif // SWITCH_ON_PLL

/* Initialize control flow count */
  CtrlFlowCnt = 0x00000000;
  CtrlFlowCntInv = 0xFFFFFFFF;

#ifdef FLASH_MEMORY_START_TEST
  /*--------------------------------------------------------------------------*/
  /*-------FLASH--------- Invariable memory CRC check ------------------------*/
  /*--------------------------------------------------------------------------*/
  CtrlFlowCnt += CRC32_TEST_CALLER;
  /* Compute the 32-bit crc of the whole Flash by CRC unit */
  CRC_Init();     /* enable crc clock and reset the crc generator */
  CRC_CalcBlockCrc((uint32_t *)ROM_START, (uint32_t)(ROM_SIZEinWORDS-1));
  CtrlFlowCntInv -= CRC32_TEST_CALLER;
  CtrlFlowCnt += CRC_TEST_CALLER;
  /* Checks CRC calculated and Reference CRC are same*/
    if(CRC_GetCRC() != *(uint32_t *)REF_CRC32)
    {
      test_result = START_TEST_FAIL_FLASH;
      FailSafePOR(test_result);
    }
    else
    { /* Test OK */
      /* store the inverted least significant byte of the CRC in the peripheral */ 
      CRC_SetIDRegister(~((uint8_t)(CRC_GetCRC())));
      /* If else statement is not executed, it will be detected by control flow monitoring */
      CtrlFlowCntInv -= CRC_TEST_CALLER;
    }
#endif

#ifdef CONTROL_FLOW_CHK
  /*--------------------------------------------------------------------------*/
  /*   Verify Control flow before RAM init (which clears Ctrl flow counters)  */
  /*--------------------------------------------------------------------------*/
  if (((CtrlFlowCnt ^ CtrlFlowCntInv) != 0xFFFFFFFFuL)
      ||(CtrlFlowCnt != CHECKPOINT1 ))
  {
     test_result = START_TEST_FAIL_CHECKPOINT1;
     FailSafePOR(test_result);
  }
  else
  {
	  /* Control Flow Checkpoint 1 OK */
  }
#endif
  /*--------------------------------------------------------------------------*/
  /* --------RAM---------- Variable memory functional test -------------------*/
  /*--------------------------------------------------------------------------*/
#ifdef RAM_START_TEST
  /* WARNING: Stack is zero-initialized when exiting from this routine */
  if (STL_FullRamMarchC() != SUCCESS)
  {
    test_result = START_TEST_FAIL_RAM;
    FailSafePOR(test_result);
  }
    /* Full RAM Test OK */
    /* Both CtrlFlowCnt and CtrlFlowCntInv are zeroed then re-initialized inside
       the test routine to have inverse values */
#ifdef RAM_CRC_REF_CHCK
  /*--------------------------------------------------------------------------*/
  /*-- Store reference 32-bit CRC in RAM after RAM test (if not corrupted) ---*/
  /*--------------------------------------------------------------------------*/
  if ((CRC->IDR ^ ((uint8_t)CRC->DR)) == 0xFFu)   /* CRC_IDR: Independent data register , CRC_DR: Current CRC value */
  { /* Ref 32-bit CRC OK*/
    RefCrc32 = CRC->DR;
    RefCrc32Inv = ~RefCrc32;
  }
  else
  {
    test_result = START_TEST_FAIL_CRC_REG;
    FailSafePOR(test_result);  // Ref 32-bit CRC corrupted
  }
#endif //RAM_CRC_REF
#endif // RAM_START_TEST

#ifdef FREQ_TEST_START
  /*--------------------------------------------------------------------------*/
  /*----------------------- Clock Frequency Self Test ------------------------*/
  /*--------------------------------------------------------------------------*/
  CtrlFlowCnt += CLOCK_TEST_CALLER;
  ClockStatus CLK_FREQ_TEST_Result = TEST_ONGOING; /* In case of unexpected exit */
  CtrlFlowCnt += CLOCK_TEST_CALLEE;

  /* Start low speed Internal (LSI) oscillator */
  CtrlFlowCnt += LSI_INIT_CALLEE;

  RCC->CSR |= RCC_CSR_LSION;
  TimeOut = LSI_START_TIMEOUT;
  do{ TimeOut--; }
  while(((RCC->CSR  & RCC_CSR_LSIRDY) == 0u) && (TimeOut != 0u));

  if(TimeOut == 0)
  {
	  CLK_FREQ_TEST_Result = LSI_START_FAIL;
	  test_result = START_TEST_FAIL_FREQ_LSI;
  }
  else
  {   CtrlFlowCntInv -= LSI_INIT_CALLEE;
      TimeOut = DBP_REG_RST_TIMEOUT;
      CtrlFlowCnt += DBR_INIT_CALLEE;

	  /* Enable the Power clock*/
	  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	  tmpreg = (RCC->APB1ENR & RCC_APB1ENR_PWREN);

	  /* Enabled access to modify BDCR register*/
	  PWR->CR |= PWR_CR_DBP;

	  do{ TimeOut--; }
	  while(((PWR->CR & PWR_CR_DBP) == 0u) && (TimeOut != 0u));

	  if(TimeOut == 0)
	  {
		  CLK_FREQ_TEST_Result = DBR_RESET_FAIL;
		  test_result = START_TEST_FAIL_FREQ_DBR;
	  }
	  else
	  {   CtrlFlowCntInv -= DBR_INIT_CALLEE;
		  TimeOut = LSE_START_TIMEOUT;
		  CtrlFlowCnt += LSE_INIT_CALLEE;
		  /* Check if LSE is already On*/
		  if((RCC->BDCR & (RCC_BDCR_LSEON | RCC_BDCR_LSERDY )) != (RCC_BDCR_LSEON | RCC_BDCR_LSERDY) )
		  {
			  //Reset the D3 domain
			  RCC->BDCR |= RCC_BDCR_BDRST;
			  __DSB();
			  __ISB();
			  RCC->BDCR &= (~(RCC_BDCR_BDRST));
			  __DSB();
			  __ISB();

			  RCC->BDCR |= RCC_BDCR_LSEON;    			   /* Enable LSE */

			  /* Wait till LSE is ready */
			  do{ TimeOut--;
			      lse_rdy_wait_prd++;}
			  while(((RCC->BDCR & RCC_BDCR_LSERDY) == 0u) && (TimeOut != 0u));
		  }
		  if (TimeOut == 0u)   // debugger shows TimeOut at 1486045 down from 3125000
		  {
			  CLK_FREQ_TEST_Result = LSE_START_FAIL;
			  test_result = START_TEST_FAIL_FREQ_LSE;
		  }
		  else /* Start High-speed external oscillator (HSE) and Clock Security System (CSS) */
		  {
			  CtrlFlowCntInv -= LSE_INIT_CALLEE;
			  TimeOut = HSE_START_TIMEOUT;
			  CtrlFlowCnt += HSE_INIT_CALLEE;
			  RCC->CR |= RCC_CR_HSEON; 						/* Start-up the oscillator (HSE: High-speed External) */
			  /* Wait till HSE is ready */
			  do{ TimeOut--; }
			  while(((RCC->CR & RCC_CR_HSERDY) == 0u) && (TimeOut != 0u));
			  if (TimeOut == 0u)  // debugger shows TimeOut at 16893 down from 31250
			  {
				  CLK_FREQ_TEST_Result = HSE_START_FAIL;
				  test_result = START_TEST_FAIL_FREQ_HSE;
			  }
			  else
			  {
				  RCC->CR |= RCC_CR_CSSON;   				/* Enable the Clock Security System */
				  CtrlFlowCntInv -= HSE_INIT_CALLEE;
				  /*-------------- Reference Measurement ---------------------------------*/
				  TimeOut = CLOCK_SWITCH_TIMEOUT;
				  CtrlFlowCnt += CLOCK_SWITCH_CALLEE;
				  RCC->CFGR &= ~RCC_CFGR_SW;         		/* first clear */
				  RCC->CFGR |= RCC_CFGR_SW_HSE; 			/* Switch the main clock to HSE */
				  do {TimeOut--;}
				  while(((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE)  && (TimeOut != 0u));
				  if (TimeOut == 0u)  // debugger shows TimeOut at 999 down from 1000
				  {
					  CLK_FREQ_TEST_Result = HSI_HSE_SWITCH_FAIL;
					  test_result = START_TEST_FAIL_FREQ_SWITCH;
				  }
				  else
				  {   /* Configure TIM5 to measure LSI period */
					  CtrlFlowCntInv -= CLOCK_SWITCH_CALLEE;
					  /* Configure TIM5 to measure LSI period */
					  RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;  	/* Enable TIM5 clock */
					  /* Connect internally the TIM5_CH4 Input Capture to the LSE clock output */
					  TIM5->OR &= ~TIM_OR_TI4_RMP;			/* Clear TI4_RMP */
					  TIM5->OR |= TIM_OR_TI4_RMP_1;			/* LSE connected to TIM5_CH4 input for calibration */
					  /* Configure TIM5 time base */
					  TIM5->PSC = 0x0000u;					/* Set the Prescaler value */
					  TIM5->EGR |= TIM_EGR_UG;  			/* Set or reset the UG Bit */
					  /* Configure TIM5 time base */
					  TIM5->CR1 &= ~TIM_CR1_DIR;   			/* up-counter */
					  TIM5->CR1 &= ~TIM_CR1_CMS; 			/* edge-aligned */
					  TIM5->CR1 &= ~TIM_CR1_CKD; 			/* no clock division */
					  /* Set the Auto-reload value */
					  TIM5->ARR = 0xFFFF;
					  /* Set the Auto reload value */
					  TIM5->PSC = 0x0000;
					  TIM5->EGR |= TIM_EGR_UG;  				/* PSCReloadMode_Immediate (TODO: THIS IS DONE TWICE)*/
					  /* Configure LSI oscillator into TIM5 channel 4 input  */
					  TIM5->CCER &= ~TIM_CCER_CC4E;   					/* clear CC4E: capture disabled*/
					  /* Select the Input and set the filter */
					  TIM5->CCMR2 &= ~TIM_CCMR2_CC4S;   				/* clear CC4S */
					  TIM5->CCMR2 |= TIM_CCMR2_CC4S_0;  				/* CC4 channel is configured as input, IC4 is mapped on TI4 */
					  TIM5->CCMR2 &= ~TIM_CCMR2_IC4F;  					/* clear IC4F: sets filter to "no filter") */
					  TIM5->CCER  &= ~(TIM_CCER_CC4P | TIM_CCER_CC4NP);  /*  00: non-inverted/rising edge */
					  TIM5->CCER  |= TIM_CCER_CC4E;  					/* Capture enabled */
					  TIM5->CCMR2 |= TIM_CCMR2_IC4PSC; 					/* Sets pre-scalar to DIV8 */
					  /* Enable TIM5 Interrupt channel */
					  NVIC_SetPriority(TIM5_IRQn,0u);    				/* set priority of IRQChannel*/
					  NVIC_EnableIRQ(TIM5_IRQn);   						/* Enable External Interrupt */
					  /* Enable TIM5 counter */
					  TIM5->CR1 |= TIM_CR1_CEN;
					  /* Reset the flags */
					  TIM5->SR = 0u;
					  LSEPeriodFlag = 0u;
					  /* Enable the CC4 Interrupt Request */
					  TIM5->DIER |= TIM_DIER_CC4IE; 				    /*!<Capture/Compare 4 interrupt enable   */
					  __enable_irq();
					  /* Wait for two subsequent LSE periods measured by TIM5 interrupt */
					  while (LSEPeriodFlag == 0u)
					  { }
					  LSEPeriodFlag = 0u;
					  while (LSEPeriodFlag == 0u)
					  { }
					  /*-------------------- HSE measurement check for 8MHz crystal ----------------*/
					  if (PeriodValue < HSE_LimitLow(8000000uL))
					  {
						  CLK_FREQ_TEST_Result = EXT_SOURCE_FAIL;		/* Sub-harmonics: HSE -10% below expected */
						  test_result = START_TEST_FAIL_FREQ_LOW;
					  }
					  else
					  {
						  if (PeriodValue > HSE_LimitHigh(8000000uL))
						  {
							  CLK_FREQ_TEST_Result = EXT_SOURCE_FAIL;		/* Harmonics: HSE +10% above expected */
							  test_result = START_TEST_FAIL_FREQ_HIGH;
						  }
						  else
						  {  /* HSE clock is OK */
							  CLK_FREQ_TEST_Result = FREQ_OK;         	/* Crystal or Resonator started correctly */
						  } /* No harmonics */
					  }  /* No sub-harmonics */
				  }    /* Clock switched correctly */
			  } /* HSE started correctly */
		  }   /* LSE started correctly */
	  }/*Disabled Backup power domain correctly*/
  }/*LSI started Correctly*/

  /* Switch back PLL to be fed by HSI internal clock */
  RCC->CFGR &= ~RCC_CFGR_SW;  /* clear System clock switch: results in RCC_CFGR_SW_HSI */
  CtrlFlowCntInv -= CLOCK_TEST_CALLEE;
  
  switch ( CLK_FREQ_TEST_Result )
  {
    case FREQ_OK:
      /* Clock frequency OK */
      break;

    case LSI_START_FAIL:
      /* LSI start-up failure */
      FailSafePOR(test_result);
      break;

    case DBR_RESET_FAIL:
      /* Disable Backup register reset failure */
      FailSafePOR(test_result);
      break;

    case LSE_START_FAIL:
      /* LSE start-up failure */
      FailSafePOR(test_result);
      break;

    case HSE_START_FAIL:
      /* HSE start-up failure */
      FailSafePOR(test_result);
      break;

    case HSI_HSE_SWITCH_FAIL:
      /* Clock switch failure */
      FailSafePOR(test_result);
      break;

    case EXT_SOURCE_FAIL:
      /* Clock Source failure */
      FailSafePOR(test_result);
      break;

    case TEST_ONGOING:
    default:
      /* Abnormal Clock Test routine termination */
      test_result = START_TEST_FAIL_ABNORMAL_TERMINATION;
      FailSafePOR(test_result);
      break;
  }
  /* Either switch back to HSI or start PLL on HSE asap */
  CtrlFlowCntInv -= CLOCK_TEST_CALLER;
  #endif //FREQ_TEST

#ifdef LAST_FLOW_CONTROL_START_TEST
  /*--------------------------------------------------------------------------*/
  /* -----  Verify Control flow before Starting main program execution ------ */
  /*--------------------------------------------------------------------------*/
   
   if (((CtrlFlowCnt ^ CtrlFlowCntInv) != 0xFFFFFFFFuL)
      ||(CtrlFlowCnt != CHECKPOINT2 ))
   {
     /* Control Flow Error Checkpoint */
     test_result = START_TEST_FAIL_CHECKPOINT2;
     FailSafePOR(test_result);
   }
   /* Control Flow Checkpoint 2 OK */
#endif
   /* exiting startup test: record test result in RAM  */
   StartTestResult = test_result;
   StartTestResultInv = ~StartTestResult;
   /* restart the application */
   GotoCompilerStartUp()   /* reset handler */
}


/******************************************************************************/
/**
  * @brief  This function handles TIM5 global interrupt request.
  * @param  : None
  * @retval : None
  */
void TIM5_IRQHandler(void)
{
  uint32_t tmpCC4_last_cpy;

  if ((TIM5->SR & TIM_SR_CC4IF) != 0u )
  {
    tmpCC4_last_cpy = tmpCC4_last; 	/* store previous captured value */
    tmpCC4_last = TIM5->CCR4;       /* get currently captured value */
    /* The CC4IF flag is already cleared here be reading CCR4 register */

    /* overwrite results only in case the data is required */
    if (LSEPeriodFlag == 0u)
    {
      /* take correct measurement only */
      if ((TIM5->SR & TIM_SR_CC4OF) == 0u)
      {
        PeriodValue = tmpCC4_last - tmpCC4_last_cpy;    /* Compute period length */
        PeriodValueInv = ~PeriodValue;                  /* Record inverser */
        LSEPeriodFlag = 0xAAAAAAAAu;                    /* Set Flag tested */
        LSEPeriodFlagInv = 0x55555555u;                 /* Record inverser */
      }
      else
      {
        TIM5->SR &= (uint16_t)(~TIM_SR_CC4OF);          /* ignore computation in case of IC overflow */
      }
      /* ignore computation in case data is not required */
    }
  }
}

/**
  * @brief  This function verifies that RAM is functional,
  *   using the March C- algorithm.
  * @param  : None
  *   at the exception of CtrlFlowCntInv, set to 0xFFFFFFFF.
  * @retval : ErrorStatus = (ERROR, SUCCESS)
  */
ErrorStatus STL_FullRamMarchC(void)
{
  ErrorStatus Result = SUCCESS;
  uint32_t *p;       /* RAM pointer */

  uint32_t *ra= __builtin_return_address(0u); /* save return address (as it will be destroyed) */

  /* CtrlFlowCnt not used at function entry, since it will be cleared by the
     routine. CtrlFlowCntInv is written at the function exit */

   /* ---------------------------- STEP 1 ----------------------------------- */
   /* Write background with addresses increasing */
   for ( p = RAM_START; p <= RAM_END; ++p )
   {
      /* Scrambling not important when there's no consecutive verify and write */
      *p = BCKGRND;
   }

   /* ---------------------------- STEP 2 ----------------------------------- */
   /* Verify background and write inverted background with addresses increasing */
   for ( p = RAM_START; p <= RAM_END; ++p )
   {
      if ( *p != BCKGRND )
         {
            Result = ERROR;
         }
      *p = INV_BCKGRND;
   }

   /* ---------------------------- STEP 3 ----------------------------------- */
   /* Verify inverted background and write background with addresses increasing */
   for ( p = RAM_START; p <= RAM_END; ++p )
   {
      if ( *p != INV_BCKGRND )
         {
            Result = ERROR;
         }
      *p = BCKGRND;
   }

   /* ---------------------------- STEP 4 ----------------------------------- */
   /* Verify background and write inverted background with addresses decreasing */
   for ( p = RAM_END; p >= RAM_START; --p )
   {
      if ( *p != BCKGRND )
         {
            Result = ERROR;
         }
      *p = INV_BCKGRND;
   }

   /* ---------------------------- STEP 5 ----------------------------------- */
   /* Verify inverted background and write background with addresses decreasing */
   for ( p = RAM_END; p >= RAM_START; --p )
   {
      if ( *p != INV_BCKGRND )
         {
            Result = ERROR;
         }
      *p = BCKGRND;
   }

   /* ---------------------------- STEP 6 ----------------------------------- */
   /* Verify background with addresses increasing */
   for ( p = RAM_START; p <= RAM_END; p++ )
   {
      if (*p != BCKGRND)
      {
         Result = ERROR;    /* No need to take into account scrambling here */
      }
   }

  /* ==============================================================================*/

   CtrlFlowCntInv = 0xFFFFFFFFuL;

  /* Restore destroyed return address back into the stack (all the content is destroyed).
     Next line of code supposes the {r4-r10,pc} only was saved into stack for Keil
     so their restored values are not valid:
     => optiomizations at caller must be switched off as caller cannot relay on r4-r7 values!!! */
	*(uint32_t*)(__get_MSP() + 4u) = (uint32_t)ra;
	return (Result);
}

/**
  * @brief  Initializes CRC peripheral (enable its clock and reset CRC)
  * @param  : None
  * @retval : None
  */
void CRC_Init(void)
{
  CtrlFlowCnt += CRC32_INIT_CALLEE;  	/* This is for control flow monitoring */
  RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN; 	/* Enable CRC module clock */
  CRC->CR = CRC_CR_RESET;  				/* Reset CRC generator */
  CtrlFlowCntInv -= CRC32_INIT_CALLEE;
}

/**
  * @brief  Returns the current CRC value.
  * @param  None
  * @retval 32-bit CRC
  */
uint32_t CRC_GetCRC(void)
{
  return (CRC->DR);
}

/**
  * @brief  Stores a 8-bit data in the Independent Data(ID) register.
  * @param  IDValue: 8-bit value to be stored in the ID register
  * @retval None
  */
void CRC_SetIDRegister(uint8_t IDValue)
{
  CRC->IDR = IDValue;
}

/**
  * @brief  Inializes the pointers to the Flash memory required during
  *   run-time
  * @param  : None
  * @retval : None
  */
void STL_FlashCrc32Init(void)
{
  p_RunCrc32Chk = (uint32_t*)ROM_START;
  p_RunCrc32ChkInv = ((uint32_t *)(uint32_t)(~(uint32_t)(ROM_START)));
  CRC_Init(); /* Reset the CRC generator */
}

/**
  * @brief  Computes the 16-bit CRC of a given part of the memory (words
  *   by words).
  * @param p: points to the first memory byte to be taken into account
  * @param len: length (in bytes) of the memory to be computed
  * @retval : 32-bit CRC using the 0x4C11DB7 polynomial
  */
uint32_t CRC_CalcBlockCrc(const uint32_t *p, uint32_t len)
{
	/* This is for control flow monitoring */
	CtrlFlowCnt += CRC32_TEST_CALLEE;
	do  /* Load memory content into the CRC generator data register */
	{
		CRC->DR = *p++;
	}
	while (len--);
	CtrlFlowCntInv -= CRC32_TEST_CALLEE;
	return (CRC->DR);
}

/**
  * @brief  Computes the crc in multiple steps and compare it with the
  *   ref value when the whole memory has been tested
  * @param  : None
  * @retval : ClassBTestStatus (TEST_RUNNING, CLASS_B_DATA_FAIL,
  *   TEST_FAILURE, TEST_OK)
  */
ClassBTestStatus STL_crc32Run(void)
{
    ClassBTestStatus Result = CTRL_FLW_ERROR; /* In case of abnormal function exit*/
    CtrlFlowCnt += CRC32_RUN_TEST_CALLEE;

	if ((((uint32_t)p_RunCrc32Chk) ^ ((uint32_t)p_RunCrc32ChkInv)) == 0xFFFFFFFFuL)  /* Check safety var integrity */
	{
		if (p_RunCrc32Chk < (uint32_t *)ROM_END)
		{
		  CRC_CalcBlockCrc((const uint32_t *)p_RunCrc32Chk, (uint32_t)(FLASH_BLOCK_WORDS-1));
		  p_RunCrc32Chk += FLASH_BLOCK_WORDS;     							/* Increment pointer to next block */
		  p_RunCrc32ChkInv = ((uint32_t *)~((uint32_t)p_RunCrc32Chk)); 		/* Update safety var */
		  Result = TEST_RUNNING;
		}
		else
		{
		  if ((RefCrc32 ^ RefCrc32Inv) == 0xFFFFFFFFuL) 	/* Check safety var integrity */
		  {
			  if(CRC_GetCRC() == (uint32_t)(RefCrc32)) 		/* Check new CRC to last CRC */
			  {
				  Result = TEST_OK;
			  }
			  else
			  {
				  Result = TEST_FAILURE;  						/* New CRC does not match last CRC */
			  }
			  STL_FlashCrc32Init(); 						/* Prepare next test (or redo it if this one failed) */
		  }
		  else /* Class B error on RefCrc32 */
		  {
			Result = CLASS_B_DATA_FAIL;
		  }
		}
	}
	else  /* Class B error p_RunCrc32Chk */
	{
		Result = CLASS_B_DATA_FAIL;
	}
	CtrlFlowCntInv -= CRC32_RUN_TEST_CALLEE;
	return (Result);
	}

RunFlashResult_t RunTime_Flash_CRC(void)
{
    /*----------------------------------------------------------------------*/
	 /*------------------ Invariable memory CRC check -----------------------*/
	 /*----------------------------------------------------------------------*/
	 CtrlFlowCnt += FLASH_TEST_CALLER;
	 RunFlashResult_t test_result;

	 RomTest = STL_crc32Run();  /* Requires the control flow check to be modified */
	 switch ( RomTest )
	 {
	   case TEST_RUNNING:
		   CtrlFlowCntInv -= FLASH_TEST_CALLER;
		   test_result = FLASH_RUN_TEST_ONGOING;
		 break;

	   case TEST_OK:
		 CtrlFlowCntInv -= FLASH_TEST_CALLER;
		 test_result = FLASH_RUN_TEST_PASS;
		 break;

	   case TEST_FAILURE:
		   test_result = FLASH_RUN_TEST_FAIL_CRC_CHECK;
		 break;
	   case CLASS_B_DATA_FAIL:
		   test_result = FLASH_RUN_TEST_FAIL_SAF_VAR;
		 break;
	   default:
		   test_result = FLASH_RUN_TEST_ABNORMAL_TERMINATION;
		 break;
	 }
	 return test_result;
}

/** @brief UL 1998 variable initialize.
 *    This function sets the initial values to all variables
 *	 	used for UL 1998 safety related checks.
 *
 * @param None.
 * @return None.
 */

void safety_init(void)
{
	/* Initialize CLASS_B_RAM */
	uint8_t *start_address;
	uint8_t *end_address;

 	start_address 	= (uint8_t*)&_sCLASS_B_RAM;
 	end_address 	= (uint8_t*)&_eCLASS_B_RAM;
	while(start_address < end_address)
	{
		*start_address = 0x00;
		++start_address;
	}

 	start_address 	= (uint8_t*)&_sCLASS_B_RAM_REV;
 	end_address 	= (uint8_t*)&_eCLASS_B_RAM_REV;
	while(start_address < end_address)
	{
		*start_address = 0x00;
		++start_address;
	}
}

/**
  * @brief  Initializes the pointer to the RAM for the run-time
  *   transparent functional test.
  * @param  : None
  * @retval : None
  */
void STL_TranspMarchCInit(void)
{
  /* start address of the test has to be aligned to 16 address range */
   p_RunTimeRamChk = (uint32_t *)((uint32_t)CLASS_B_START & 0xFFFFFFFCuL);
   p_RunTimeRamChkInv = (uint32_t *)(uint32_t)(~(uint32_t)p_RunTimeRamChk);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief  This function verifies that 6 words of RAM are functional,
  *   overlapping) using the March C- algorithm.
  * @param  : None
  * @retval : ClassBTestStatus (TEST_RUNNING, CLASS_B_DATA_FAIL,
  *   TEST_FAILURE, TEST_OK)
  */
RunRamResult_t STL_TranspMarchC(void)
{
  ClassBTestStatus Result = TEST_RUNNING;
  RunRamResult_t test_result = RAM_RUN_TEST_ONGOING;
  uint32_t i;        /* Index for RAM physical addressing */

  ISRCtrlFlowCnt += RAM_MARCHC_ISR_CALLEE;
  /* Check Class B var integrity */
  if ((((uint32_t)p_RunTimeRamChk) ^ ((uint32_t)p_RunTimeRamChkInv)) == 0xFFFFFFFFuL)
  {
    //if (p_RunTimeRamChk >= CLASS_B_END)
	if (p_RunTimeRamChk >= RAM_END)   // full ram check
    {
      /*------------- Apply March C- to the RAM Buffer itself --------------- */
      p_RunTimeRamChk = &RunTimeRamBuf[0];
      p_RunTimeRamChkInv = (uint32_t*)(~(uint32_t)(&RunTimeRamBuf[0]));

      /*---------------------------- STEP 1 --------------------------------- */
      /* Write background with addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        *(p_RunTimeRamChk + i) = BCKGRND;
      }

      /*---------------------------- STEP 2 --------------------------------- */
      /* Verify background and write inverted background addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        if ( *(p_RunTimeRamChk + i) != BCKGRND )
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i) = INV_BCKGRND;
      }

      /*---------------------------- STEP 3 --------------------------------- */
      /* Verify inverted background and write background addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        if ( *(p_RunTimeRamChk + i) != INV_BCKGRND )
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i) = BCKGRND;
      }

      /*---------------------------- STEP 4 --------------------------------- */
      /* Verify background and write inverted background addresses decreasing */
      for( i = RT_RAM_BLOCKSIZE; i > 0u ; --i )
      {
        if ( *(p_RunTimeRamChk + i - 1u) != BCKGRND)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 1u) = INV_BCKGRND;
      }

      /*---------------------------- STEP 5 --------------------------------- */
      /* Verify inverted background and write background addresses decreasing */
      for( i = RT_RAM_BLOCKSIZE; i > 0u ; --i )
      {
        if ( *(p_RunTimeRamChk + i - 1u) != INV_BCKGRND)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 1u) = BCKGRND;
      }

      /*---------------------------- STEP 6 --------------------------------- */
      /* Verify background with addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        if ( *(p_RunTimeRamChk + i) != BCKGRND )
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
      }

      /* Check again Class B variable integrity */
      if ((((uint32_t)p_RunTimeRamChk) ^ ((uint32_t)p_RunTimeRamChkInv)) == 0xFFFFFFFFuL)
      {
        /* Prepare next Tranparent RAM test from the beginning of Class A area */
        p_RunTimeRamChk = CLASS_B_START;
        p_RunTimeRamChkInv = ((uint32_t *)~((uint32_t)CLASS_B_START));

        if (Result == TEST_RUNNING)
        {
          Result = TEST_OK; /* Means all selected variable memory was scanned */
          test_result = RAM_RUN_TEST_PASS;
        }
        else  /* Buffer is not functional */
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
      }
      else  /* Class B error on p_RunTimeRamChk */
      {
        Result = CLASS_B_DATA_FAIL;
        test_result = RAM_RUN_TEST_FAIL_SAF_VAR;
      }

    } /* ------------------ End of Buffer Self-check ------------------------ */
    else
    { /* ------------------ Regular memory Self-check ----------------------- */
      /*---------------------------- STEP 1 --------------------------------- */
      /* Save the content of the 6 words to be tested and start MarchC -
         Write background with addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        RunTimeRamBuf[i] = *(p_RunTimeRamChk + i - 1u);
        *(p_RunTimeRamChk + i - 1u) = BCKGRND;
      }

      /*---------------------------- STEP 2 --------------------------------- */
      /* Verify background and write inverted background addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        if ( *(p_RunTimeRamChk + i - 1u) != BCKGRND )
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 1u) = INV_BCKGRND;
      }

      /*---------------------------- STEP 3 --------------------------------- */
      /* Verify inverted background and write background addresses increasing */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        if ( *(p_RunTimeRamChk + i - 1u) != INV_BCKGRND)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 1u) = BCKGRND;
      }

      /*---------------------------- STEP 4 --------------------------------- */
      /* Verify background and write inverted background addresses decreasing */
      for( i = RT_RAM_BLOCKSIZE; i > 0u; --i )
      {
        if ( *(p_RunTimeRamChk + i - 2u) != BCKGRND)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 2u) = INV_BCKGRND;
      }

      /*---------------------------- STEP 5 --------------------------------- */
      /* Verify inverted background and write background addresses decreasing */
      for( i = RT_RAM_BLOCKSIZE; i > 0u; --i )
      {
        if ( *(p_RunTimeRamChk + i - 2u) != INV_BCKGRND)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 2u) = BCKGRND;
      }

      /*---------------------------- STEP 6 --------------------------------- */
      /* Verify background with addresses increasing */
      /* and restore the content of the 6 tested words */
      for( i = 0u; i < RT_RAM_BLOCKSIZE; ++i )
      {
        if ( *(p_RunTimeRamChk + i - 1u) != BCKGRND)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;
        }
        *(p_RunTimeRamChk + i - 1u) = RunTimeRamBuf[i];
      }

      /* Check again Class B variable integrity */
      if ((((uint32_t)p_RunTimeRamChk) ^ ((uint32_t)p_RunTimeRamChkInv)) == 0xFFFFFFFFuL)
      {
        /* Prepare next Row Tranparent RAM test */
        p_RunTimeRamChk += RT_RAM_BLOCKSIZE - (2u * RT_RAM_BLOCK_OVERLAP);
        p_RunTimeRamChkInv = (uint32_t *)(uint32_t)(~(uint32_t)p_RunTimeRamChk);

        if (Result != TEST_RUNNING)
        {
          Result = TEST_FAILURE; test_result = RAM_RUN_TEST_FAIL;  /* Word line under test was not functional */
        }
        else
        {
          /* Do nothing: keep Result as TEST_RUNNING */
        }
      }
      else  /* Class B error on p_RunTimeRamChk when exiting the function*/
      {
        Result = CLASS_B_DATA_FAIL;
        test_result = RAM_RUN_TEST_FAIL_SAF_VAR;
      }
    } /* --------------- End of Regular memory Self-check --------------------- */
  }
  else  /* Class B error on p_RunTimeRamChk when entering the function*/
  {
    Result = CLASS_B_DATA_FAIL;
    test_result = RAM_RUN_TEST_FAIL_SAF_VAR;
  }

  ISRCtrlFlowCntInv -= RAM_MARCHC_ISR_CALLEE;

  return (test_result);

}

RunRamResult_t RunTimeRAMTest(void)
{
	RunRamResult_t RamTest;
	RamTest = STL_TranspMarchC();
	return RamTest;
}

uint32_t get_startup_test_result(void)
{
	return(StartTestResult);
}

uint32_t get_startup_test_result_inv(void)
{
	return(StartTestResultInv);
}

uint32_t get_clk_period(void)
{
	return(PeriodValue);
}

uint32_t get_lse_wait_period(void)
{
	return(lse_rdy_wait_prd);
}

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

