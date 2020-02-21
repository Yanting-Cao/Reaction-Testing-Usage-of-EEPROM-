/******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

char lcd_buffer[6];    // LCD display buffer

int mode;			//For switch 
int count;		//For counting how many times TIM3_Config() is called

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type

uint16_t Tim4_PrescalerValue, Tim3_PrescalerValue;//timers      3,4           //
TIM_HandleTypeDef  Tim4_Handle, Tim3_Handle;		 //timers       3,4          //
TIM_OC_InitTypeDef Tim4_OCInitStructure;				//timers        3,4         //
void TIM4_Config(void),TIM3_Config(void);			 //timers         3,4        //
void TIM4_OC_Config(void);										//timers          3,4       //
 
RNG_HandleTypeDef Rng_Handle;

__IO uint16_t Tim4_CCR; // the pulse of the TIM4
//for using LCD
Point_Typedef singlePoint;
DoublePoint_Typedef doublePoint;
DigitPosition_Typedef charPosition;
uint32_t rnd; //is not used
uint32_t TickTime; //to store time counting starting from reset

void EXTI0_IRQHandler(void);//is not used
void EXTI1_IRQHandler(void);//is not used

void show(uint32_t n);//input a uint32_t value and output its string from; 
											//more detail will be described below above this actual function

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void GetRNG(void);//output random number and store it into num(the var initiated in the next line); 
									//more detail will be described below above this actual function
uint32_t num;			//store uint32_t random numbers globally
char s[10];				//store converted string (originally uint32_t) globally
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
	/*uint8_t * aCharPtr;
	
	uint8_t aChar='a';*/
	
	singlePoint=POINT_ON ;
	doublePoint=DOUBLEPOINT_OFF;
	charPosition=LCD_DIGIT_POSITION_1;
	HAL_Init();

  /* Configure the system clock to 4 MHz */
  SystemClock_Config();
 
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest
  
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	//BSP_LED_Off(LED4);

	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);
	mode = 0;//the reset states brings
	count = 0;//TIM3 counter is reset to 0 at the beginning
	//BSP_LCD_GLASS_ScrollSentence((uint8_t*) " START PRESS RIGHT TO RESET", 1, 200);
	BSP_LCD_GLASS_DisplayString((uint8_t*)"START");	
	
  //EXTI0_Config();
	
	TIM4_Config();
	
	Tim4_CCR=10000;       //1 s to fire an interrupt.
	
	TIM4_OC_Config();
	
	count = 0;
//******************* use emulated EEPROM ====================================
	//First, Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }
// then can write to or read from the emulated EEPROM
  //EE_WriteVariable(VirtAddVarTab[0], 0xffff);
	//EE_WriteVariable(VirtAddVarTab[0], 0xffff);

	
	
	
//*********************use RNG ================================  

	

  /* Infinite loop */
  while (1)
  {
		

	}
}

/**
  * @brief  Generates Random number
  * @param  None.
  * @retval None.
  */
void GetRNG(){
	Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
//then can use RNG
	/*rnd = 0xff;
	if(HAL_RNG_GenerateRandomNumber(&Rng_Handle,&rnd)!=HAL_OK)
		return 0xfffffffE; this is the test code, which is not used.*/
	
	num = HAL_RNG_GetRandomNumber(&Rng_Handle);
	
	num = (0x0000FFF)&(num); //the inital record going to be stored in the eeprom
													
	HAL_RNG_DeInit(&Rng_Handle);//deinitiate the RNG
}

/**
  * @brief  Converts uint32_t to char
  * @param  n:the uint32_t var to be operated 
  * @retval None.
  */
void show(uint32_t n){
	int i, j;
  i = 0;
  char buf[10];
  do{
        buf[i++] = n % 10 + '0';//next number
  } while ((n /= 10)>0);//delete this number   
  i -= 1;
  for (j = 0; i >= 0; j++, i--)//reverse
        s[j] = buf[i];
  s[j] = '\0'; 
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  // The following clock configuration sets the Clock configuration sets after System reset                
  // It could be avoided but it is kept to illustrate the use of HAL_RCC_OscConfig and HAL_RCC_ClockConfig 
  // and to be eventually adapted to new clock configuration                                               

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}


//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz
void  TIM3_Config(void)
{
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint32_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM2 instance */
  Tim3_Handle.Instance = TIM3; 
	Tim3_Handle.Init.Period = num*5-1; //the period is changed to the amount generated by GetRNG()
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    Error_Handler();
  }
  
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    Error_Handler();}
}

// configure Timer4 base.
void  TIM4_Config(void)
{
  
  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 10000) - 1;
  
  /* Set TIM4 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = 10000-1;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  ///if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
		//Error_Handler();
  //} 
}



void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint16_t temp;
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
							
				if (mode == 1||0)					//first state: when the LED4 is blinking
							{
								BSP_LCD_GLASS_Clear();
								
								if (mode == 1)	 //continue blinking LED4
									BSP_LED_Toggle(LED4);
								
								BSP_LCD_GLASS_DisplayString((uint8_t*)"WAIT"); //show "waiting" on the screen
								GetRNG();//get RNG for TIM3
								TIM3_Config();//Call TIM3 with RNG
								
								mode = 2;//second state
								
							}else if (mode == 2) {
							BSP_LCD_GLASS_Clear();//at first of 2nd state clear the screen 
							BSP_LCD_GLASS_DisplayString((uint8_t*)"EARLY");//before the 3rd state, press select will give an"early"
							mode = 0;																			//and back to the first state
							}else if (mode == 3){												 //the thrid state begins
								BSP_LCD_GLASS_Clear();										//first clear the screen
								BSP_LED_Off(LED5);											 //start blink LED5
								TickTime = HAL_GetTick() - TickTime;		//get the difference between the TickTime, which is the reaction time
								show(TickTime);												 //convert the TickTime to a string
								BSP_LCD_GLASS_DisplayString((uint8_t*)s);	//Display the string TickTime on the screen
								EE_ReadVariable(VirtAddVarTab[0], &temp);//read the existance var in the eeprom
								if (temp>TickTime){                     //if there is less time reaction than the exist one
									EE_WriteVariable(VirtAddVarTab[0], TickTime);//replace it in the eeprom
								}
							mode = 4;																	//mode 4 is the end mode, waiting for reset.
								
							}
						  break;	
			case GPIO_PIN_1:     //left button
						  BSP_LCD_GLASS_Clear();//after press left button first
						
							EE_ReadVariable(VirtAddVarTab[0], &temp);//from the eeprom read the highest record.
							show(temp);//make the record a string
							BSP_LCD_GLASS_DisplayString((uint8_t*)s);//show the record to the screen
							BSP_LED_Off(LED4);//close all the LEDs
							BSP_LED_Off(LED5);
							mode = 4;//end
							//EE_WriteVariable(VirtAddVarTab[0], num); not in use
							//EE_ReadVariable(VirtAddVarTab[0], 0x0000); not in use
							//EXTI1_IRQHandler();//handles EXTI1 interrupt request, not in use
							
							break;
			case GPIO_PIN_2:    //right button						  to play again.
							BSP_LCD_GLASS_Clear();//after press right button, first clear the screen.
							SystemClock_Config();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"RESET");//show "RESET"
							BSP_LED_Off(LED5);//close all the LEDs
							mode = 0;//Reset the state to 0
							//EXTI2_IRQHandler();
							break;
			case GPIO_PIN_3:    //up button							
					
							break;
			
			case GPIO_PIN_5:    //down button						
					
							break;
			default://
						//default
						break;
	  } 
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32lxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
	/*switch (mode){
		 case 0:
			 if ((*htim).Instance==TIM3)
				 BSP_LED_Toggle(LED5);
		 break;
		 case 1:
			 if ((*htim).Instance==TIM3)
				 BSP_LED_Toggle(LED5);
		 break;}*/

	  if (((*htim).Instance==TIM3) && mode == 2){
			count++;//count + 1 when the TIM3 is called
			
			  //BSP_LCD_GLASS_DisplayString((uint8_t*)"Press");
			
			  //clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
			  
			if (count >= 1){//Process continues only if TIM3 is called more than twice
			BSP_LED_On(LED5);//the LED5 turns on until select is pressed
			TickTime = HAL_GetTick();//as long as the LED5 turns on, the timer start to count
			BSP_LCD_GLASS_DisplayString((uint8_t*)"PRESS");//show press
			mode = 3;//change to the thrid state
			count = 0;//count initialized
			}
			__HAL_TIM_SET_COUNTER(htim, 0x00000000);//clear the timer counter at the end of call back to avoid interrupt interval variation!
			
		}
		if (mode ==4)BSP_LED_Off(LED5);//end state close LED5
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	  if (mode == 0) {
			BSP_LED_Toggle(LED4);
			mode = 1;
			//clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
			__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h
		}else if (mode == 1) {
			BSP_LED_Toggle(LED4);
			mode = 0;
			//clear the timer counter at the end of call back to avoid interrupt interval variation!  in stm32l4xx_hal_tim.c, the counter is not cleared after  OC interrupt
			__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this macro is defined in stm32l4xx_hal_tim.h
		}
		
		
}


static void Error_Handler(void)
{
   /* Turn LED4 on */
    BSP_LED_On(LED4);
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
void assert_failed(char *file, uint32_t line)
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
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
