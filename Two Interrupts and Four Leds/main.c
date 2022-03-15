/******************************************************************************************
  B e g i n  o f   p r o g r a m  - four leds, a push button, an IR sensor and interrupts
******************************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
//#include "stm32f4xx_exti.h"
//#include "stm32f4xx_syscfg.h"
//#include "misc.h"

/******************************************************************************************
  P r i v a t e   v a r i a b l e s
******************************************************************************************/
GPIO_InitTypeDef GPIO_Initializer;
TIM_TimeBaseInitTypeDef Timer3_Handler;
volatile uint16_t PrescalerValue;
/******************************************************************************************
  F u n c t i o n   p r o t o t y p e s
******************************************************************************************/
void Push_Button_Setup(GPIO_InitTypeDef GPIO_InitStruct);
void IR_Sensor_Init(GPIO_InitTypeDef GPIO_InitStruct);
void Two_Leds_Begin(GPIO_InitTypeDef GPIO_InitStruct);
void Blink_Two_Leds(TIM_TimeBaseInitTypeDef htim3);
void Button_Led_Setup(GPIO_InitTypeDef GPIO_InitStruct);
void IRSensor_Led_Init(GPIO_InitTypeDef GPIO_InitStruct);
void Delay(__IO uint32_t nCount);
/******************************************************************************************
  M a i n   r o u t i n e
******************************************************************************************/
int main(void)
{
  SystemInit();

  Push_Button_Setup(GPIO_Initializer);
  IR_Sensor_Init(GPIO_Initializer);

  Two_Leds_Begin(GPIO_Initializer);
  Blink_Two_Leds(Timer3_Handler);

}
/******************************************************************************************
  P r i v a t e   f u n c t i o n
******************************************************************************************/
void Push_Button_Setup(GPIO_InitTypeDef GPIO_InitStruct)
{
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOD */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	  /* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* Tell system that you will use PD0 for EXTI_Line0 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

	/* PD0 is connected to EXTI_Line0 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

    /* Add Interrupt Request (IRQ) vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

void IR_Sensor_Init(GPIO_InitTypeDef GPIO_InitStruct)
{
	/* Set variables used */
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* Enable clock for GPIOB */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable clock for SYSCFG */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Set pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Tell system that you will use PB12 for EXTI_Line12 */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
	/* PB12 is connected to EXTI_Line12 */
	EXTI_InitStruct.EXTI_Line = EXTI_Line12;
	/* Enable interrupt */
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	/* Interrupt mode */
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	/* Triggers on rising and falling edge */
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	/* Add to EXTI */
	EXTI_Init(&EXTI_InitStruct);

	/* Add IRQ vector to NVIC */
	/* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	/* Set priority */
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	/* Set sub priority */
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	/* Enable interrupt */
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	/* Add to NVIC */
	NVIC_Init(&NVIC_InitStruct);
}

void Two_Leds_Begin(GPIO_InitTypeDef GPIO_InitStruct)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

	Button_Led_Setup(GPIO_InitStruct);
	IRSensor_Led_Init(GPIO_InitStruct);
}

void Blink_Two_Leds(TIM_TimeBaseInitTypeDef htim3)
{
	PrescalerValue = (uint16_t)((SystemCoreClock/2)/13107)-1;
	htim3.TIM_Period 			= 0xFFFF; // counting up to 65535, 5 seconds
	htim3.TIM_Prescaler 		= PrescalerValue;
	htim3.TIM_ClockDivision 	= 0;
	htim3.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &htim3);

	TIM_OCInitTypeDef OCInit;
	OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	OCInit.TIM_OutputState  = TIM_OutputState_Enable;
	OCInit.TIM_Pulse 		= 13170; //Hz
	OCInit.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &OCInit);  // pin -> PC6, High for 1 sec, low for 4 sec

	OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	OCInit.TIM_OutputState  = TIM_OutputState_Enable;
	OCInit.TIM_Pulse 		= 26214; //Hz
	OCInit.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &OCInit); // pin -> PC7 high for 2 sec, low for 3 sec

	TIM_Cmd(TIM3, ENABLE);
}

void Button_Led_Setup(GPIO_InitTypeDef GPIO_InitStruct)
{
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_WriteBit(GPIOA, GPIO_Pin_14, Bit_RESET);
}

void IRSensor_Led_Init(GPIO_InitTypeDef GPIO_InitStruct)
{
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_RESET);
}

void Delay(__IO uint32_t nCount)
{
	while(nCount--){}
}
/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void)
{
	uint8_t times;
	/* Make sure that interrupt flag is set */
    if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Do your stuff when PD0 is changed */
        for(times = 0; times < 5; times++)
        {
        	GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_SET);
        	Delay(1680000);
        	GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_RESET);
        	Delay(1680000);
        }
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void)
{
	uint8_t times;
    /* Make sure that interrupt flag is set */
    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
        /* Do your stuff when PB12 is changed */
        for(times = 0; times < 10; times++)
         {
        	GPIO_WriteBit(GPIOA, GPIO_Pin_14, Bit_SET);
         	Delay(840000);
        	GPIO_WriteBit(GPIOA, GPIO_Pin_14, Bit_RESET);
         	Delay(840000);
         }
        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
/******************************************************************************************
  E n d   o f   p r o g r a m
******************************************************************************************/
