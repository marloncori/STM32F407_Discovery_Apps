/******************************************************************************************
  B e g i n  o f   p r o g r a m  - ultrasonic sensor, leds timer and interrupt
******************************************************************************************/
#include "stm32f4xx.h"
#include <stdio.h>

#define TRIGGER_PIN		GPIO_Pin_6  //PC6
#define ECHO_PIN 		GPIO_Pin_7  //PB7
#define TRIGGER_PORT	GPIOC
#define ECHO_PORT		GPIOB
/******************************************************************************************
 P r i v a t e   v a r i a b l e s
******************************************************************************************/
GPIO_InitTypeDef 		Ultrasonic_Sensor;
GPIO_InitTypeDef 		Leds_Handler;
TIM_TimeBaseInitTypeDef TIM_TimeBase3;
TIM_OCInitTypeDef 		TIM3_OCSetter;
TIM_TimeBaseInitTypeDef TIM_TimeBase4;
TIM_ICInitTypeDef 		TIM4_ICSetter;
NVIC_InitTypeDef		NVIC_Handler;
volatile uint16_t PrescalerValue;
__IO uint16_t Raw_Value  = 0;
__IO uint16_t Distance   = 0;
/******************************************************************************************
 F u n c t i o n   p r o t o t y p e s
******************************************************************************************/
void HCSR04_Trigger_Setup(void);
void HCSR04_Trigger_Channel();
void HCSR04_Echo_Setup(void);
void HCSR04_Echo_Channels(void);
void HCSR04_Begin(void);
void LEDS_Init(void);
/******************************************************************************************
  M a i n  r o u t i n e
*****************************************************************************************/
int main(void)
{
	HCSR04_Trigger_Setup();
	HCSR04_Trigger_Channel();
	HCSR04_Echo_Setup();
	HCSR04_Echo_Channels();
	HCSR04_Begin();
	LEDS_Init();
}
/******************************************************************************************
  P r i v a t e   f u n c t i o n s
*****************************************************************************************/
void HCSR04_Trigger_Setup(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	Ultrasonic_Sensor.GPIO_Pin	 = TRIGGER_PIN;
	Ultrasonic_Sensor.GPIO_Mode  = GPIO_Mode_AF;
	Ultrasonic_Sensor.GPIO_Speed = GPIO_Speed_100MHz;
	Ultrasonic_Sensor.GPIO_OType = GPIO_OType_PP;
	Ultrasonic_Sensor.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(TRIGGER_PORT, &Ultrasonic_Sensor);
	GPIO_PinAFConfig(TRIGGER_PORT, GPIO_PinSource6, GPIO_AF_TIM3);
	//timer 3 setup
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000) -1;
	TIM_TimeBase3.TIM_Period = 0xFFFF; // 65535 - max value
	TIM_TimeBase3.TIM_Prescaler = PrescalerValue; // 1 MHz
	TIM_TimeBase3.TIM_ClockDivision = 0;
	TIM_TimeBase3.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBase3);
}

void HCSR04_Trigger_Channel(void)
{
	TIM3_OCSetter.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM3_OCSetter.TIM_OutputState = TIM_OutputState_Enable;
	TIM3_OCSetter.TIM_Pulse       = 1;
	TIM3_OCSetter.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM3_OCSetter);
	TIM_Cmd(TIM3, ENABLE);
}

void HCSR04_Echo_Setup(void)
{
	Ultrasonic_Sensor.GPIO_Pin	 = ECHO_PIN;
	Ultrasonic_Sensor.GPIO_Mode  = GPIO_Mode_AF;
	Ultrasonic_Sensor.GPIO_Speed = GPIO_Speed_100MHz;
	Ultrasonic_Sensor.GPIO_OType = GPIO_OType_PP;
	Ultrasonic_Sensor.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(ECHO_PORT, &Ultrasonic_Sensor);
	GPIO_PinAFConfig(TRIGGER_PORT, GPIO_PinSource7, GPIO_AF_TIM4);

	NVIC_Handler.NVIC_IRQChannel  = TIM4_IRQn;
	NVIC_Handler.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Handler.NVIC_IRQChannelSubPriority = 1;
	NVIC_Handler.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Handler);
}

void HCSR04_Echo_Channels(void)
{
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 2000000) -1;
	TIM_TimeBase4.TIM_Period = 0xFFFF; // 65535 - max value
	TIM_TimeBase4.TIM_Prescaler = PrescalerValue; // 2 MHz
	TIM_TimeBase4.TIM_ClockDivision = 0;
	TIM_TimeBase4.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBase4);

	//Capture rising mode, it will keep looking for the Echo rising edge
	TIM4_ICSetter.TIM_Channel     = TIM_Channel_1;
	TIM4_ICSetter.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	TIM4_ICSetter.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICSetter.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICSetter.TIM_ICFilter    = 0x0;
	TIM_PWMIConfig(TIM4, &TIM4_ICSetter);

}

void HCSR04_Begin(void)
{
	//Capture falling mode for channel 2
	TIM4_ICSetter.TIM_Channel     = TIM_Channel_2;
	TIM4_ICSetter.TIM_ICPolarity  = TIM_ICPolarity_Falling;
	TIM4_ICSetter.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICSetter.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICSetter.TIM_ICFilter    = 0x0;
	TIM_PWMIConfig(TIM4, &TIM4_ICSetter);

	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
}

void LEDS_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	Leds_Handler.GPIO_Pin	= GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	Leds_Handler.GPIO_Mode  = GPIO_Mode_OUT;
	Leds_Handler.GPIO_Speed = GPIO_Speed_100MHz;
	Leds_Handler.GPIO_OType = GPIO_OType_PP;
	Leds_Handler.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &Leds_Handler);
	GPIO_WriteBit(GPIOD, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6, Bit_RESET);
}
// ISR to read the incoming value so as to get the distance to object
void TIM4_IRQHandler(void)
{
	//for a start clear the capture-compare interrupt pin
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	Raw_Value = TIM_GetCapture2(TIM4);
	printf("========================================================\n");
	printf("         [ HC-SR04 Ultrasonic Sensor Reading ] \n");
	printf("========================================================\n");
	printf("Time duration: %d ms |", Raw_Value);
	//convert the value to distance in centimeters
	Distance = (Raw_Value/116)-5;
	printf("\tDistance: %d cm\n", Distance);
	printf("=========================================================\n");
	if(Distance < 2)
	{
		GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);
	}
	else if(Distance > 2 && Distance < 5)
	{
		GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET);
	}
	else if(Distance > 5 && Distance < 9)
	{
		GPIO_WriteBit(GPIOD, GPIO_Pin_5, Bit_SET);
	}
	else if(Distance > 9 && Distance < 15)
	{
		GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_SET);
	} else {
		GPIO_WriteBit(GPIOD, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6, Bit_RESET);
	}
}
/******************************************************************************************
*  E n d   o f   p r o g r a m
****************************************************************************************/
