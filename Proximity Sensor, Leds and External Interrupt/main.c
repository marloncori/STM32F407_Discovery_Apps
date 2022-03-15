#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

#define Yellow_Led GPIO_Pin_2
#define Green_Led GPIO_Pin_1
#define White_Led GPIO_Pin_4
#define Blue_Led GPIO_Pin_6

#define IR_Sensor GPIO_Pin_2
#define DELAY_TIME 16800000
#define DELAY_TIME2 3380000

GPIO_InitTypeDef GPIO_InitElement;
EXTI_InitTypeDef EXTI_IRSensor;
NVIC_InitTypeDef NVIC_IRSensor;

void STM32_Enable_Clocks(void);
void GPIO_Init_Leds(void);
void GPIO_Init_IRSensor(void);
void EXTI_Init_Line2(void);
void NVIC_Init_EXTI_Channel2(void);
void Blink_Two_Leds(void);
void Delay_uS(__IO uint32_t time);
void Blink_Alternate(void);

int main(void)
{
	STM32_Enable_Clocks();
	GPIO_Init_Leds();
	GPIO_Init_IRSensor();
	EXTI_Init_Line2();
	NVIC_Init_EXTI_Channel2();

	while (1)
	{
		Blink_Two_Leds();
	}
}

void STM32_Enable_Clocks(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
}

void GPIO_Init_Leds(void)
{
	  GPIO_InitElement.GPIO_Pin 	= Yellow_Led | Green_Led | Blue_Led | White_Led;
	  GPIO_InitElement.GPIO_Mode 	= GPIO_Mode_OUT;
	  GPIO_InitElement.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitElement.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	  GPIO_InitElement.GPIO_Speed	= GPIO_Speed_50MHz;
	  GPIO_Init(GPIOD, &GPIO_InitElement);
}

void GPIO_Init_IRSensor(void)
{
	  GPIO_InitElement.GPIO_Pin 	= IR_Sensor;
	  GPIO_InitElement.GPIO_Mode 	= GPIO_Mode_IN;
	  GPIO_InitElement.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitElement.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	  GPIO_InitElement.GPIO_Speed	= GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitElement);
}

void EXTI_Init_Line2(void)
{
   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
   EXTI_IRSensor.EXTI_Line = EXTI_Line2;
   EXTI_IRSensor.EXTI_Mode = EXTI_Mode_Interrupt;
   EXTI_IRSensor.EXTI_Trigger = EXTI_Trigger_Falling;
   EXTI_IRSensor.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_IRSensor);
}

void NVIC_Init_EXTI_Channel2(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_IRSensor.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_IRSensor.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_IRSensor.NVIC_IRQChannelSubPriority = 0;
	NVIC_IRSensor.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_IRSensor);
}

void Blink_Two_Leds(void)
{
	GPIO_WriteBit(GPIOD, Yellow_Led, Bit_SET);
	GPIO_WriteBit(GPIOD, Green_Led, Bit_RESET);
	Delay_uS(DELAY_TIME);
	GPIO_WriteBit(GPIOD, Yellow_Led, Bit_RESET);
	GPIO_WriteBit(GPIOD, Green_Led, Bit_SET);
	Delay_uS(DELAY_TIME);
}

void Blink_Alternate(void)
{
	GPIO_WriteBit(GPIOD, White_Led, Bit_SET);
	Delay_uS(DELAY_TIME2);
	GPIO_WriteBit(GPIOD, Blue_Led, Bit_SET);
	Delay_uS(DELAY_TIME2);
	GPIO_WriteBit(GPIOD, White_Led, Bit_RESET);
	Delay_uS(DELAY_TIME2);
	GPIO_WriteBit(GPIOD, Blue_Led, Bit_RESET);
	Delay_uS(DELAY_TIME2);
}

void Delay_uS(__IO uint32_t time)
{
	while(time--){}
}

void EXTI_IRQHandler(void)
{
	uint32_t counter = 0;
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		do{
			counter++;
			Blink_Alternate();
		}while(counter < 4);
	}
	EXTI_ClearITPendingBit(EXTI_Line2);
}

