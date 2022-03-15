/**************************************************************************************
  B e g i n  o f   p r o g r a m  -  four leds with timer-triggered blinking
***************************************************************************************/
#include "stm32f4xx.h"

//#include "stm32f4_discovery.h"
#define NUM_LEDS 	4
#define Leds_Port 	GPIOC

/**************************************************************************************
 P r o t o t y p e s
***************************************************************************************/
void Leds_Enable_Clock(void);
void Leds_Setup(GPIO_InitTypeDef LEDS, uint16_t PINS[4]);
void Leds_Timer_Channel_Setup(uint16_t SOURCES[4]);
void Leds_Timer_Setup(TIM_TimeBaseInitTypeDef htim3);
void vLeds_GPIO_Task(GPIO_InitTypeDef LEDS, uint16_t PINS[4],
		uint16_t SOURCES[4], TIM_TimeBaseInitTypeDef htim3);
void vLeds_Blink_Task(TIM_OCInitTypeDef OCInit);

/**************************************************************************************
 V a r i a b l e s
***************************************************************************************/
GPIO_InitTypeDef ledInit;
TIM_TimeBaseInitTypeDef TIM_LedTimer;
TIM_OCInitTypeDef TIM_OCInit;

uint16_t four_leds[NUM_LEDS] = {GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9};
uint16_t sources[NUM_LEDS] = {GPIO_PinSource6, GPIO_PinSource7, GPIO_PinSource8, GPIO_PinSource9};
volatile uint16_t PrescalerValue;

/**************************************************************************************
 M a i n   r o u t i n e
***************************************************************************************/
int main(void)
{
	vLeds_GPIO_Task(ledInit, four_leds, sources, TIM_LedTimer);
	vLeds_Blink_Task(TIM_OCInit);
}

/**************************************************************************************
 F u n c t i o n s
***************************************************************************************/
void Leds_Enable_Clock(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	printf("\nClock have been enabled.\n");
}

void Leds_Setup(GPIO_InitTypeDef LEDS, uint16_t PINS[4]){
	  LEDS.GPIO_Pin 		= PINS[0] | PINS[1] | PINS[2] | PINS[3];
	  LEDS.GPIO_Mode 		= GPIO_Mode_AF;
	  LEDS.GPIO_OType 		= GPIO_OType_PP;
	  LEDS.GPIO_Speed 		= GPIO_Speed_100MHz;
	  LEDS.GPIO_PuPd 		= GPIO_PuPd_NOPULL;
	  GPIO_Init(Leds_Port, &LEDS);
	  printf("\tLeds has been configured.\n");
}

void Leds_Timer_Channel_Setup(uint16_t SOURCES[4])
{
	uint8_t i;
	for(i=0; i<NUM_LEDS; i++){
	  GPIO_PinAFConfig(Leds_Port, SOURCES[i], GPIO_AF_TIM3);
	}
	printf("\nTimer channels have been setup.\n");
}

void Leds_Timer_Setup(TIM_TimeBaseInitTypeDef htim3)
{												//	freq Hz
	PrescalerValue = (uint16_t)((SystemCoreClock/2)/13107)-1;
	htim3.TIM_Period 			= 0xFFFF; // counting up to 65535, 5 seconds
	htim3.TIM_Prescaler 		= PrescalerValue;
	htim3.TIM_ClockDivision 	= 0;
	htim3.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &htim3);
	printf("\tCouting period has been defined.\n");
}

void vLeds_GPIO_Task(GPIO_InitTypeDef LEDS, uint16_t PINS[4],
		uint16_t SOURCES[4], TIM_TimeBaseInitTypeDef htim3)
{
	Leds_Enable_Clock();
	Leds_Setup(LEDS, PINS);
	Leds_Timer_Channel_Setup(SOURCES);
	Leds_Timer_Setup(htim3);
	printf("\tLed GPIO Task successfully initialized!\n");
}

void vLeds_Blink_Task(TIM_OCInitTypeDef OCInit)
{   // LED 1
	printf("\nLed Blink Task starting...\n");
	/* OC stands for "output compare" */
	OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	OCInit.TIM_OutputState  = TIM_OutputState_Enable;
	OCInit.TIM_Pulse 		= 13170; //Hz
	OCInit.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &OCInit);  // pin -> PC6, High for 1 sec, low for 4 sec
	printf("\tLed 1 is on!\n");
		//output compare mode set up for channel 1 of tim3
	// LED 2
	OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	OCInit.TIM_OutputState  = TIM_OutputState_Enable;
	OCInit.TIM_Pulse 		= 26214; //Hz
	OCInit.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &OCInit); // high for 2 sec, low for 3 sec
	printf("\tLed 2 is on!\n");
    // LED 3
	OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	OCInit.TIM_OutputState  = TIM_OutputState_Enable;
	OCInit.TIM_Pulse 		= 39321; //Hz
	OCInit.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &OCInit); //high for 3 sec, low for 2 sec
	printf("\tLed 3 is on!\n");
	// LED 4
	OCInit.TIM_OCMode 		= TIM_OCMode_PWM1;
	OCInit.TIM_OutputState  = TIM_OutputState_Enable;
	OCInit.TIM_Pulse 		= 52428; //Hz
	OCInit.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OC4Init(TIM3, &OCInit); // high for 4 sec, low for 1 sec
	printf("\tLed 4 is on!\n");

	TIM_Cmd(TIM3, ENABLE);
}
/**************************************************************************************
  E n d   o f   p r o g r a m
***************************************************************************************/
