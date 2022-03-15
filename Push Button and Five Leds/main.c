#include <stddef.h>
#include "stm32l0xx.h"


#define DEBOUNCE_TIME 3000000

HAL_GPIO_InitTypeDef GPIO_Component;
uint16_t Pins[4] = {GPIO_Pin_3, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_10};
uint8_t Pressed_Event = 0;
BitAction Current_State;

void LEDS_Setup(GPIO_InitTypeDef LEDS, uint16_t PINS[4]){
	  LEDS.GPIO_Pin = PINS[0] | PINS[1] | PINS[2] | PINS[3];
	  LEDS.GPIO_Mode = GPIO_Mode_OUT;
	  LEDS.GPIO_OType = GPIO_OType_PP;
	  LEDS.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  LEDS.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &LEDS);

	  LEDS.GPIO_Pin = PINS[2];
	  LEDS.GPIO_Mode = GPIO_Mode_OUT;
	  LEDS.GPIO_OType = GPIO_OType_PP;
	  LEDS.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  LEDS.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &LEDS);
}

void Wait_uS(__IO uint32_t nCount)
{
   while(nCount--){
   }
}

void Blink_Leds(GPIO_InitTypeDef LEDS, uint16_t PINS[4]){
	GPIO_WriteBit(GPIOA, PINS[2], Bit_RESET);

	GPIO_WriteBit(GPIOB, PINS[0], Bit_SET);
    GPIO_WriteBit(GPIOB, PINS[1], Bit_RESET);

    GPIO_WriteBit(GPIOB, PINS[2], Bit_SET);
    GPIO_WriteBit(GPIOB, PINS[3], Bit_RESET);
}

void Toggle_Leds(GPIO_InitTypeDef LEDS, uint16_t PINS[4]){
    GPIO_WriteBit(GPIOA, PINS[2], Bit_SET);
	GPIO_WriteBit(GPIOB, PINS[0], Bit_RESET);
    GPIO_WriteBit(GPIOB, PINS[1], Bit_SET);

    GPIO_WriteBit(GPIOB, PINS[2], Bit_RESET);
    GPIO_WriteBit(GPIOB, PINS[3], Bit_SET);
}

void Enable_GPIOD_Clock(){
	 //enable PORTD (GPIOD) clock
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
}


void BUTTON_Setup(GPIO_InitTypeDef Switch, uint16_t PIN){
	Switch.GPIO_Pin =  PIN;
	Switch.GPIO_Mode = GPIO_Mode_IN;
	Switch.GPIO_OType = GPIO_OType_PP;
	Switch.GPIO_PuPd = GPIO_PuPd_DOWN;
	Switch.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &Switch);
}

BitAction Check_Button_State(GPIO_InitTypeDef Switch, uint16_t PIN){
	BitAction state;
	state = GPIO_ReadInputDataBit(GPIOA, PIN);
  return state;
}

void Enable_GPIOA_Clock(){
//enable PORTD (GPIOD) clock
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}

int main(void)
{
	Enable_GPIOD_Clock();
	Enable_GPIOA_Clock();

	LEDS_Setup(GPIO_Component, Pins);
	BUTTON_Setup(GPIO_Component, Pins[3]);

  while (1)
  {
	Current_State = Check_Button_State(GPIO_Component, Pins[3]);
    if(Current_State == Bit_SET)
    {
    	if(!Pressed_Event)
    	{
    		Pressed_Event = 1;
    	}
    	else
    	{
    	   Pressed_Event = 0;
    	}
    	Wait_uS(DEBOUNCE_TIME);
    }

    if(!Pressed_Event)
    {
    	Blink_Leds(GPIO_Component, Pins);
    }
    else
    {
    	Toggle_Leds(GPIO_Component, Pins);
    }

  }

}
