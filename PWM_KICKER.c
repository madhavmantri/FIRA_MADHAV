#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_gpio.h>


void Initialise_Clock()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

void Initialise_GPIO()
{
	GPIO_InitTypeDef GPIO_PWM;

	GPIO_PWM.GPIO_Mode = GPIO_Mode_AF;
	GPIO_PWM.GPIO_Pin = GPIO_Pin_6;
	GPIO_PWM.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_PWM.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_PWM.GPIO_OType = GPIO_OType_PP;
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_Init(GPIOA,&GPIO_PWM);

}
 void Initialise_Tim()
 {
		TIM_TimeBaseInitTypeDef TIM3_INIT;
		TIM_OCInitTypeDef TIM3_OC;
		TIM3_INIT.TIM_Prescaler =0;
		TIM3_INIT.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM3_INIT.TIM_CounterMode = TIM_CounterMode_Up;
		TIM3_INIT.TIM_Period = 80;

		TIM_TimeBaseInit(TIM3,&TIM3_INIT);

		TIM3_OC.TIM_OCMode = TIM_OCMode_PWM1;
		TIM3_OC.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM3_OC.TIM_OutputState = TIM_OutputState_Enable ;
		TIM3_OC.TIM_Pulse=16;
		TIM_OC1Init(TIM3,&TIM3_OC);

		TIM_Cmd(TIM3,ENABLE);
 }
 int main()
 {
	 Initialise_Clock();
	 Initialise_GPIO();
	 Initialise_Tim();
	 while(1)
	 {

	 }
 }

