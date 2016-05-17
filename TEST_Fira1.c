#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <misc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_flash.h>

#include "Peripherals.h"




/* Pin Configuration
 * Board - Fira_2014
 * Current Processor Speed - 168MHz
 *
 * Pins for LEDs: PE8 - PE12
 * Pin for Switch: 	PB0-Id_1
 * 					PB1=Id_2
 * 					PB10-Id_0
 * 					PB11-Bot Color
 *
 * Pins for Motor PWM - PB6 PB7 Motor 1
 * 						PB8 PB9 Motor 2
 *
 * PWM on Timer 4
 * Timer 4 Channels - 	PB6 Channel 1
 *						PB7 Channel 2
 * 						PB8 Channel 3
 * 						PB9 Channel 4
 *
 * 	UART on Channel 2
 * 	Pins for UART -  	PA3 Rx
 * 						PA2 Tx
 *
 * 	Timer Interrupt on Timer 2
 *
 * 	External Interrupt - PB4 Motor 1
 * 						 PC10 Motor 2
 *
 * 	Pins for Encoder - PB5 Motor 1
 * 					   PC11 Motor 2
 *
 */
#define STORE_SIZE 200
volatile int8_t VL=0,VR=0,Store[STORE_SIZE][5]={0},Send_Back_Flag=0;
volatile int NumStored = 0; // number of packets stored in Store
volatile int StoreIdx = 0; // idx to store next packet

volatile int8_t Data=0,TeamID=0;
int Bot_Id=0,Bot_Flag=0;
volatile int i=0,loop=0,p=0,j=0,enable_timer_int=0;
volatile int Previous_Error[2] = {0}, Ticks[2] = {0}, D_Error[2] = {0}, Error[2] = {0}, Target[2] = {0};
volatile int Cycle_Complete_Flag = 0, Flag = 0,Count=0,Enable_Flag=0;
volatile int Velocity[2]={0};
volatile int Motor_Velocity[2] = {0},
	Negative_Error[2] = {0}, Zero_Error[2] = {0}, Positive_Error[2] = {0},
	Negative_D_Error[2] = {0}, Zero_D_Error[2] = {0}, Positive_D_Error[2] = {0},
	Kp_Small[2] = {0}, Kp_Medium[2] = {0}, Kp_Large[2] = {0},
	Kd_Small[2] = {0}, Kd_Medium[2] = {0}, Kd_Large[2] = {0},
	Fuzzy_Matrix[2][3][3] = {{{0}}};
volatile int Kp_Multiplier[2], Kd_Multiplier[2],
	Kp_Divider[2], Kd_Divider[2],
    Fault[2] = {0};
volatile int Ticks_Array_L[100]={0},Ticks_Array_R[100]={0};





void Error_Fuzzification(void)
{
	int i = 0;
	for(i = 0; i < 2; i++)
	{
		Positive_Error[i] = Negative_Error[i] = Zero_Error[i] = 0;
		if(Error[i]>10)
		{
			Positive_Error[i] = 10;
		}
		else if(Error[i]<-10)
		{
			Negative_Error[i] = 10;
		}
		else
		{
			if(Error[i] > 0)
			{
				Positive_Error[i] = (Error[i]);
				Zero_Error[i] = 10 - ((Error[i]));
			}
			else
			{
				Negative_Error[i] =  -((Error[i]));
				Zero_Error[i] = 10 + (Error[i]);
			}
		}
		Positive_D_Error[i] = Negative_D_Error[i] = Zero_D_Error[i] = 0;
		if(D_Error[i]>10)
		{
			Positive_D_Error[i] = 10;
		}
		else if(D_Error[i]<-10)
		{
			Negative_D_Error[i] = 10;
		}
		else
		{
			if(D_Error[i] > 0)
			{
				Positive_D_Error[i] = (D_Error[i]);
				Zero_D_Error[i] = 10 - ((D_Error[i]));
			}
			else
			{
				Negative_D_Error[i] =  -(D_Error[i]);
				Zero_D_Error[i] = 10 + (D_Error[i]);
			}
		}


	}
}

void Create_Fuzzy_Matrix(void)
{
	int i = 0;
/*
	for(i = 0; i < 2; i++)
		{
			Fuzzy_Matrix[i][0][0] = Negative_D_Error[i] * Negative_Error[i];
			Fuzzy_Matrix[i][0][1] = Zero_D_Error[i] * Negative_Error[i];
			Fuzzy_Matrix[i][0][2] = Positive_D_Error[i] * Negative_Error[i];
			Fuzzy_Matrix[i][1][0] = Negative_D_Error[i] * Zero_Error[i];
			Fuzzy_Matrix[i][1][1] = Zero_D_Error[i] * Zero_Error[i];
			Fuzzy_Matrix[i][1][2] = Positive_D_Error[i] * Zero_Error[i];
			Fuzzy_Matrix[i][2][0] = Negative_D_Error[i] * Positive_Error[i];
			Fuzzy_Matrix[i][2][1] = Zero_D_Error[i] * Positive_Error[i];
			Fuzzy_Matrix[i][2][2] = Positive_D_Error[i] * Positive_Error[i];
		}
*/
	for(i = 0; i < 2; i++)
	{

		Fuzzy_Matrix[i][0][0] = Negative_D_Error[i] < Negative_Error[i] ? Negative_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][1] = Zero_D_Error[i] < Negative_Error[i] ? Zero_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][0][2] = Positive_D_Error[i] < Negative_Error[i] ? Positive_D_Error[i] : Negative_Error[i];
		Fuzzy_Matrix[i][1][0] = Negative_D_Error[i] < Zero_Error[i] ? Negative_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][1] = Zero_D_Error[i] < Zero_Error[i] ? Zero_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][1][2] = Positive_D_Error[i] < Zero_Error[i] ? Positive_D_Error[i] : Zero_Error[i];
		Fuzzy_Matrix[i][2][0] = Negative_D_Error[i] < Positive_Error[i] ? Negative_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][1] = Zero_D_Error[i] < Positive_Error[i] ? Zero_D_Error[i] : Positive_Error[i];
		Fuzzy_Matrix[i][2][2] = Positive_D_Error[i] < Positive_Error[i] ? Positive_D_Error[i] : Positive_Error[i];
	}


}
void Determine_Weights(void)
{
	int i ;
	Kp_Multiplier[1] = Kp_Multiplier[0] = Kd_Multiplier[1] = Kd_Multiplier[0] = 0;

	for(i = 0; i < 2; i++)
	{

		Kp_Small[i] = Fuzzy_Matrix[i][1][1];

		Kp_Medium[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][0][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][1][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][1][2];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][0][0] ? Kp_Medium[i] : Fuzzy_Matrix[i][0][0];
		Kp_Medium[i] = Kp_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kp_Medium[i] : Fuzzy_Matrix[i][2][2];


		Kp_Large[i] =  Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];


		Kd_Small[i] = Fuzzy_Matrix[i][0][2] > Fuzzy_Matrix[i][2][0] ? Fuzzy_Matrix[i][0][2] : Fuzzy_Matrix[i][2][0];
		Kd_Small[i] = Kd_Small[i] > Fuzzy_Matrix[i][1][1] ? Kd_Small[i] : Fuzzy_Matrix[i][1][1];


		Kd_Medium[i] = Fuzzy_Matrix[i][0][0] > Fuzzy_Matrix[i][0][1] ? Fuzzy_Matrix[i][0][0] : Fuzzy_Matrix[i][0][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][1] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][1];
		Kd_Medium[i] = Kd_Medium[i] > Fuzzy_Matrix[i][2][2] ? Kd_Medium[i] : Fuzzy_Matrix[i][2][2];


		Kd_Large[i] = Fuzzy_Matrix[i][1][0] > Fuzzy_Matrix[i][1][2] ? Fuzzy_Matrix[i][1][0] : Fuzzy_Matrix[i][1][2];


		Kp_Multiplier[i] = (Kp_Small[i] * Kp_Small_Value) + (Kp_Medium[i] * Kp_Medium_Value) + (Kp_Large[i] * Kp_Large_Value);
		Kd_Multiplier[i] = (Kd_Small[i] * Kd_Small_Value) + (Kd_Medium[i] * Kd_Medium_Value) + (Kd_Large[i] * Kd_Large_Value);

		Kp_Divider[i] = (Kp_Small[i] + Kp_Medium[i] + Kp_Large[i]);
		Kd_Divider[i] = (Kd_Small[i] + Kd_Medium[i] + Kd_Large[i]);
	}

/*
	for(i = 0; i < 2; i++)
	{

		Kp_Small[i] = Fuzzy_Matrix[i][1][1];
		Kp_Medium[i] = (Fuzzy_Matrix[i][1][0] + Fuzzy_Matrix[i][0][1] + Fuzzy_Matrix[i][2][1] + Fuzzy_Matrix[i][1][2] + Fuzzy_Matrix[i][0][0] + Fuzzy_Matrix[i][2][2])/6;
		Kp_Large[i] =  (Fuzzy_Matrix[i][0][2] + Fuzzy_Matrix[i][2][0])/2;

		Kd_Small[i] = Kp_Small[i];
		Kd_Medium[i] = Kp_Medium[i];
		Kd_Large[i] = Kp_Large[i];

		Kp_Multiplier[i] = (Kp_Small[i] * Kp_Small_Value) + (Kp_Medium[i] * Kp_Medium_Value) + (Kp_Large[i] * Kp_Large_Value);
		Kd_Multiplier[i] = (Kd_Small[i] * Kd_Small_Value) + (Kd_Medium[i] * Kd_Medium_Value) + (Kd_Large[i] * Kd_Large_Value);

		Kp_Divider[i] = (Kp_Small[i] + Kp_Medium[i] + Kp_Large[i]);
		Kd_Divider[i] = (Kd_Small[i] + Kd_Medium[i] + Kd_Large[i]);
	}
*/
/*
		for(i = 0; i < 2; i++)
		{

			Kp_Small[i] = Fuzzy_Matrix[i][1][1];
			Kp_Medium[i] = sqrt(((Fuzzy_Matrix[i][1][0]*Fuzzy_Matrix[i][1][0]) + (Fuzzy_Matrix[i][0][1]*Fuzzy_Matrix[i][0][1]) + (Fuzzy_Matrix[i][2][1]*Fuzzy_Matrix[i][2][1]) + (Fuzzy_Matrix[i][1][2]*Fuzzy_Matrix[i][1][2]) + (Fuzzy_Matrix[i][0][0]*Fuzzy_Matrix[i][0][0]) + (Fuzzy_Matrix[i][2][2]*Fuzzy_Matrix[i][2][2]))/6);
            Kp_Large[i]= sqrt(((Fuzzy_Matrix[i][0][2]*Fuzzy_Matrix[i][0][2]) + (Fuzzy_Matrix[i][2][0]*Fuzzy_Matrix[i][2][0]))/2);
			Kd_Small[i] = Kp_Small[i];
			Kd_Medium[i] = Kp_Medium[i];
			Kd_Large[i] = Kp_Large[i];

			Kp_Multiplier[i] = (Kp_Small[i] * Kp_Small_Value) + (Kp_Medium[i] * Kp_Medium_Value) + (Kp_Large[i] * Kp_Large_Value);
			Kd_Multiplier[i] = (Kd_Small[i] * Kd_Small_Value) + (Kd_Medium[i] * Kd_Medium_Value) + (Kd_Large[i] * Kd_Large_Value);

			Kp_Divider[i] = (Kp_Small[i] + Kp_Medium[i] + Kp_Large[i]);
			Kd_Divider[i] = (Kd_Small[i] + Kd_Medium[i] + Kd_Large[i]);
		}
*/
}

void Give_Motor_Velocity(void)
{
	int16_t i;

	for(i = 0; i < 2; i++)
	{

		if(Kp_Divider[i] != 0 && Kd_Divider[i] != 0)
		{
			Fault[i] = ((Error[i] * Kp_Multiplier[i]) / Kp_Divider[i]);
			Fault[i] += ((D_Error[i] * Kd_Multiplier[i]) / Kd_Divider[i]);
		}
		Motor_Velocity[i] += Fault[i];
	}

	if(Motor_Velocity[0] < 0)
	{
		if(Motor_Velocity[0] < -254)
		{

		//	TIM_SetCompare2(TIM4,254);
			Motor_Velocity[0] = -254;
		}
		TIM_SetCompare2(TIM4,254);
		TIM_SetCompare1(TIM4,254+Motor_Velocity[0]);
	}
	else
	{
		if(Motor_Velocity[0] > 254)
		{
			//TIM_SetCompare1(TIM4,254);
			Motor_Velocity[0] = 254;
		}
		TIM_SetCompare1(TIM4,254);
		TIM_SetCompare2(TIM4,254-Motor_Velocity[0]);
	}

	if(Motor_Velocity[1] < 0)
	{
		if(Motor_Velocity[1] <-254)
		{
			//TIM_SetCompare3(TIM4,254);
			Motor_Velocity[1] = -254;
		}

		TIM_SetCompare3(TIM4,254);
		TIM_SetCompare4(TIM4,254+Motor_Velocity[1]);
	}
	else
	{
		if(Motor_Velocity[1] >254)
		{
			//TIM_SetCompare4(TIM4,254);
			Motor_Velocity[1] = 254;
		}

		TIM_SetCompare4(TIM4,254);
		TIM_SetCompare3(TIM4,254-Motor_Velocity[1]);

	}
}

/*
void Give_Motor_Velocity(void)
{
	int16_t i;

	for(i = 0; i < 2; i++)
	{

		if(Kp_Divider[i] != 0 && Kd_Divider[i] != 0)
		{
			Fault[i] = ((Error[i]  *Kp_Multiplier[i]) / Kp_Divider[i]);
			Fault[i] += ((D_Error[i] * Kd_Multiplier[i]) / Kd_Divider[i]);
		}
		Motor_Velocity[i] += Fault[i];
	}

	if(Motor_Velocity[1] < 0)
	{
		if(Motor_Velocity[1] <-Top)
		{
			Motor_Velocity[1] = -Top;
		}
		TIM_SetCompare3(TIM4,-Motor_Velocity[1]);
		TIM_SetCompare4(TIM4,0);
	}
	else
	{
		if(Motor_Velocity[1] >Top)
		{
			Motor_Velocity[1] = Top;
		}
		TIM_SetCompare4(TIM4,Motor_Velocity[1]);
		TIM_SetCompare3(TIM4,0);
	}

	if(Motor_Velocity[0] < 0)
	{
		if(Motor_Velocity[0] < -Top)
		{
			Motor_Velocity[0] = -Top;
		}
		TIM_SetCompare2(TIM4,-Motor_Velocity[0]);
		TIM_SetCompare1(TIM4,0);
	}
	else
	{
		if(Motor_Velocity[0] > Top)
		{
			Motor_Velocity[0] = Top;
		}
		TIM_SetCompare1(TIM4,Motor_Velocity[0]);
		TIM_SetCompare2(TIM4,0);
	}


}
*/
int main()
{
	 int i=0,j=0;
	Initialise_Clock();
	Initialise_GPIO();
	Initialise_TimerPWM();
	Initialise_UART();

	Initialise_ExternalInterrupt();
	Initialise_TimerInterrupt();
	enable_timer_int=0;

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11))
	{
		TeamID=Team_Id_2;
	}
	else
	{
		TeamID=Team_Id_1;
	}
	Bot_Id=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)*4+GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)*2+GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
/*
	if(TeamID==126)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
	}
	else if(TeamID=127)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
	}
*/
	if(Bot_Id==0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
	}
	else if (Bot_Id==1)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
	}
	else if(Bot_Id==4)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);
		GPIO_SetBits(GPIOB,GPIO_Pin_15);
	}

		Ticks[0] = Ticks[1] = 0;



	    while(1)
	    {

	    	if(enable_timer_int)
	    	{
	    		//TIM_SetCounter(TIM2,0);
	    		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	    		enable_timer_int=0;
	    	}
			if(Cycle_Complete_Flag == 1)
			{
				Cycle_Complete_Flag = 0;
				//GPIO_ResetBits(GPIOB,GPIO_Pin_14);
				Error_Fuzzification();
				Create_Fuzzy_Matrix();
				Determine_Weights();
				Give_Motor_Velocity();
			}
			if(Send_Back_Flag==1)
			{
				Send_Back_Flag=0;
				Target[0]=0;
				Target[1]=0;
				Count = 0;
				for(i=(StoreIdx - NumStored + STORE_SIZE)%STORE_SIZE; Count<NumStored; i++, Count++)
				{
					i = i % STORE_SIZE;
					USART_SendData(USART2,122);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Bot_Id);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Store[i][0]);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Store[i][1]);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Store[i][2]);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Store[i][3]);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
					USART_SendData(USART2,Store[i][4]);
					while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);

				}
							// Send 123 to indicate end of data
				USART_SendData(USART2,123);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
				StoreIdx = 0;
				NumStored = 0;
			}
			/*if(loop>100)
			{
				Target[0]=0;
				Target[1]=0;
				p=0;
				loop=0;
				enable_timer_int=0;
				for(i=0;i<100;i++)
				{
					USART_SendData(USART2,Ticks_Array_L[i]);
					while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
				}
				for(i=0;i<100;i++)
			    {
					USART_SendData(USART2,Ticks_Array_R[i]);
					while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
			    }
				Reset_Handler();

			}*/
		}
}

void USART2_IRQHandler()
{

	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		Data = USART_ReceiveData(USART2);
		if(Data!=122&&Bot_Flag==0)
		{
			if(Data==TeamID && Flag==0 )
			{
				Flag++;
			}
			else if(Flag==2*Bot_Id+1)
			{
				Velocity[0]=Data;
				Flag++;
			}
			else if(Flag==2*Bot_Id+2)
			{
				Velocity[1]=Data;
				Flag++;
			}

			else if(Flag==11)
			{
				Target[0]=Velocity[0];
				Target[1]=Velocity[1];

				Store[StoreIdx][0]=Data;
				Store[StoreIdx][1]=Target[0];
				Store[StoreIdx][2]=Target[1];
				Store[StoreIdx][3]=VL;
				Store[StoreIdx][4]=VR;
				StoreIdx = (StoreIdx + 1 ) % STORE_SIZE;
				NumStored++;
				if(NumStored > STORE_SIZE)
				{
					NumStored = STORE_SIZE;
				}
				Flag=0;
			}
			else
			{
				Flag++;
			}
		}
		else if(Data==122)
		{
			Bot_Flag=1;
		}
		else if(Data!=122&&Bot_Flag==1)
		{
			if(Data==Bot_Id)
			{
				Send_Back_Flag=1;
			}
			Bot_Flag=0;
		}

		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		NVIC_ClearPendingIRQ(USART2_IRQn);
	}
}

void EXTI15_10_IRQHandler()
{
	if((EXTI->IMR & EXTI_IMR_MR10) && (EXTI->PR & EXTI_PR_PR10))
		{
			if(GPIOC->IDR&GPIO_Pin_11)
			{
				Ticks[1]++;
			}
			else
			{
				Ticks[1]--;
			}
			EXTI->PR |= EXTI_PR_PR10 ;
			NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
		}
}

void EXTI4_IRQHandler()
{

	if((EXTI->IMR & EXTI_IMR_MR4) && (EXTI->PR & EXTI_PR_PR4))
	{
		if(GPIOB->IDR&GPIO_Pin_5)
		{
			Ticks[0]++;
		}
		else
		{
			Ticks[0]--;
		}
		EXTI->PR |= EXTI_PR_PR4 ;
		NVIC_ClearPendingIRQ(EXTI4_IRQn);
	}
}

void TIM2_IRQHandler()
{

	Cycle_Complete_Flag = 1;

	Error[0] = (Target[0] - Ticks[0]);
	Error[1] = (Target[1] - Ticks[1]) ;

	D_Error[1] = Error[1] - Previous_Error[1];
	D_Error[0] = Error[0] - Previous_Error[0];

	Previous_Error[1] = Error[1];
	Previous_Error[0] = Error[0];
/*
	if(Target[0]>0&&Target[1]>0)
	{
		Ticks_Array_L[p]=Ticks[0];
		Ticks_Array_R[p]=Ticks[1];
		p++;
		loop++;
	}
*/
	VL=Ticks[0];
	VR=Ticks[1];

	Ticks[0] = 0;
	Ticks[1] = 0;

	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
}

