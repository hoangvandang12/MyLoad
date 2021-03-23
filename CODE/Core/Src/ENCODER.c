/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 OA electronic                                        																 */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
//***********************************************************************************************************
//  OA Electronic. 																				
//  Website: facebook
//  E-Mail : hoangvandang12@gmail.com
//  Date   : March/25/2021
//	ver		 : 1.0.0
//***********************************************************************************************************

/***********NOTE: configuration connector cube_mx**********

	///EXAMPLE///
	
	ENCODER_PA 		
	ENCODER_PA	
	ENCODER_BT 		

*/////////////////////////////////////////////////////////////////

/***********NOTE: configuration external interrupt **********



*/////////////////////////////////////////////////////////////////

#include "ENCODER.h"

volatile int16_t TickCount=0;
volatile int8_t TickCount_int8=0;
uint8_t step;
volatile bool update=false, TickBt=false;

void Encorder_C11_Init(uint8_t st){
	
	step=st;
}

static void Tick_EXTI_CallBack_Increase(void){
	
	TickCount_int8+=step;
	TickCount+=step;
	if(TickCount>=32000)
		TickCount=0;
}

static void Tick_EXTI_CallBack_Decrease(void){
	
	TickCount-=step;
	TickCount_int8-=step;
	if(TickCount<=(-32000))
		TickCount=0;
}

void Encoder_C11(int16_t *number, int16_t min, int16_t max){

		if(TickCount>max)
			TickCount=min;
		if(TickCount<min)
			TickCount=max;
		
		if(update){
			*number=TickCount;
			update=false;
		}
}

void Encoder_C11_int8(int8_t *number, int8_t min, int8_t max){

		if(TickCount_int8>max)
			TickCount_int8=min;
		if(TickCount_int8<min)
			TickCount_int8=max;
		
		if(update){
			*number=TickCount_int8;
			update=false;
		}
}


bool Encoder_BT(void){
	
		if(TickBt){	
			TickBt=false;
			return true;
		}
	return false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	
	if(GPIO_Pin == ENCODER_PA_Pin){	// NOTE: rotary encoder maybe diffirent 
			
		if(HAL_GPIO_ReadPin(ENCODER_PB_GPIO_Port,ENCODER_PB_Pin)){
			
			Tick_EXTI_CallBack_Decrease();
		}
		else{
					
			Tick_EXTI_CallBack_Increase();
		}
		update=true;
	}
	
	if(GPIO_Pin == ENCODER_BT_Pin){
	
		TickBt=true;
	}
		
	
}
/* END FILE	*/

