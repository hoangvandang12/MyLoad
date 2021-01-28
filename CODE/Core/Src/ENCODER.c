#include "ENCODER.h"

void ENCODER_EC_11(uint16_t *value, uint16_t GPIO_Pin){
		
	if(GPIO_Pin == ENCODER_PA_Pin){
				
		if(HAL_GPIO_ReadPin(ENCODER_PB_GPIO_Port,ENCODER_PB_Pin)){
			
			if(*value<4095)
				(*value)+=20;
		}
		else{
			
			if(*value>0)
				(*value)-=20;
		}
	}
}



