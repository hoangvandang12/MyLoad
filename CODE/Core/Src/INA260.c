#include "INA260.h"

static void INA260_Write(uint8_t slave_add, uint8_t register_add, uint16_t data){
		
	uint8_t tg=0;
	tg=data>>8;
	data=(data<<8)|tg;
	HAL_I2C_Mem_Write(&hi2c1,slave_add,register_add,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&data,2,2000);
}	

static void INA260_Read(uint8_t slave_add, uint8_t register_add, uint16_t *data){
	
	uint16_t buff=0;
	uint8_t tg=0;
	
	HAL_I2C_Mem_Read(&hi2c1,slave_add,register_add,I2C_MEMADD_SIZE_8BIT,(uint8_t*)&buff,2,2000);
	tg=buff>>8;
	buff=(buff<<8)|tg;
	*data=buff;
}

//static void INA260_Point_Set

void INA260_Config(AVERAGING_MODE Average, VOLTAGE_CONVERSION_TIME Voltage_time, CURRENT_CONVERSION_TIME Current_time, OPERATING_MODE Mode, bool Reset){
	
	uint16_t data=0;
	
	if(Reset){
		
		data |= (1<<15); // Resets all registers to default values; this bit self-clears.
		Reset=false; // clear bit reset
		goto end;
	}
	
	//---------------Averaging Mode---------------//
	switch(Average){	
		case AVR1:
			data &= ~(1<<9);
			data &= ~(1<<10);
			data &= ~(1<<11);
			break;
		case AVR4:
			data |=  (1<<9);
			data &= ~(1<<10);
			data &= ~(1<<11);
			break;
		case AVR16:
			data &= ~(1<<9);
			data |=  (1<<10);
			data &= ~(1<<11);
			break;
		case AVR64:
			data |=  (1<<9);
			data |=  (1<<10);
			data &= ~(1<<11);
			break;
		case AVR128:
			data &= ~(1<<9);
			data &= ~(1<<10);
			data |=  (1<<11);
			break;
		case AVR256:
			data |=  (1<<9);
			data &= ~(1<<10);
			data |=  (1<<11);
			break;
		case AVR512:
			data &= ~(1<<9);
			data |=  (1<<10);
			data |=  (1<<11);
			break;
		case AVR1024:
			data |=  (1<<9);
			data |=  (1<<10);
			data |=  (1<<11);
			break;
		default:
			data &= ~(1<<9);
			data &= ~(1<<10);
			data &= ~(1<<11);
			break;
	}
	//---------------END Averaging Mode---------------//
	
	//---------------Bus Voltage Conversion Time---------------//
	switch(Voltage_time){			
		case V_CONV_140_US:
			data &= ~(1<<6);
			data &= ~(1<<7);
			data &= ~(1<<8);
			break;
		case V_CONV_204_US:
			data &=  (1<<6);
			data &= ~(1<<7);
			data &= ~(1<<8);
			break;
		case V_CONV_332_US:
			data &= ~(1<<6);
			data &=  (1<<7);
			data &= ~(1<<8);
			break;
		case V_CONV_588_US:
			data &=  (1<<6);
			data &=  (1<<7);
			data &= ~(1<<8);
			break;
		case V_CONV_1_1_MS:
			data &= ~(1<<6);
			data &= ~(1<<7);
			data &=  (1<<8);
			break;
		case V_CONV_2_116_MS:
			data &=  (1<<6);
			data &= ~(1<<7);
			data &=  (1<<8);
			break;
		case V_CONV_4_156_MS:
			data &= ~(1<<6);
			data &=  (1<<7);
			data &=  (1<<8);
			break;
		case V_CONV_8_244_MS:
			data &=  (1<<6);
			data &=  (1<<7);
			data &=  (1<<8);
			break;
		default: // 1.1MS
			data &= ~(1<<6);
			data &= ~(1<<7);
			data &=  (1<<8);
			break;
	}
	//---------------END Bus Voltage Conversion Time---------------//
	
	//---------------Shunt Current Conversion Time---------------//
	switch(Current_time){	
		case C_CONV_140_US:
			data &= ~(1<<3);
			data &= ~(1<<4);
			data &= ~(1<<5);
			break;
		case C_CONV_204_US:
			data &=  (1<<3);
			data &= ~(1<<4);
			data &= ~(1<<5);
			break;
		case C_CONV_332_US:
			data &= ~(1<<3);
			data &=  (1<<4);
			data &= ~(1<<5);
			break;
		case C_CONV_588_US:
			data &=  (1<<3);
			data &=  (1<<4);
			data &= ~(1<<5);
			break;
		case C_CONV_1_1_MS:
			data &= ~(1<<3);
			data &= ~(1<<4);
			data &=  (1<<5);
			break;
		case C_CONV_2_116_MS:
			data &=  (1<<3);
			data &= ~(1<<4);
			data &=  (1<<5);
			break;
		case C_CONV_4_156_MS:
			data &= ~(1<<3);
			data &=  (1<<4);
			data &=  (1<<5);
			break;
		case C_CONV_8_244_MS:
			data &=  (1<<3);
			data &=  (1<<4);
			data &=  (1<<5);
			break;
		default: // 1.1MS
			data &= ~(1<<3);
			data &= ~(1<<4);
			data &=  (1<<5);
			break;
	}
	//---------------END Shunt Current Conversion Time---------------//
	
	//---------------Operating Mode---------------//
	switch(Mode){		
		case POWER_DOWN_TRIGGER:
			data &= ~(1<<0);
			data &= ~(1<<1);
			data &= ~(1<<2);
			break;
		case CURRENT_TRIGGER:
			data &=  (1<<0);
			data &= ~(1<<1);
			data &= ~(1<<2);
			break;
		case VOLTAGE_TRIGGER:
			data &= ~(1<<0);
			data &=  (1<<1);
			data &= ~(1<<2);
			break;
		case CURRENT_VOLTAGE_TRIGGER:
			data &=  (1<<0);
			data &=  (1<<1);
			data &= ~(1<<2);
			break;
		case POWER_DOWN_CONTINUTE:
			data &= ~(1<<0);
			data &= ~(1<<1);
			data &=  (1<<2);
			break;
		case CURRENT_CONTINUTE:
			data &=  (1<<0);
			data &= ~(1<<1);
			data &=  (1<<2);
			break;
		case VOLTAGE_CONTINUTE:
			data &= ~(1<<0);
			data &=  (1<<1);
			data &=  (1<<2);
			break;
		case CURRENT_VOLTAGE_CONTINUTE:
			data &=  (1<<0);
			data &=  (1<<1);
			data &=  (1<<2);
			break;
		default: // CURRENT_VOLTAGE_CONTINUTE
			data &=  (1<<0);
			data &=  (1<<1);
			data &=  (1<<2);
			break;
	}
	//---------------END Operating Mode---------------//
	end:
		INA260_Write(INA260_ADDRESS,CONFIG_REGISTER_ADD,data);
	
}

float INA260_Current_Read(void){
	
	uint16_t data=0;
	
	INA260_Read(INA260_ADDRESS,CURRENT_REGISTER_ADD,&data);
	
	if(data &= (1<<15))
		return -((data-32768)*1.25/1000);
	else
		return (data*1.25/1000);
}

float INA260_Voltage_Read(void){
	
	uint16_t data=0;
	
	INA260_Read(INA260_ADDRESS,VOLTAGE_REGISTER_ADD,&data);
	
	return (data*1.25/1000);
}

float INA260_Power_Read(void){
	
	uint16_t data=0;
	
	INA260_Read(INA260_ADDRESS,POWER_REGISTER_ADD,&data);
	
	return (data/10);
}

uint16_t INA260_ID(void){
	
	uint16_t data=0;
	
	INA260_Read(INA260_ADDRESS,MANUFACTURER_REGISTER_ADD,&data);
	
	return data;
}

