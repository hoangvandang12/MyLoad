#ifndef _INA260_H_
#define _INA260_H_

#include "main.h"
#include "stdbool.h"

#define CONFIG_REGISTER_ADD 				0x00
#define CURRENT_REGISTER_ADD 				0x01
#define VOLTAGE_REGISTER_ADD 				0x02
#define POWER_REGISTER_ADD 					0x03
#define MASK_EN_REGISTER_ADD 				0x06
#define ALERT_REGISTER_ADD 					0x07
#define MANUFACTURER_REGISTER_ADD 	0xFE
#define DIE_ID_REGISTER_ADD 				0xFF

#define INA260_ADDRESS							0x80
typedef enum{
	
	AVR1 = 0,
	AVR4,
	AVR16,
	AVR64,
	AVR128,
	AVR256,
	AVR512,
	AVR1024
}averaging_mode_t;

typedef enum{
	
	V_CONV_140_US = 0,
	V_CONV_204_US,
	V_CONV_332_US,
	V_CONV_588_US,
	V_CONV_1_1_MS,
	V_CONV_2_116_MS,
	V_CONV_4_156_MS,
	V_CONV_8_244_MS
}voltage_conversion_time_t;


typedef enum{
	
	C_CONV_140_US = 0,
	C_CONV_204_US,
	C_CONV_332_US,
	C_CONV_588_US,
	C_CONV_1_1_MS,
	C_CONV_2_116_MS,
	C_CONV_4_156_MS,
	C_CONV_8_244_MS
}current_conversion_time_t;

typedef enum{
	
	POWER_DOWN_TRIGGER = 0,
	CURRENT_TRIGGER,
	VOLTAGE_TRIGGER,
	CURRENT_VOLTAGE_TRIGGER,
	POWER_DOWN_CONTINUTE,
	CURRENT_CONTINUTE,
	VOLTAGE_CONTINUTE,
	CURRENT_VOLTAGE_CONTINUTE
}operating_mode_t;

typedef struct{
	
	averaging_mode_t 					AVERAGING_MODE;
	voltage_conversion_time_t VOLTAGE_CONVERSION_TIME;
	current_conversion_time_t CURRENT_CONVERSION_TIME;
	operating_mode_t 					OPERATING_MODE;
}ina260_t;

extern I2C_HandleTypeDef hi2c1;

static void INA260_Write(uint8_t slave_add, uint8_t register_add, uint16_t data);
static void INA260_Read(uint8_t slave_add, uint8_t register_add, uint16_t *data);
void INA260_Config(ina260_t *ina260, averaging_mode_t Average, voltage_conversion_time_t Voltage_time, current_conversion_time_t Current_time, operating_mode_t Mode, bool Reset);
float INA260_Current_Read(void);
float INA260_Voltage_Read(void);
float INA260_Power_Read(void);
uint16_t INA260_ID(void);


#endif
