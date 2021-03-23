/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 OA electronic                                        																 */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
//***********************************************************************************************************
//  OA Electronic. 																				
//  Website: facebook
//  E-Mail : hoangvandang12@gmail.com
//  Date   : jan/18/2020
//	ver		 : 1.0
//***********************************************************************************************************

#include "PID.h"

//uint16_t PID(float point, float set_point, uint16_t max, uint16_t min){
//	
//	float error;
//	float derivative;
//	float previous_error=0.0;
//	float integral=0.0;
//	float output;
//	
//	error=-(set_point-point);
//	integral=integral+error*dt;
//	derivative=(error-previous_error)/dt;
//	output=Kp*error+Ki*integral+Kd*derivative;
//	previous_error=error;
//	if(output>max)	output=max;
//	if(output<min)		output=min;
//	
//	return ((uint16_t) output);
//}

uint16_t PID_VR2(float point, float set_point, uint16_t max, uint16_t min, bool inver, float Kp, float Ki, float Kd, float T, bool rst){
	
	static float E=0.0, E1=0.0, E2=0.0;
	static float alpha=0.0;
	static float beta=0.0;
	static float gama=0.0;
	static float output=0.0, LastOutput=0.0;
	
	if(inver){		
		E = -(set_point-point);	
	}
	else
		E = set_point-point;	
	
	alpha = 2*T*Kp + Ki*T*T + 2*Kd;
	beta = T*T*Ki - 4*Kd - 2*T*Kp;
	gama = 2*Kd;
	output = (alpha*E + beta*E1 + gama*E2 + 2*T*LastOutput)/(2*T);
	
	LastOutput=output;
	E2=E1;
	E1=E;
	
	if(output>max)	
		output=max;
	if(output<min)	
		output=min;
	
	if(rst)
		LastOutput=output=0;
	
	return ((uint16_t) output+1950);
	
}



/* END FILE	*/
