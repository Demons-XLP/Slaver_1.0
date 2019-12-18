#ifndef __APP_CAR_HPP
#define __APP_CAR_HPP
#include "stm32f4xx_hal.h"
#include "bsp_motor.hpp"

extern void Master_Order_Caculate(void);
extern int16_t A[4];

extern void MotorInit();
extern Motor_t DJI_Motor_3508;
extern pid app_car_Claw_pid_In;  //��צ�ڻ�
extern pid app_car_Claw_pid_Out; //��צ�⻷
extern softmotor app_car_Claw_motor;  //��צ���


#define Claw_Out_1_ON  				HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_SET)  //һ����צ
#define Claw_Out_1_OFF  			HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_RESET)  
#define Claw_Out_2_ON  				HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_SET)  //������צ
#define Claw_Out_2_OFF 			  HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_RESET)
#define OverTurn_Claw_ON 		  HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_SET)  //��צ
#define OverTurn_Claw_OFF  		HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_RESET) 
#define Clamp_ON              HAL_GPIO_WritePin(Air_Cylinder4_GPIO_Port,Air_Cylinder4_Pin,GPIO_PIN_SET)  //��ȡ
#define Clamp_OFF   				  HAL_GPIO_WritePin(Air_Cylinder4_GPIO_Port,Air_Cylinder4_Pin,GPIO_PIN_RESET)
#define Launch_ON   					HAL_GPIO_WritePin(Air_Cylinder5_GPIO_Port,Air_Cylinder5_Pin,GPIO_PIN_SET)  //����
#define Launch_OFF   					HAL_GPIO_WritePin(Air_Cylinder5_GPIO_Port,Air_Cylinder5_Pin,GPIO_PIN_RESET)


#define L_Omron Omron[0]   //��ߴ�����
#define R_Omron Omron[1]  //�ұߴ�����

#endif

