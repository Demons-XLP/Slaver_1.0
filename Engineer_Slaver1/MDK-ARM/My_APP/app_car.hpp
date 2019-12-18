#ifndef __APP_CAR_HPP
#define __APP_CAR_HPP
#include "stm32f4xx_hal.h"
#include "bsp_motor.hpp"

extern void Master_Order_Caculate(void);
extern int16_t A[4];

extern void MotorInit();
extern Motor_t DJI_Motor_3508;
extern pid app_car_Claw_pid_In;  //移爪内环
extern pid app_car_Claw_pid_Out; //移爪外环
extern softmotor app_car_Claw_motor;  //移爪电机


#define Claw_Out_1_ON  				HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_SET)  //一级伸爪
#define Claw_Out_1_OFF  			HAL_GPIO_WritePin(Air_Cylinder1_GPIO_Port,Air_Cylinder1_Pin,GPIO_PIN_RESET)  
#define Claw_Out_2_ON  				HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_SET)  //二级伸爪
#define Claw_Out_2_OFF 			  HAL_GPIO_WritePin(Air_Cylinder2_GPIO_Port,Air_Cylinder2_Pin,GPIO_PIN_RESET)
#define OverTurn_Claw_ON 		  HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_SET)  //翻爪
#define OverTurn_Claw_OFF  		HAL_GPIO_WritePin(Air_Cylinder3_GPIO_Port,Air_Cylinder3_Pin,GPIO_PIN_RESET) 
#define Clamp_ON              HAL_GPIO_WritePin(Air_Cylinder4_GPIO_Port,Air_Cylinder4_Pin,GPIO_PIN_SET)  //夹取
#define Clamp_OFF   				  HAL_GPIO_WritePin(Air_Cylinder4_GPIO_Port,Air_Cylinder4_Pin,GPIO_PIN_RESET)
#define Launch_ON   					HAL_GPIO_WritePin(Air_Cylinder5_GPIO_Port,Air_Cylinder5_Pin,GPIO_PIN_SET)  //弹射
#define Launch_OFF   					HAL_GPIO_WritePin(Air_Cylinder5_GPIO_Port,Air_Cylinder5_Pin,GPIO_PIN_RESET)


#define L_Omron Omron[0]   //左边传感器
#define R_Omron Omron[1]  //右边传感器

#endif

