/** 
* @brief    副控功能实现
* @details  关于气缸、电机功能的实现
* @author   Xu LiangPu
* @date      2019.12.8
* @version  0.1
* @par Copyright (c):  RM2020电控
* @par 日志
*				具体使用方法见Readme.md
*				版本变更:
*				2019.10		|		1.0		|		调通CAN总线相关中断处理函数
*				2019.10		|		1.1		|		加入条件编译,自主选择开启某个CAN线
*				2019.10		|		1.2		|		加入条件编译,分为使用信号量和普通中断两种
*				2019.10		|		1.3		|		修改发送相关函数，使得更方便使用
*				2019.10		|		1.4		|		根据代码规范修改了一部分函数名称
*				2019.10		|		1.5		|		参考RM19代码，修改对外接口函数，封装更彻底
*				2019.10		|		1.6		|		修改接收中断和CAN线发送函数逻辑
*				2019.11.11|		1.7		|		添加对初始化结果的检测和更改发送函数的返回类型，并对结果做检测，此次改动是为了适配C++电机库
*				2019.11.15|		1.8		|		在回调参数中固定添加C++电机库的解析函数，现在两个库可以直接捆绑使用无需做过多修改了
*				2019.11.15|		1.9		|		修改了函数注释
*				2019.11.15|		2.0		|		去除了信号量使用的条件编译，现在只需要选择使用哪一个CAN总线了，添加了一个freertos的条件编译使用选项
*/
                                                                                                                                                                                     
#include "app_car.hpp"
#include "main.h"
#include "bsp_my_can.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "task_main.h"

Motor_t DJI_Motor_3508(8192, 19);
pid app_car_Claw_pid_In(1.5,0,0,0,10000,0,0,0);  //移爪内环
pid app_car_Claw_pid_Out(1.9,0.1,20,300,10000,0,0,800); //移爪外环
softmotor app_car_Claw_motor(1,0x201,&DJI_Motor_3508,&app_car_Claw_pid_In,&app_car_Claw_pid_Out);  //移爪电机



int16_t A[4];  //用来存放主控发来解析后的信息
int16_t Slaver_Feedback[4];  //副控反馈给主控的数据
int8_t Flag1  = 1;  //用来初始化抓取机构电机的初始位置判断信息
uint8_t Omron[4] = {0,0,0,0};
float Claw_motor_Origin;  //存放爪子零点
float Claw_TargetAngle;  //爪子目标值
float Proportion = 1;   //爪子位移路程的比例
float Claw_L_Lim,Claw_R_Lim;  //爪子电机的左右边限位
uint8_t app_car_ClawTake_Flag1;  //是否一级取弹判断
uint8_t app_car_ClawTake_Flag2;  //是否二级取弹判断




/** 
* @brief   进行初始化
* @remarks 
*/
void MotorInit()
{
  app_car_Claw_motor.Enable_Block(8000,20,1);
	osDelay(1000);  //等待电机数据初始化
	Claw_motor_Origin = app_car_Claw_motor.SoftAngle;  //初始化零点信息
  Omron[0] = HAL_GPIO_ReadPin(Omron1_GPIO_Port,Omron1_Pin);  //左爪
	Omron[1] = HAL_GPIO_ReadPin(Omron2_GPIO_Port,Omron2_Pin);  
	Omron[2] = HAL_GPIO_ReadPin(Omron3_GPIO_Port,Omron3_Pin);  //右爪
	Omron[3] = HAL_GPIO_ReadPin(Omron4_GPIO_Port,Omron4_Pin);
}



/** 
* @brief  GPIO外部中断的回调函数，扔这里就行，不用管它
* 日志    
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case Omron1_Pin:
			Omron[0] = HAL_GPIO_ReadPin(Omron1_GPIO_Port,Omron1_Pin);		//左爪
			break;
		case Omron2_Pin:
			Omron[1] = HAL_GPIO_ReadPin(Omron2_GPIO_Port,Omron2_Pin);   
			break;
		case Omron3_Pin:           
			Omron[2] = HAL_GPIO_ReadPin(Omron3_GPIO_Port,Omron3_Pin);  //右爪
			break;
		case Omron4_Pin:
			Omron[3] = HAL_GPIO_ReadPin(Omron4_GPIO_Port,Omron4_Pin);
			break;				
	}
}

/** 
* @brief   解析主机发送数据并自行根据判断执行命令
* @remarks 在主任务中周期执行
* 日志    
*/


 void Master_Order_Caculate(void)
{
	/* 主机控制气缸 */  // 本来是在中断处理函数里面的，但是为了减小Can总线的压力，也不差这2ms
	A[0] = Master_Order[0]  << 8 | Master_Order[1];  //从主控接收到的拨轮信息
	A[1] = Master_Order[2]  << 8 | Master_Order[3];  //控制抓取机构电机左右移动
	A[2] = Master_Order[4]  << 8 | Master_Order[5];  //模式设置
	A[3] = Master_Order[6]  << 8 | Master_Order[7];  //CH1

	
	switch (A[2])  //模式判断
	{
		
case 32:  //限位初始化，左中右下
			if(app_car_Claw_motor.block->IsBlock == 1)
				 {
						app_car_Claw_motor.Safe_Set();  //堵转停止
						app_car_Claw_motor.block->Clear_BlockFlag();
						Claw_motor_Origin = app_car_Claw_motor.SoftAngle;  //重新初始化零点
						if(Claw_motor_Origin < 0)  //左边限位
							{
								Claw_L_Lim = Claw_motor_Origin;   //记录左限位
								Claw_TargetAngle = Claw_motor_Origin + 1100.f;  //回到中间取弹位置的目标值
							}
						else  //右边限位
							{
								Claw_R_Lim = Claw_motor_Origin;  //记录右限位
								Claw_TargetAngle = Claw_motor_Origin - 1220.f;  //回到中间取弹位置的目标值
							}
					}

	       Claw_TargetAngle += A[1] * 0.002;
			   app_car_Claw_motor.Angle_Set(Claw_TargetAngle);  //控制爪子电机左右移动  
         app_car_ClawTake_Flag1 = ENDING;
   
break;
case 12 :  //副控取弹的模式，左上右下
			
			if(A[0] > 0)  //拨轮上   左边箱
			   {
				  
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f);  //限位
					 Claw_TargetAngle = Claw_L_Lim + 50.f;
					
			   }
		  else if(A[0] < 0)  //拨轮下  右边箱
			   {
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					 Claw_TargetAngle = Claw_L_Lim + 1900.641f;
			   }
			if(A[3] > 0)  //右边通道     上  中间箱 
				{
					app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					Claw_TargetAngle = Claw_L_Lim + 978.838f;
				}
				
			else if(A[3] < 0)  //传感器没检测到弹药箱，即可以取弹了
				{
				  app_car_ClawTake_Flag1 = RUNNING;  //一级取弹开始
				}
				app_car_Claw_motor.Angle_Set(Claw_TargetAngle);
break;
case 13:    //自动对位取弹，左上右中
	      app_car_ClawTake_Flag1 = ENDING;
break;			
case 22:  //安全模式
	    app_car_Claw_motor.Safe_Set();
			app_car_ClawTake_Flag1 = ENDING;
break;
case 33:  
			if(A[3] > 0)  //右边通道     上  中间箱 
				{
					app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					Claw_TargetAngle = Claw_L_Lim + 978.838f;
				}
				app_car_Claw_motor.Angle_Set(Claw_TargetAngle);
				app_car_ClawTake_Flag1 = ENDING;
break;
case   11:  //二级取弹
				{
					if(A[0] > 0)  //拨轮上   左边箱
			   {
				  
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f);  //限位
					 Claw_TargetAngle = Claw_L_Lim + 50.f;
					
			   }
		  else if(A[0] < 0)  //拨轮下  右边箱
			   {
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					 Claw_TargetAngle = Claw_L_Lim + 1900.641f;
			   }
			if(A[3] > 0)  //右边通道     上  中间箱 
				{
					app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					Claw_TargetAngle = Claw_L_Lim + 978.838f;
				}
				
			else if(A[3] < 0)  //传感器没检测到弹药箱，即可以取弹了
				{
				  app_car_ClawTake_Flag2 = RUNNING;  //一级取弹开始
				}
				 app_car_Claw_motor.Angle_Set(Claw_TargetAngle);
				}
break;
default:
	    app_car_Claw_motor.Safe_Set();
			app_car_ClawTake_Flag1 = ENDING;
break;
}

}

