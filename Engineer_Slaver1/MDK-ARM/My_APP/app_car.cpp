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
* @brief   一级取弹任务函数
* @remarks 取第一排弹药箱
* 日志    
*/
void Caisson_Take(void)
{
	
  Claw_Out_1_ON;  //一级伸爪
	osDelay(150);
//	Claw_Out_2_ON;  //二级伸爪
//	osDelay(150);
	OverTurn_Claw_ON;  //翻出爪子
	osDelay(500);  //等待爪子下去
	Clamp_OFF;  //抓取弹药箱
	osDelay(100);  //等待夹紧弹药箱
	OverTurn_Claw_OFF;  //翻回爪子
	osDelay(1000);  //等待子弹落尽
	Clamp_ON;  //松开爪子，准备将弹药箱扔掉
	osDelay(100);  //等待爪子松开弹药箱
	Launch_ON;  //将爪子弹射出去
	osDelay(200);  //等待弹射完成
	Launch_OFF; //收回弹射气缸
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
		
case 31:
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

   
break;
case 11 :  //副控取弹的模式
			
			if(A[0] > 0)
			   {
				  
//					 Claw_Out_1_ON;
//					Claw_Out_2_ON;
						app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f);  //限位
					 Claw_TargetAngle = Claw_L_Lim + 50.f;
					 Caisson_Take();
					
			   }
			else if(A[3] > 0)
			{
				
				app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
			  Claw_TargetAngle = Claw_L_Lim + 978.838f;
				Caisson_Take();
			}
			 else if(A[0] < 0)
			   {
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					 Claw_TargetAngle = Claw_L_Lim + 1900.641f;
//				  Claw_Out_1_OFF;
//					Claw_Out_2_OFF;
					 Caisson_Take();
			   }
				 


				app_car_Claw_motor.Angle_Set(Claw_TargetAngle);
break;
	
case 22:  //安全模式
	    app_car_Claw_motor.Safe_Set();
break;
default:
	    app_car_Claw_motor.Safe_Set();
break;
}

}

