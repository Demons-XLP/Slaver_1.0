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
pid app_car_Claw_pid_In(1,0,0,0,5000,0,0,0);  //移爪内环
pid app_car_Claw_pid_Out(1,0,0,0,5000,0,0,0); //移爪外环
softmotor app_car_Claw_motor(1,0x203,&DJI_Motor_3508,&app_car_Claw_pid_In,&app_car_Claw_pid_Out);  //移爪电机



int16_t A[4];  //用来存放主控发来解析后的信息




/** 
* @brief   进行初始化
* @remarks 
*/
void MotorInit()
{
  app_car_Claw_motor.Enable_Block(8000,20,1);
}



/** 
* @brief   执行主机命令
* @remarks 在主任务中周期执行
* 日志    
*/
//void Master_Order_Execute(void){
//	/* 主机控制气缸 */  // 本来是在中断处理函数里面的，但是为了减小Can总线的压力，也不差这2ms
//	((Master_Order[1]>>0)&1) == 1 ? Claw_Out_1_ON : Claw_Out_1_OFF;       // 一级伸爪
//	((Master_Order[1]>>1)&1) == 1 ? Claw_Out_2_ON : Claw_Out_2_OFF;       // 二级伸爪
//	((Master_Order[1]>>2)&1) == 1 ? OverTurn_Claw_ON : OverTurn_Claw_OFF; // 翻爪
//	((Master_Order[1]>>3)&1) == 1 ? Clamp_ON : Clamp_OFF;                 // 夹取
//	((Master_Order[1]>>4)&1) == 1 ? Launch_ON : Launch_OFF;               // 弹射	
//}

void Master_Order_Caculate(void)
{
	/* 主机控制气缸 */  // 本来是在中断处理函数里面的，但是为了减小Can总线的压力，也不差这2ms
	A[0] = Master_Order[0]  << 8 | Master_Order[0];  //从主控接收到的拨轮信息
//	if(A[0] > 0)  //判断是否要进行取弹
//	{
//	  Claw_Out_1_ON;  //一级伸爪
//		OverTurn_Claw_ON;  //翻出爪子
//		osDelay(100);  //等待爪子下去
//		Clamp_OFF;  //抓取弹药箱
//		osDelay(500);  //等待夹紧弹药箱
//		OverTurn_Claw_OFF;  //翻回爪子
//		osDelay(1500);  //等待子弹落尽
//		Clamp_ON;  //松开爪子，准备将弹药箱扔掉
//		osDelay(10);  //等待爪子松开弹药箱
//		Launch_ON;  //将爪子弹射出去
//		osDelay(100);  //等待弹射完成
//		Launch_OFF; //收回弹射气缸
//	}
	if(A[0] > 0)
	{
//	  Claw_Out_1_ON;
//		Claw_Out_2_ON;
//		OverTurn_Claw_ON;
//		Clamp_ON;
//		Launch_ON;
	  Claw_Out_1_ON;  //一级伸爪
		OverTurn_Claw_ON;  //翻出爪子
		osDelay(800);  //等待爪子下去
		Clamp_OFF;  //抓取弹药箱
		osDelay(300);  //等待夹紧弹药箱
		OverTurn_Claw_OFF;  //翻回爪子
		osDelay(1000);  //等待子弹落尽
		Clamp_ON;  //松开爪子，准备将弹药箱扔掉
		osDelay(50);  //等待爪子松开弹药箱
		Launch_ON;  //将爪子弹射出去
		osDelay(100);  //等待弹射完成
		Launch_OFF; //收回弹射气缸
    

	}
	if(A[0] < 0)
	{

//	  Claw_Out_1_OFF;
//		Claw_Out_2_OFF;
//		OverTurn_Claw_OFF;
//		Clamp_OFF;
//    Launch_OFF;
//		app_car_Claw_motor.Angle_Set(1);  //将爪子移到下一个区域取弹，具体数据多少待测
//		app_car_Claw_motor.Speed_Set(5000);
	  Claw_Out_1_OFF;
   	app_car_Claw_motor.Safe_Set();
	}
}

