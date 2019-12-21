#include "task_main.h"
#include "bsp_can.hpp"
#include "bsp_motor.hpp"
#include "bsp_adc_deal.h"
#include "cmsis_os.h"
#include "app_car.hpp"


/** 
* @brief   一级取弹任务函数
* @remarks 取第一排弹药箱
*           type 运行参数，即是否要运行该模式
* 日志    
*/
void CaissonTake_FirstRow(uint8_t type)
{
	switch(type)
	{
		case ENDING:
			Claw_Out_1_OFF;
    break;
    case RUNNING:
			Claw_Out_1_ON;  //一级伸爪
			osDelay(150);
			OverTurn_Claw_ON;  //翻出爪子
			osDelay(500);  //等待爪子下去
			Clamp_OFF;  //抓取弹药箱
			osDelay(100);  //等待夹紧弹药箱
			OverTurn_Claw_OFF;  //翻回爪子
			osDelay(1500);  //等待子弹落尽
			Clamp_ON;  //松开爪子，准备将弹药箱扔掉
			osDelay(100);  //等待爪子松开弹药箱
			Launch_ON;  //将爪子弹射出去
			osDelay(700);  //等待弹射完成
			Launch_OFF; //收回弹射气缸
		  app_car_ClawTake_Flag1 = ENDING;
		break;
		default:
		break;
	}
  
}

/** 
* @brief   一级取弹任务函数
* @remarks 取第一排弹药箱
*           type 运行参数，即是否要运行该模式
* 日志    
*/
void CaissonTake_SecondRow(uint8_t type)
{
	switch(type)
	{
		case ENDING:
			Claw_Out_1_OFF;
		  Claw_Out_2_OFF;
    break;
    case RUNNING:
			Claw_Out_1_ON;  //一级伸爪
			osDelay(150);
			Claw_Out_2_ON;  //二级伸爪
			osDelay(150);
			OverTurn_Claw_ON;  //翻出爪子
			osDelay(500);  //等待爪子下去
			Clamp_OFF;  //抓取弹药箱
			osDelay(100);  //等待夹紧弹药箱
			OverTurn_Claw_OFF;  //翻回爪子
			osDelay(1500);  //等待子弹落尽
			Clamp_ON;  //松开爪子，准备将弹药箱扔掉
			osDelay(100);  //等待爪子松开弹药箱
			Launch_ON;  //将爪子弹射出去
			osDelay(700);  //等待弹射完成
			Launch_OFF; //收回弹射气缸
		  app_car_ClawTake_Flag2 = ENDING;
		break;
		default:
		break;
	}
  
}



void Caisson_TakeTask(void const *argument)
{
  static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取当前的系统时间
	for(;;)
	{
		CaissonTake_FirstRow(app_car_ClawTake_Flag1);  //一级取弹
		CaissonTake_SecondRow(app_car_ClawTake_Flag2);  //二级取弹
		vTaskDelayUntil(&xLastWakeTime,5/portTICK_PERIOD_MS); //定周期延时
	}
}



void MainTask(void const * argument)
{
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取当前的系统时间
	
  bsp_ADC_Sensor_Init();  //夏普红外初始化
	bsp_can_Init();  //CAN初始化
  manager::CANSelect(&hcan1,&hcan2);  //选择CAN1和CAN2
	MotorInit();  //电机相关初始化
	osThreadDef(Caisson_TakeTask,Caisson_TakeTask,osPriorityAboveNormal,0,128);
	osThreadCreate(osThread(Caisson_TakeTask),NULL);
	
	for(;;)
	{
		Master_Order_Caculate();   //解算主控发来的命令并执行
	  manager::CANSend();  //电机托管
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS); //定周期延时
	}
	
}
 


