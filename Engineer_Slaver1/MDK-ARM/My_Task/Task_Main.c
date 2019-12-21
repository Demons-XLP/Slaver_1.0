#include "task_main.h"
#include "bsp_can.hpp"
#include "bsp_motor.hpp"
#include "bsp_adc_deal.h"
#include "cmsis_os.h"
#include "app_car.hpp"


/** 
* @brief   һ��ȡ��������
* @remarks ȡ��һ�ŵ�ҩ��
*           type ���в��������Ƿ�Ҫ���и�ģʽ
* ��־    
*/
void CaissonTake_FirstRow(uint8_t type)
{
	switch(type)
	{
		case ENDING:
			Claw_Out_1_OFF;
    break;
    case RUNNING:
			Claw_Out_1_ON;  //һ����צ
			osDelay(150);
			OverTurn_Claw_ON;  //����צ��
			osDelay(500);  //�ȴ�צ����ȥ
			Clamp_OFF;  //ץȡ��ҩ��
			osDelay(100);  //�ȴ��н���ҩ��
			OverTurn_Claw_OFF;  //����צ��
			osDelay(1500);  //�ȴ��ӵ��価
			Clamp_ON;  //�ɿ�צ�ӣ�׼������ҩ���ӵ�
			osDelay(100);  //�ȴ�צ���ɿ���ҩ��
			Launch_ON;  //��צ�ӵ����ȥ
			osDelay(700);  //�ȴ��������
			Launch_OFF; //�ջص�������
		  app_car_ClawTake_Flag1 = ENDING;
		break;
		default:
		break;
	}
  
}

/** 
* @brief   һ��ȡ��������
* @remarks ȡ��һ�ŵ�ҩ��
*           type ���в��������Ƿ�Ҫ���и�ģʽ
* ��־    
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
			Claw_Out_1_ON;  //һ����צ
			osDelay(150);
			Claw_Out_2_ON;  //������צ
			osDelay(150);
			OverTurn_Claw_ON;  //����צ��
			osDelay(500);  //�ȴ�צ����ȥ
			Clamp_OFF;  //ץȡ��ҩ��
			osDelay(100);  //�ȴ��н���ҩ��
			OverTurn_Claw_OFF;  //����צ��
			osDelay(1500);  //�ȴ��ӵ��価
			Clamp_ON;  //�ɿ�צ�ӣ�׼������ҩ���ӵ�
			osDelay(100);  //�ȴ�צ���ɿ���ҩ��
			Launch_ON;  //��צ�ӵ����ȥ
			osDelay(700);  //�ȴ��������
			Launch_OFF; //�ջص�������
		  app_car_ClawTake_Flag2 = ENDING;
		break;
		default:
		break;
	}
  
}



void Caisson_TakeTask(void const *argument)
{
  static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡ��ǰ��ϵͳʱ��
	for(;;)
	{
		CaissonTake_FirstRow(app_car_ClawTake_Flag1);  //һ��ȡ��
		CaissonTake_SecondRow(app_car_ClawTake_Flag2);  //����ȡ��
		vTaskDelayUntil(&xLastWakeTime,5/portTICK_PERIOD_MS); //��������ʱ
	}
}



void MainTask(void const * argument)
{
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡ��ǰ��ϵͳʱ��
	
  bsp_ADC_Sensor_Init();  //���պ����ʼ��
	bsp_can_Init();  //CAN��ʼ��
  manager::CANSelect(&hcan1,&hcan2);  //ѡ��CAN1��CAN2
	MotorInit();  //�����س�ʼ��
	osThreadDef(Caisson_TakeTask,Caisson_TakeTask,osPriorityAboveNormal,0,128);
	osThreadCreate(osThread(Caisson_TakeTask),NULL);
	
	for(;;)
	{
		Master_Order_Caculate();   //�������ط��������ִ��
	  manager::CANSend();  //����й�
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS); //��������ʱ
	}
	
}
 


