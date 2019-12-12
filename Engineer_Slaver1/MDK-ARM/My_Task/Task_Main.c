#include "task_main.h"
#include "bsp_can.hpp"
#include "bsp_motor.hpp"
#include "bsp_adc_deal.h"
#include "cmsis_os.h"
#include "app_car.hpp"





void MainTask(void const * argument)
{
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡ��ǰ��ϵͳʱ��
	
	MotorInit();  //�����س�ʼ��
  bsp_ADC_Sensor_Init();  //���պ����ʼ��
	bsp_can_Init();  //CAN��ʼ��
  manager::CANSelect(&hcan1,&hcan2);  //ѡ��CAN1��CAN2
	
	for(;;)
	{
		Master_Order_Caculate();   //�������ط��������ִ��
	  manager::CANSend();  //����й�
		vTaskDelayUntil(&xLastWakeTime,1/portTICK_PERIOD_MS); //��������ʱ
	}
	
}
 
