/** 
* @brief    ���ع���ʵ��
* @details  �������ס�������ܵ�ʵ��
* @author   Xu LiangPu
* @date      2019.12.8
* @version  0.1
* @par Copyright (c):  RM2020���
* @par ��־
*				����ʹ�÷�����Readme.md
*				�汾���:
*				2019.10		|		1.0		|		��ͨCAN��������жϴ�����
*				2019.10		|		1.1		|		������������,����ѡ����ĳ��CAN��
*				2019.10		|		1.2		|		������������,��Ϊʹ���ź�������ͨ�ж�����
*				2019.10		|		1.3		|		�޸ķ�����غ�����ʹ�ø�����ʹ��
*				2019.10		|		1.4		|		���ݴ���淶�޸���һ���ֺ�������
*				2019.10		|		1.5		|		�ο�RM19���룬�޸Ķ���ӿں�������װ������
*				2019.10		|		1.6		|		�޸Ľ����жϺ�CAN�߷��ͺ����߼�
*				2019.11.11|		1.7		|		��ӶԳ�ʼ������ļ��͸��ķ��ͺ����ķ������ͣ����Խ������⣬�˴θĶ���Ϊ������C++�����
*				2019.11.15|		1.8		|		�ڻص������й̶����C++�����Ľ����������������������ֱ������ʹ�������������޸���
*				2019.11.15|		1.9		|		�޸��˺���ע��
*				2019.11.15|		2.0		|		ȥ�����ź���ʹ�õ��������룬����ֻ��Ҫѡ��ʹ����һ��CAN�����ˣ������һ��freertos����������ʹ��ѡ��
*/
                                                                                                                                                                                     
#include "app_car.hpp"
#include "main.h"
#include "bsp_my_can.h"
#include "cmsis_os.h"
#include "bsp_motor.hpp"
#include "task_main.h"

Motor_t DJI_Motor_3508(8192, 19);
pid app_car_Claw_pid_In(1.5,0,0,0,10000,0,0,0);  //��צ�ڻ�
pid app_car_Claw_pid_Out(1.9,0.1,20,300,10000,0,0,800); //��צ�⻷
softmotor app_car_Claw_motor(1,0x201,&DJI_Motor_3508,&app_car_Claw_pid_In,&app_car_Claw_pid_Out);  //��צ���



int16_t A[4];  //����������ط������������Ϣ
int16_t Slaver_Feedback[4];  //���ط��������ص�����
int8_t Flag1  = 1;  //������ʼ��ץȡ��������ĳ�ʼλ���ж���Ϣ
uint8_t Omron[4] = {0,0,0,0};
float Claw_motor_Origin;  //���צ�����
float Claw_TargetAngle;  //צ��Ŀ��ֵ
float Proportion = 1;   //צ��λ��·�̵ı���
float Claw_L_Lim,Claw_R_Lim;  //צ�ӵ�������ұ���λ
uint8_t app_car_ClawTake_Flag1;  //�Ƿ�һ��ȡ���ж�
uint8_t app_car_ClawTake_Flag2;  //�Ƿ����ȡ���ж�




/** 
* @brief   ���г�ʼ��
* @remarks 
*/
void MotorInit()
{
  app_car_Claw_motor.Enable_Block(8000,20,1);
	osDelay(1000);  //�ȴ�������ݳ�ʼ��
	Claw_motor_Origin = app_car_Claw_motor.SoftAngle;  //��ʼ�������Ϣ
  Omron[0] = HAL_GPIO_ReadPin(Omron1_GPIO_Port,Omron1_Pin);  //��צ
	Omron[1] = HAL_GPIO_ReadPin(Omron2_GPIO_Port,Omron2_Pin);  
	Omron[2] = HAL_GPIO_ReadPin(Omron3_GPIO_Port,Omron3_Pin);  //��צ
	Omron[3] = HAL_GPIO_ReadPin(Omron4_GPIO_Port,Omron4_Pin);
}



/** 
* @brief  GPIO�ⲿ�жϵĻص���������������У����ù���
* ��־    
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
		case Omron1_Pin:
			Omron[0] = HAL_GPIO_ReadPin(Omron1_GPIO_Port,Omron1_Pin);		//��צ
			break;
		case Omron2_Pin:
			Omron[1] = HAL_GPIO_ReadPin(Omron2_GPIO_Port,Omron2_Pin);   
			break;
		case Omron3_Pin:           
			Omron[2] = HAL_GPIO_ReadPin(Omron3_GPIO_Port,Omron3_Pin);  //��צ
			break;
		case Omron4_Pin:
			Omron[3] = HAL_GPIO_ReadPin(Omron4_GPIO_Port,Omron4_Pin);
			break;				
	}
}

/** 
* @brief   ���������������ݲ����и����ж�ִ������
* @remarks ��������������ִ��
* ��־    
*/


 void Master_Order_Caculate(void)
{
	/* ������������ */  // ���������жϴ���������ģ�����Ϊ�˼�СCan���ߵ�ѹ����Ҳ������2ms
	A[0] = Master_Order[0]  << 8 | Master_Order[1];  //�����ؽ��յ��Ĳ�����Ϣ
	A[1] = Master_Order[2]  << 8 | Master_Order[3];  //����ץȡ������������ƶ�
	A[2] = Master_Order[4]  << 8 | Master_Order[5];  //ģʽ����
	A[3] = Master_Order[6]  << 8 | Master_Order[7];  //CH1

	
	switch (A[2])  //ģʽ�ж�
	{
		
case 32:  //��λ��ʼ������������
			if(app_car_Claw_motor.block->IsBlock == 1)
				 {
						app_car_Claw_motor.Safe_Set();  //��תֹͣ
						app_car_Claw_motor.block->Clear_BlockFlag();
						Claw_motor_Origin = app_car_Claw_motor.SoftAngle;  //���³�ʼ�����
						if(Claw_motor_Origin < 0)  //�����λ
							{
								Claw_L_Lim = Claw_motor_Origin;   //��¼����λ
								Claw_TargetAngle = Claw_motor_Origin + 1100.f;  //�ص��м�ȡ��λ�õ�Ŀ��ֵ
							}
						else  //�ұ���λ
							{
								Claw_R_Lim = Claw_motor_Origin;  //��¼����λ
								Claw_TargetAngle = Claw_motor_Origin - 1220.f;  //�ص��м�ȡ��λ�õ�Ŀ��ֵ
							}
					}

	       Claw_TargetAngle += A[1] * 0.002;
			   app_car_Claw_motor.Angle_Set(Claw_TargetAngle);  //����צ�ӵ�������ƶ�  
         app_car_ClawTake_Flag1 = ENDING;
   
break;
case 12 :  //����ȡ����ģʽ����������
			
			if(A[0] > 0)  //������   �����
			   {
				  
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f);  //��λ
					 Claw_TargetAngle = Claw_L_Lim + 50.f;
					
			   }
		  else if(A[0] < 0)  //������  �ұ���
			   {
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					 Claw_TargetAngle = Claw_L_Lim + 1900.641f;
			   }
			if(A[3] > 0)  //�ұ�ͨ��     ��  �м��� 
				{
					app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					Claw_TargetAngle = Claw_L_Lim + 978.838f;
				}
				
			else if(A[3] < 0)  //������û��⵽��ҩ�䣬������ȡ����
				{
				  app_car_ClawTake_Flag1 = RUNNING;  //һ��ȡ����ʼ
				}
				app_car_Claw_motor.Angle_Set(Claw_TargetAngle);
break;
case 13:    //�Զ���λȡ������������
	      app_car_ClawTake_Flag1 = ENDING;
break;			
case 22:  //��ȫģʽ
	    app_car_Claw_motor.Safe_Set();
			app_car_ClawTake_Flag1 = ENDING;
break;
case 33:  
			if(A[3] > 0)  //�ұ�ͨ��     ��  �м��� 
				{
					app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					Claw_TargetAngle = Claw_L_Lim + 978.838f;
				}
				app_car_Claw_motor.Angle_Set(Claw_TargetAngle);
				app_car_ClawTake_Flag1 = ENDING;
break;
case   11:  //����ȡ��
				{
					if(A[0] > 0)  //������   �����
			   {
				  
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f);  //��λ
					 Claw_TargetAngle = Claw_L_Lim + 50.f;
					
			   }
		  else if(A[0] < 0)  //������  �ұ���
			   {
					 app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					 Claw_TargetAngle = Claw_L_Lim + 1900.641f;
			   }
			if(A[3] > 0)  //�ұ�ͨ��     ��  �м��� 
				{
					app_car_Claw_motor.Limit(Claw_R_Lim - 80.f,Claw_L_Lim + 80.f); 
					Claw_TargetAngle = Claw_L_Lim + 978.838f;
				}
				
			else if(A[3] < 0)  //������û��⵽��ҩ�䣬������ȡ����
				{
				  app_car_ClawTake_Flag2 = RUNNING;  //һ��ȡ����ʼ
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

