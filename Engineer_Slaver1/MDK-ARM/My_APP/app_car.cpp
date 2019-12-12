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

Motor_t DJI_Motor_3508(8192, 19);
pid app_car_Claw_pid_In(1,0,0,0,5000,0,0,0);  //��צ�ڻ�
pid app_car_Claw_pid_Out(1,0,0,0,5000,0,0,0); //��צ�⻷
softmotor app_car_Claw_motor(1,0x203,&DJI_Motor_3508,&app_car_Claw_pid_In,&app_car_Claw_pid_Out);  //��צ���



int16_t A[4];  //����������ط������������Ϣ




/** 
* @brief   ���г�ʼ��
* @remarks 
*/
void MotorInit()
{
  app_car_Claw_motor.Enable_Block(8000,20,1);
}



/** 
* @brief   ִ����������
* @remarks ��������������ִ��
* ��־    
*/
//void Master_Order_Execute(void){
//	/* ������������ */  // ���������жϴ���������ģ�����Ϊ�˼�СCan���ߵ�ѹ����Ҳ������2ms
//	((Master_Order[1]>>0)&1) == 1 ? Claw_Out_1_ON : Claw_Out_1_OFF;       // һ����צ
//	((Master_Order[1]>>1)&1) == 1 ? Claw_Out_2_ON : Claw_Out_2_OFF;       // ������צ
//	((Master_Order[1]>>2)&1) == 1 ? OverTurn_Claw_ON : OverTurn_Claw_OFF; // ��צ
//	((Master_Order[1]>>3)&1) == 1 ? Clamp_ON : Clamp_OFF;                 // ��ȡ
//	((Master_Order[1]>>4)&1) == 1 ? Launch_ON : Launch_OFF;               // ����	
//}

void Master_Order_Caculate(void)
{
	/* ������������ */  // ���������жϴ���������ģ�����Ϊ�˼�СCan���ߵ�ѹ����Ҳ������2ms
	A[0] = Master_Order[0]  << 8 | Master_Order[0];  //�����ؽ��յ��Ĳ�����Ϣ
//	if(A[0] > 0)  //�ж��Ƿ�Ҫ����ȡ��
//	{
//	  Claw_Out_1_ON;  //һ����צ
//		OverTurn_Claw_ON;  //����צ��
//		osDelay(100);  //�ȴ�צ����ȥ
//		Clamp_OFF;  //ץȡ��ҩ��
//		osDelay(500);  //�ȴ��н���ҩ��
//		OverTurn_Claw_OFF;  //����צ��
//		osDelay(1500);  //�ȴ��ӵ��価
//		Clamp_ON;  //�ɿ�צ�ӣ�׼������ҩ���ӵ�
//		osDelay(10);  //�ȴ�צ���ɿ���ҩ��
//		Launch_ON;  //��צ�ӵ����ȥ
//		osDelay(100);  //�ȴ��������
//		Launch_OFF; //�ջص�������
//	}
	if(A[0] > 0)
	{
//	  Claw_Out_1_ON;
//		Claw_Out_2_ON;
//		OverTurn_Claw_ON;
//		Clamp_ON;
//		Launch_ON;
	  Claw_Out_1_ON;  //һ����צ
		OverTurn_Claw_ON;  //����צ��
		osDelay(800);  //�ȴ�צ����ȥ
		Clamp_OFF;  //ץȡ��ҩ��
		osDelay(300);  //�ȴ��н���ҩ��
		OverTurn_Claw_OFF;  //����צ��
		osDelay(1000);  //�ȴ��ӵ��価
		Clamp_ON;  //�ɿ�צ�ӣ�׼������ҩ���ӵ�
		osDelay(50);  //�ȴ�צ���ɿ���ҩ��
		Launch_ON;  //��צ�ӵ����ȥ
		osDelay(100);  //�ȴ��������
		Launch_OFF; //�ջص�������
    

	}
	if(A[0] < 0)
	{

//	  Claw_Out_1_OFF;
//		Claw_Out_2_OFF;
//		OverTurn_Claw_OFF;
//		Clamp_OFF;
//    Launch_OFF;
//		app_car_Claw_motor.Angle_Set(1);  //��צ���Ƶ���һ������ȡ�����������ݶ��ٴ���
//		app_car_Claw_motor.Speed_Set(5000);
	  Claw_Out_1_OFF;
   	app_car_Claw_motor.Safe_Set();
	}
}

