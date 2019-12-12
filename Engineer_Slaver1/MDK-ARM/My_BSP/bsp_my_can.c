/** 
	* @brief    CAN
	* @details  CAN回调函数
	* @author   Xu LiangPu
	* @date     2019.12.6 
	* @version  1.0
	* @par 
	* @par 日志
*/

#include "bsp_my_can.h"
#include "can.h"
#include "bsp_motor.hpp"
#include "string.h"

#define MasterCan hcan2  //主副控通信为can2通信
#define MasterID 0x101  //主控标识符ID
#define SlaverID 0x102  //副控标识符ID
CAN_RxHeaderTypeDef RxHead;

uint8_t Master_Order[8];
static float data[8];//接收数据缓冲区


/**
* @brief  CAN接收中断
* @details  重新定义接收中断，会自动在CAN中断中调用
*           为了使副控能接收到主控的命令，在外部重新定义的CAN接收中断
* @param  NULL
* @retval  NULL
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, (uint8_t*)data);
	if(hcan == &MasterCan)
	{
		if(RxHead.StdId == MasterID)
		{
		  memcpy(Master_Order,data,8);
		}
		else manager::CANUpdate(hcan, &RxHead, (uint8_t*)data);
	}
	else manager::CANUpdate(hcan, &RxHead, (uint8_t*)data);
}


