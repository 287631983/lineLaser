/**
 * @file        pro.h
 * @author      陈维
 * @version     V01
 * @date        2016.09.21
 * @brief       协议定义
 * @note
 *
 * @attention   COYPRIGHT INMOTION ROBOT
 **/

#ifndef _PRO_H_
#define _PRO_H_

/**
 * @brief  数据包ID
 */
typedef enum
{
    PACK_NULL = 0,
    PACK_GET_POS,
    PACK_MOTOR_RUN,
    PACK_SET_LED,
    PACK_MOTOR_STOP,
    PACK_POS,
} PackageIDTypeDef;



#endif



/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
