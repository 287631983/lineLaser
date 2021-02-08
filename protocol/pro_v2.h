/**
 * @file         pro_v2.h
 * @author       Weyne Chen
 * @version      V01
 * @date         2020.03.17
 * @brief
 * @note
 * @attention    COPYRIGHT WEYNE
 **/

#ifndef __PRO_V2_H
#define __PRO_V2_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C"
{
#endif
//解包buffer大小
#define PARSE_LEN        500
//雷达地址
#define LIDAR_ADDRESS    0x10
//最小数据长度
#define MIN_PRO_NUM      14

//数据包头尾、控制字
#define P_HEADER         0xAA
#define P_TAIL           0x55
#define P_CTRL           0xA5

typedef struct
{
    uint8_t  device_address;
    uint8_t  function_code;
    uint16_t start_address;
    uint32_t len;
} SdkProtocolHeaderTypeDef;

typedef struct
{
    uint8_t  data_id;
    uint8_t  *data_in_buff;
    uint32_t data_in_len;
    uint8_t  *data_out_buff;
    uint32_t *data_out_len;
    uint16_t offset;
} PackageDataStruct;

bool Package(PackageDataStruct package);
bool Unpacking(PackageDataStruct *package);

#ifdef __cplusplus
}
#endif
#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
