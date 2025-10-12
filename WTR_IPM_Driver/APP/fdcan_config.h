#ifndef __FDCAN_CONFIG_H
#define __FDCAN_CONFIG_H

#include "fdcan.h"
#include "stm32g4xx.h"

// 帧类型定义
typedef enum {
    DATA_FRAME   = 0, // 数据帧
    REMOTE_FRAME = 1, // 远程帧
} CAN_FRAME_TYPE;

// 接收消息结构体定义
typedef struct {
    __IO uint16_t id;       // CANID
    CAN_FRAME_TYPE rtr;     // 远程帧，数据帧
    __IO uint8_t len;       // CAN报文长度
    __IO uint8_t buffer[8]; // CAN报文内容
} CAN_MSG;

// 接收命令类型定义
typedef enum {
    CAN_STOP         = 1, // 停止指令
    CAN_SET_POSITION = 2, // 设置位置指令
    CAN_SET_SPEED    = 3, // 设置速度指令
} CAN_COMMAND_TYPE;

// 接收消息结构体定义
typedef struct {
    __IO float value;              // 接收的值
    __IO CAN_COMMAND_TYPE command; // 接收的命令
} CAN_RX_MSG;

void FDCAN_Config(void);
uint8_t FDCAN_Send_Msg(CAN_MSG *msg);
void FDCAN_Msg_Decode(uint8_t *buf, CAN_RX_MSG *msg);

#endif