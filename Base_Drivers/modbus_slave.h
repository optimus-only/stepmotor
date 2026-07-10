#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "stdint.h"

// Modbus 从机设备地址
#define MODBUS_SLAVE_ADDRESS  0x08

// Modbus 发送缓冲区大小
#define MODBUS_TX_BUF_SIZE    128

// --- 寄存器地址映射表 ---
// 因为速度、位置是 int32_t，每个占用 2 个 16位寄存器 (高字在前，低字在后)
#define REG_GOAL_INTERVAL     0x0000  // 目标间距 
#define REG_GOAL_ACCEL      0x0001  // 目标加速度   最大加速度5000r/ss
#define REG_GOAL_CURRENT      0x0002  //目标电流
#define REG_GOAL_SPEED        0x0003  //目标速度      最大速度50r/s
#define REG_ENABLEOFF_TIME    0x0004  //使能关闭时间
#define REG_STATUS_WORD       0x0005 // 状态字 (只读，反馈电机状态)   0x00,停止 ；0x01,任务完成 ；0x02,任务执行中 ；0x03,过载  ；0x04,	堵转

// 寄存器总数量 (0x0000 ~ 0x000c 共6)
#define MODBUS_REG_NUM        6

// 供底层调用的 API
void Modbus_Init(void);
void Modbus_Receive_Task(uint8_t *rx_data, uint16_t rx_len);
void Modbus_Update_Feedback(void);
// 用户需要自己实现的底层发送函数 (在 main.c 或 uart_mixed.c 中实现)
extern void Modbus_Hardware_Transmit(uint8_t *tx_data, uint16_t tx_len);

#endif