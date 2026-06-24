#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "stdint.h"

// Modbus 从机设备地址
#define MODBUS_SLAVE_ADDRESS  0x08

// Modbus 发送缓冲区大小
#define MODBUS_TX_BUF_SIZE    128

// --- 寄存器地址映射表 ---
// 因为速度、位置是 int32_t，每个占用 2 个 16位寄存器 (高字在前，低字在后)
#define REG_GOAL_SPEED_H      0x0000  // 目标速度 高16位
#define REG_GOAL_SPEED_L      0x0001  // 目标速度 低16位
#define REG_GOAL_ACCEL_H      0x0002  // 目标加速度 高16位
#define REG_GOAL_ACCEL_L      0x0003  // 目标加速度 低16位
#define REG_LEFT_LIMIT_H      0x0004  // 左极限 高16位
#define REG_LEFT_LIMIT_L      0x0005  // 左极限 低16位
#define REG_RIGHT_LIMIT_H     0x0006  // 右极限 高16位
#define REG_RIGHT_LIMIT_L     0x0007  // 右极限 低16位

#define REG_CONTROL_WORD      0x0008  // 控制字 (写1:开始找极限, 写2:停止)
#define REG_STATUS_WORD       0x0009  // 状态字 (只读，反馈电机状态)

// === 新增：当前位置反馈寄存器 ===
#define REG_CURRENT_POS_H     0x000A  // 当前位置 高16位 (地址 10)
#define REG_CURRENT_POS_L     0x000B  // 当前位置 低16位 (地址 11)

// 寄存器总数量 (0x0000 ~ 0x000B 共12个)
#define MODBUS_REG_NUM        12

// 供底层调用的 API
void Modbus_Init(void);
void Modbus_Receive_Task(uint8_t *rx_data, uint16_t rx_len);
void Modbus_Update_Feedback(void);
// 用户需要自己实现的底层发送函数 (在 main.c 或 uart_mixed.c 中实现)
extern void Modbus_Hardware_Transmit(uint8_t *tx_data, uint16_t tx_len);

#endif