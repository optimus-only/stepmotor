#include "modbus_slave.h"
#include <string.h>
#include "loop.h"
#include "motor_control.h"
#include "control_config.h"
#include "uart_mixed.h"
#include "Location_Tracker.h"
extern void Motor_Control_Write_Goal_Speed(int32_t speed);
extern void Motor_LimitFinder_Start(void);
uint16_t auto_stop_time=100;
uint16_t Modbus_RegPool[MODBUS_REG_NUM] = {0};
uint8_t Modbus_TxBuf[MODBUS_TX_BUF_SIZE];
uint8_t Debug_Error_Frame[16]; 
uint16_t Debug_Error_Len = 0;

// CRC16 查表法/计算法实现 (计算法省内存)
static uint16_t Modbus_CRC16(uint8_t *puchMsg, uint16_t usDataLen) 
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < usDataLen; i++) {
        crc ^= puchMsg[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// 辅助函数：将两个 16 位寄存器组合成 32 位有符号整数
static int32_t Combine_To_Int32(uint16_t high_word, uint16_t low_word)
{
    return (int32_t)(((uint32_t)high_word << 16) | low_word);
}

// 处理具体的数据下发动作 
static void Modbus_Execute_Action(uint16_t reg_addr)
{
    int32_t val32;
    switch(reg_addr)
    {
			
			 case REG_GOAL_INTERVAL :  
//            val32=Modbus_RegPool[REG_GOAL_INTERVAL];					
				    val32=(limit_finder.min_pos_raw+limit_finder.max_pos_raw-Modbus_RegPool[REG_GOAL_INTERVAL])/2;
				    if(val32>(limit_finder.min_pos_raw + LIMIT_BACKOFF_DIST))
						{ limit_finder.safe_min_pos = val32;}
						
						val32=(limit_finder.min_pos_raw+limit_finder.max_pos_raw+Modbus_RegPool[REG_GOAL_INTERVAL])/2;
				    if(val32<(limit_finder.max_pos_raw - LIMIT_BACKOFF_DIST))
						{limit_finder.safe_max_pos = val32; }
            break;
       
        case REG_GOAL_ACCEL:
            val32 = Modbus_RegPool[REG_GOAL_ACCEL]*Move_Pulse_NUM;
				    if(val32<Move_Rated_UpAcc)
						{
							Location_Tracker_Set_UpAcc(val32);
				      Location_Tracker_Set_DownAcc(val32);
						} 
            break;
						
        case REG_GOAL_CURRENT:
           val32=Modbus_RegPool[REG_GOAL_CURRENT];
			    	Motor_Control_Write_Realstic_Current(val32);
            break;
            
        
            
        case REG_GOAL_SPEED :
            val32=Modbus_RegPool[REG_GOAL_SPEED]*Move_Pulse_NUM;
				    Location_Tracker_Set_MaxSpeed(val32);
            break;
            
//        case :
//            
//            break;
				case REG_ENABLEOFF_TIME:
					   auto_stop_time=Modbus_RegPool[REG_ENABLEOFF_TIME];
					 
				    break;
    }
}

// 解析 Modbus RTU 接收帧
void Modbus_Receive_Task(uint8_t *rx_data, uint16_t rx_len)
{
    if (rx_len < 8) return; // 帧太短
    if (rx_data[0] != MODBUS_SLAVE_ADDRESS) return; // 地址不匹配
    // 校验 CRC
    uint16_t crc_recv = (rx_data[rx_len - 1] << 8) | rx_data[rx_len - 2];
    uint16_t crc_calc = Modbus_CRC16(rx_data, rx_len - 2);
    if (crc_recv != crc_calc) {
        // 大小端不同的话，CRC可能是 rx_data[len-2]<<8 | rx_data[len-1]
        crc_recv = (rx_data[rx_len - 2] << 8) | rx_data[rx_len - 1];
        if (crc_recv != crc_calc) return; // CRC 错误
    }

    uint8_t function_code = rx_data[1];
    uint16_t start_addr = (rx_data[2] << 8) | rx_data[3];
    uint16_t tx_len = 0;

    switch (function_code)
    {
        case 0x03: 
        {
            uint16_t read_cnt = (rx_data[4] << 8) | rx_data[5];
            if (start_addr + read_cnt > MODBUS_REG_NUM) return; // 越界
            
            // 发送数据打包
            Modbus_TxBuf[0] = MODBUS_SLAVE_ADDRESS;
            Modbus_TxBuf[1] = 0x03;
            Modbus_TxBuf[2] = read_cnt * 2; // 字节数
            tx_len = 3;
                  
            for (uint16_t i = 0; i < read_cnt; i++) {
                Modbus_TxBuf[tx_len++] = Modbus_RegPool[start_addr + i] >> 8;
                Modbus_TxBuf[tx_len++] = Modbus_RegPool[start_addr + i] & 0xFF;
            }
            break;
        }
        case 0x06:
        {
            uint16_t write_val = (rx_data[4] << 8) | rx_data[5];
            if (start_addr >= MODBUS_REG_NUM) return;
            
            Modbus_RegPool[start_addr] = write_val;
            Modbus_Execute_Action(start_addr); // 执行动作
            
            // 06 功能码原样返回
            memcpy(Modbus_TxBuf, rx_data, 6);
            tx_len = 6;
            break;
        }
        case 0x10: // 写多个寄存器 
        {
            uint16_t write_cnt = (rx_data[4] << 8) | rx_data[5];
            uint8_t byte_cnt = rx_data[6];
            if (start_addr + write_cnt > MODBUS_REG_NUM || byte_cnt != write_cnt * 2) return;
            
            for (uint16_t i = 0; i < write_cnt; i++) {
                Modbus_RegPool[start_addr + i] = (rx_data[7 + i*2] << 8) | rx_data[8 + i*2];
                // 每次更新完一个寄存器后，检查是否需要执行动作
                Modbus_Execute_Action(start_addr + i);
            }
            
            // 返回报文
            Modbus_TxBuf[0] = MODBUS_SLAVE_ADDRESS;
            Modbus_TxBuf[1] = 0x10;
            Modbus_TxBuf[2] = rx_data[2];
            Modbus_TxBuf[3] = rx_data[3];
            Modbus_TxBuf[4] = rx_data[4];
            Modbus_TxBuf[5] = rx_data[5];
            tx_len = 6;
            break;
        }
        default:
            return; // 不支持的功能码
    }

    // 添加 CRC 并发送
    if (tx_len > 0) {
        uint16_t crc_send = Modbus_CRC16(Modbus_TxBuf, tx_len);
        Modbus_TxBuf[tx_len++] = crc_send & 0xFF; // 低位在前
        Modbus_TxBuf[tx_len++] = (crc_send >> 8) & 0xFF; // 高位在后
        
        // 调用串口发送函数
        Modbus_Hardware_Transmit(Modbus_TxBuf, tx_len);
    }
}


// 实现底层的串口发送函数
void Modbus_Hardware_Transmit(uint8_t *tx_data, uint16_t tx_len)
{
  // 注意：uart_mixed 中参数是 char*，所以需要做个类型强转
    UART_Mixed_TxTrigger(&muart1, (char *)tx_data, tx_len);
}

void Modbus_Update_Feedback(void)
{
      
    Modbus_RegPool[REG_GOAL_INTERVAL] = (uint16_t)(limit_finder.safe_max_pos-limit_finder.safe_min_pos);
	  Modbus_RegPool[REG_GOAL_ACCEL]=(uint16_t)(location_tck.up_acc/Move_Pulse_NUM);
	  Modbus_RegPool[REG_GOAL_CURRENT]=(uint16_t) Current_Rated_Current;
	  Modbus_RegPool[REG_GOAL_SPEED]=(uint16_t)(location_tck.max_speed/Move_Pulse_NUM);
	  Modbus_RegPool[REG_ENABLEOFF_TIME]= auto_stop_time;
	  Modbus_RegPool[REG_STATUS_WORD] = motor_control.state;
	  Modbus_RegPool[REG_MOTOR_LOCATION_L]=(uint16_t)(((uint32_t) motor_control.real_location)&0xffff); 
    Modbus_RegPool[REG_MOTOR_LOCATION_H]=(uint16_t)(((uint32_t)motor_control.real_location) >> 16);

}