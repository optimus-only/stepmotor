/******
	************************************************************************
	******
	** @project : XDrive_Step
	** @brief   : Stepper motor with multi-function interface and closed loop function. 
	** @brief   : 具有多功能接口和闭环功能的步进电机
	** @author  : unlir (知不知啊)
	** @contacts: QQ.1354077136
	******
	** @address : https://github.com/unlir/XDrive
	******
	************************************************************************
	******
	** {Stepper motor with multi-function interface and closed loop function.}
	** Copyright (c) {2020}  {unlir(知不知啊)}
	** 
	** This program is free software: you can redistribute it and/or modify
	** it under the terms of the GNU General Public License as published by
	** the Free Software Foundation, either version 3 of the License, or
	** (at your option) any later version.
	** 
	** This program is distributed in the hope that it will be useful,
	** but WITHOUT ANY WARRANTY; without even the implied warranty of
	** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	** GNU General Public License for more details.
	** 
	** You should have received a copy of the GNU General Public License
	** along with this program.  If not, see <http://www.gnu.org/licenses/>.
	******
	************************************************************************
******/

/*****
  ** @file     : uart_mixed.c/h
  ** @brief    : Rein串口通信库
  ** @versions : 1.1.1
  ** @time     : 2019/11/1
  ** @reviser  : unli (HeFei China)
  ** @explain  : null
*****/


// 升级日志
/** V:1.0.6
  * 时间 2019/3/5
  * 1.UART_Mixed_RxCallback函数->使用__HAL库获取DMA剩余传输次数
  * 2.简化Uart_Mixed_DeInit函数
  * 3.增加Uart6
  * 4.修复UART_Dir_Control的Bug
  * 5.删除UART_Mixed_RxCallback函数
  
*** V:1.1.1
  * 时间 2019/11/1
  * 1.并入Rein开发包
  * 1.修改失能混合串口为停止混合串口，并添加开始混合串口，初始化也修改为调用开始函数
***/

//使用介绍
//1. 修改配置(发送模式,收发缓冲区)
//3. 使能对应串口DMA功能和中断
//4. 使能对应串口全局中断并在it文件中添加对应串口的空闲中断处理(HAL库未提供空闲处理)
//5. 可配置添加DIR引脚控制语句(见UART_Dir_Control函数)

//Oneself
#include "uart_mixed.h"

//Application_User_Core
#include "usart.h"

