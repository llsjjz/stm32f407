#include "func_uart.h"

//发送函数
void func_uart_transmit(uint8_t* txBuffer,uint16_t len)
{
    HAL_UART_Transmit(UART_2,txBuffer,len,100);
}

//接收部分
/* 接收相关全局变量 */
static uint8_t  rx_byte;                 // 中断接收单字节缓存
static char     frame_buf[64];           // 帧缓冲区（仅保存数据部分，不含首尾A）
static uint8_t  frame_idx = 0;           // 当前写入位置
static uint8_t  recv_state = 0;          // 0=等待帧头A, 1=正在接收帧内数据
static uint8_t  new_frame = 0;           // 新数据标志

/* 解析后的数据存储 */
static float    rx_float[4];
static uint8_t  rx_uint8;
static uint8_t  rx_mod;


/**
 * @brief 启动UART6文本帧接收（中断方式）
 * @param huart UART句柄，例如 &huart6
 */
void func_uart_init(void)
{
    recv_state = 0;
    frame_idx = 0;
    HAL_UART_Receive_IT(UART_2, &rx_byte, 1);   // 接收第1个字节
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ( huart == UART_2 )                     // 确认是UART6
    {
        if (recv_state == 0)                           // 状态0：等待帧头'A'
        {
            if (rx_byte == 'A' || rx_byte == 'B')
            {
				if(rx_byte == 'B')rx_mod=1;
				else rx_mod=0;
                recv_state = 1;                        // 进入接收状态
                frame_idx = 0;                         // 清空缓冲区
            }
        }
        else                                           // 状态1：正在接收帧内数据
        {
            if (rx_byte == 'A')                        // 遇到帧尾'A'，帧结束
            {
                recv_state = 0;                        // 回到等待帧头状态
                frame_buf[frame_idx] = '\0';           // 字符串终止符

                /* 解析数据部分 */
                char *p = frame_buf;
                if (*p == ',') p++;                    // 跳过开头的逗号
                // 去掉尾部可能存在的逗号（数字后的逗号）
                size_t len = strlen(p);
                if (len > 0 && p[len - 1] == ',')
                    p[len - 1] = '\0';

                // 解析4个float和1个uint8_t
                if (sscanf(p, "%f,%f,%f,%f,%hhu",
                           &rx_float[0], &rx_float[1],
                           &rx_float[2], &rx_float[3],
                           &rx_uint8) == 5)
                {
                    new_frame = 1;                     // 标记有效帧
                }
                // 解析失败则直接丢弃，不会置位 new_frame
            }
            else                                      // 普通数据字节
            {
                if (frame_idx < sizeof(frame_buf) - 1) // 防止溢出
                {
                    frame_buf[frame_idx++] = rx_byte;
                }
                else
                {
                    recv_state = 0;                   // 溢出则复位，等待新帧
                }
            }
        }

        /* 重新使能下一字节接收 */
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}

/**
 * @brief 获取文本帧解析结果
 * @param f1~f4 输出：4个float数据
 * @param val   输出：1个uint8_t数据
 * @return 1-有新数据；0-无新数据
 */
uint8_t func_uart_getValue(float *f1, float *f2, float *f3, float *f4, uint8_t *val , uint8_t *mod)
{
    if (new_frame)
    {
        *f1 = rx_float[0];
        *f2 = rx_float[1];
        *f3 = rx_float[2];
        *f4 = rx_float[3];
        *val = rx_uint8;
	    *mod = rx_mod;
        new_frame = 0;                // 清除标志
        return 1;
    }
    return 0;
}
