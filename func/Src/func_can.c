#include "func_can.h"

Motor_Measure_t motor_info[4]={0};

//**************************************************************************************************//

/**
 * @brief 将无符号整数（定点值）转换为浮点数（物理值）
 *
 * 原理：把 [0, 2^bits - 1] 的整数范围，线性映射到 [x_min, x_max] 的物理范围。
 *       常用于解析电机反馈帧：把 CAN 帧里拼出来的原始整数还原成真实角度/速度/扭矩。
 *
 * @param x_int  输入的原始整数（例如从 CAN 帧中拼出的 16 位位置值）
 * @param x_min  该物理量对应的最小值（例如 -P_MAX）
 * @param x_max  该物理量对应的最大值（例如 +P_MAX）
 * @param bits   该物理量在 CAN 帧中占用的位宽（例如位置用 16 位，速度用 12 位）
 * @return       转换后的浮点物理值
 */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// 把无符号整数按给定范围和位宽转换成浮点数 ///
    float span = x_max - x_min;          // 物理量的总跨度，例如 x_max=12.5, x_min=-12.5 则 span=25
    float offset = x_min;                // 物理量的起始偏移，用于把 [0,1] 映射回真实范围
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    //      ↑ 原始整数     ↑ 归一化系数 (1/(2^bits - 1))              ↑ 加上偏移还原到真实物理区间
}

/**
 * @brief 将浮点数（物理值）转换为无符号整数（定点值）
 *
 * 原理：与上面相反，把 [x_min, x_max] 的物理范围，线性压缩到 [0, 2^bits - 1] 的整数范围。
 *       常用于发送控制指令：把想要的目标位置/速度/扭矩，打包成 CAN 帧里的整数。
 *
 * @param x      输入的浮点物理值（例如期望位置 3.14 rad）
 * @param x_min  该物理量对应的最小值
 * @param x_max  该物理量对应的最大值
 * @param bits   该物理量在 CAN 帧中占用的位宽
 * @return       转换后的无符号整数（范围 0 ~ 2^bits - 1）
 */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// 把浮点数按给定范围和位宽转换成无符号整数 ///
    float span = x_max - x_min;          // 物理量的总跨度
    float offset = x_min;                // 物理量的起始偏移
    return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
    //            ↑ 先减去偏移，   ↑ 再乘以最大整数刻度，        ↑ 除以跨度归一化
    //              让 x 落在 [0, span] 区间
}

//**************************************************************************************************//

void func_can_init(void)
{
    CAN_FilterTypeDef f;

    f.FilterActivation     = ENABLE;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterBank           = 0;

    // 主机 ID（Master ID）= 0x200，精确匹配
    f.FilterIdHigh     = (0x200 << 5) & 0xFFFF;   // 0x4000
    f.FilterIdLow      = 0x0000;
    f.FilterMaskIdHigh = (0x7FF << 5) & 0xFFFF;   // 0xFFE0：11 位全匹配，只放行 0x200
    f.FilterMaskIdLow  = 0x0000;

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}


void func_can_transmit(uint32_t id,uint8_t datas[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t            pTxMailbox;

    tx_header.StdId = id; 
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, datas, &pTxMailbox);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             rx_data[8];

    if (hcan->Instance == CAN1) 
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

        uint8_t index = rx_data[0] & 0x0F - 1;
        
        if(index >= 0 && index <=3)
            motor_info[index].err = rx_data[0] >> 4;
            motor_info[index].position_rad = (uint16_t)rx_data[2] | ((uint16_t)rx_data[1] << 8);
            motor_info[index].speed_rads = ((uint16_t)rx_data[4] >> 4 )| ((uint16_t)rx_data[3] << 4);
            motor_info[index].moment_Nm = (uint16_t)rx_data[5] | (((uint16_t)rx_data[4] & 0x0F) << 8);
            motor_info[index].T_Mos_dc = rx_data[6];
            motor_info[index].T_Rotor_dc = rx_data[7];
    }
}

