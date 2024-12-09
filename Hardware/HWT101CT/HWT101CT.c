/*
 * HWT101CT.c
 *
 *  Created on: Nov 27, 2024
 *      Author: yinnanzhao
 */
#include "HWT101CT.h"
float global_angle=0;
float angular_velocity_y;
float angular_velocity_z;
uint16_t new_data_received;

void ParseAndPrintData(uint8_t *data, uint16_t length) {
    if (length == 11) {
        uint8_t checksum = CalculateChecksum(data, length - 1, data[1]);
        if (checksum != data[length - 1]) {   //错误
            return;
        }

        if (data[0] == 0x55 && data[1] == 0x53) {
            // 数据部分从索引 6 开始，低字节在前，高字节在后
            uint8_t yaw_l = data[6];
            uint8_t yaw_h = data[7];
            int16_t yaw = (int16_t)((yaw_h << 8) | yaw_l);

            // 将解析后的整数值转换为角度
            float angle = ((float)yaw / 32768.0) * 180.0;
            global_angle = angle;
            new_data_received = 1;
        } else if (data[0] == 0x55 && data[1] == 0x52) {
            // 解析角速度数据
            uint8_t wy_l = data[4];
            uint8_t wy_h = data[5];
            int16_t wy = (int16_t)((wy_h << 8) | wy_l);
            angular_velocity_y = ((float)wy / 32768.0) * 2000.0;

            uint8_t wz_l = data[6];
            uint8_t wz_h = data[7];
            int16_t wz = (int16_t)((wz_h << 8) | wz_l);
            angular_velocity_z = ((float)wz / 32768.0) * 2000.0;

            new_data_received = 1;
        }
    }
}

uint8_t CalculateChecksum(uint8_t *data, uint16_t length, uint8_t type)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}
