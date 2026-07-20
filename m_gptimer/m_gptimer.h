/*
 * m_gptimer.h
 *
 * 基于 GPTimer 的 bit-bang 串口驱动，适用于 ESP32-P4。
 * 协议格式: 8N1（8 数据位，无校验位，1 停止位）。
 */

#ifndef _M_GPTIMER_H_
#define _M_GPTIMER_H_

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "hal/gpio_types.h"

/**
 * @brief 初始化 GPTimer 模拟串口模块
 *
 * 创建 GPTimer（resolution = baud_rate，1 tick = 1 个 bit 周期），
 * 配置 TX 引脚为输出（空闲高电平），RX 引脚为输入（内部上拉）。
 * GPIO 中断已注册但未使能，在每次 m_gptimer_receive() 调用时激活。
 *
 * @param baud_rate  波特率（如 9600、115200）
 * @param tx_pin     TX 输出的 GPIO 编号
 * @param rx_pin     RX 输入的 GPIO 编号
 * @return 成功返回 ESP_OK，失败返回错误码
 */
esp_err_t m_gptimer_init(uint32_t baud_rate, gpio_num_t tx_pin, gpio_num_t rx_pin);

/**
 * @brief 发送数据（阻塞）
 *
 * 依次发送 len 个字节，阻塞直到全部发完或超时。
 * 同一时刻只能有一个 send 或 receive 操作（内部 mutex 互斥）。
 *
 * @param data        待发送数据的指针
 * @param len         待发送字节数（为 0 时直接返回 ESP_OK）
 * @param timeout_ms  超时时间（毫秒），portMAX_DELAY 表示永久等待
 * @return 成功返回 ESP_OK，超时返回 ESP_ERR_TIMEOUT
 */
esp_err_t m_gptimer_send(const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief 接收数据（阻塞）
 *
 * 等待接收 len 个字节，阻塞直到收满或超时。
 * 同一时刻只能有一个 send 或 receive 操作（内部 mutex 互斥）。
 *
 * @param buf         接收缓冲区指针
 * @param len         期望接收的字节数（为 0 时直接返回 ESP_OK）
 * @param timeout_ms  超时时间（毫秒），portMAX_DELAY 表示永久等待
 * @return 成功返回 ESP_OK，超时返回 ESP_ERR_TIMEOUT，
 *         帧错误（stop bit 不为高）返回 ESP_ERR_INVALID_RESPONSE
 */
esp_err_t m_gptimer_receive(uint8_t *buf, size_t len, uint32_t timeout_ms);

/**
 * @brief 反初始化模块，释放所有资源
 *
 * 停止 timer，注销 GPIO ISR，释放 timer 和 mutex。
 * @return 成功返回 ESP_OK
 */
esp_err_t m_gptimer_deinit(void);

/**
 * @brief GPTimer 模拟串口自发自收测试
 *
 * 测试方法: 用杜邦线短接 TX 和 RX 引脚
 */
void m_gptimer_test(void);

#endif /* _M_GPTIMER_H_ */
