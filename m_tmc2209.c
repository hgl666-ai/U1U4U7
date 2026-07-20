#include "m_tmc2209.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include <string.h>

#include "m_gptimer_uart.h"
#include "m_config.h"

static const char* TAG = "TMC2209";

static TaskHandle_t s_task = NULL;

m_tmc2209_conf_t m_tmc2209_conf = {0};

static uint8_t calc_crc(uint8_t *datagram, uint8_t length)
{
    uint8_t crc = 0;
    uint8_t current_byte;
    for (int i = 0; i < length; i++) {
        current_byte = datagram[i];
        for (int j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (current_byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            current_byte = current_byte >> 1;
        }
    }
    return crc;
}

static void build_write_datagram(uint8_t slave_addr, uint8_t reg_addr, uint32_t data, uint8_t* datagram){
    datagram[0] = TMC2209_SYNC_BYTE;
    datagram[1] = slave_addr;
    datagram[2] = reg_addr | 0x80;
    datagram[3] = (data>>24) & 0xFF;
    datagram[4] = (data>>16) & 0xFF;
    datagram[5] = (data>>8) & 0xFF;
    datagram[6] = data & 0xFF;
    datagram[7] = calc_crc(datagram,7);
}

static void build_read_datagram(uint8_t slave_addr, uint8_t reg_addr, uint8_t* datagram){
    datagram[0] = TMC2209_SYNC_BYTE;
    datagram[1] = slave_addr;
    datagram[2] = reg_addr;
    datagram[3] = calc_crc(datagram,3);
}

static esp_err_t write_reg(uint8_t slave_addr, uint8_t reg_addr, uint32_t data){
    uint8_t datagram[8];
    build_write_datagram(slave_addr, reg_addr, data, datagram);
    esp_err_t ret = m_gptimer_send(datagram, 8, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    return ret;
}

static esp_err_t read_reg(uint8_t slave_addr, uint8_t reg_addr, uint32_t* data){
    uint8_t req[4];
    s_task = xTaskGetCurrentTaskHandle();

    build_read_datagram(slave_addr, reg_addr, req);

    esp_err_t ret = m_gptimer_send(req, 4, 0);
    if (ret != ESP_OK)  {
        ESP_LOGE(TAG, "[%02X] read 0x%02X cmd fail, err code: %d", slave_addr, reg_addr, ret);
        return ret;
    }
    if (xTaskNotifyWait(0, 0, data, pdMS_TO_TICKS(30)) == pdTRUE) {  // 等待通知，寄存器值会自动传入data
        vTaskDelay(pdMS_TO_TICKS(5));
        return ESP_OK;
    }
    ESP_LOGE(TAG, "[%02X] read 0x%02X timeout 30ms", slave_addr, reg_addr);
    return ESP_ERR_TIMEOUT;
}

void m_tmc2209_rx_callback(uint8_t* rxdata, uint32_t rxlen) {
    if (rxlen > 1) {
        for (int i = 1; i < rxlen; i++) {
            // TMC2209 response:
            // 8bits: -1   0   1   2   3   4   5   6
            //        05   FF  00  **  **  **  **  crc
            //                     <  reg data  >  
            if (i+6 >= rxlen) {
                break;
            }
            if (rxdata[i] == 0xFF && rxdata[i-1] == TMC2209_SYNC_BYTE) {
                uint8_t crc = calc_crc(&rxdata[i-1], 7);
                if (crc != rxdata[i+6]) {
                    ESP_LOGE(TAG, "CRC error: expected 0x%02X, got 0x%02X", crc, rxdata[i+6]);
                    return;
                }
                ESP_LOGI(TAG, "CRC OK (0x%02X)", crc);

                uint32_t reply_data = (uint32_t)rxdata[i+2]<<24 |
                                      (uint32_t)rxdata[i+3]<<16 |
                                      (uint32_t)rxdata[i+4]<<8  |
                                      (uint32_t)rxdata[i+5];
                ESP_LOGI(TAG, "Received data: 0x%08X", reply_data);
                xTaskNotify(s_task, reply_data, eSetValueWithOverwrite);  // 通知读取任务并返回寄存器值
            }
        }
    }
}

void m_tmc2209_init(void) {
    s_task = xTaskGetCurrentTaskHandle();

    // 初始化串口
    g_flag.gptimer_uart_debug = true;  // 是否启用串口调试
    m_gptimer_uart_init();

    m_gptimer_rx_callback_register(m_tmc2209_rx_callback);
    // vTaskDelay(pdMS_TO_TICKS(100));

#if 0
    #define TMC2209_DEV_QTY  4
    uint8_t dev_addrs[TMC2209_DEV_QTY] = {
        TMC2209_SLAVE_ADDR0, 
        TMC2209_SLAVE_ADDR1, 
        TMC2209_SLAVE_ADDR2, 
        TMC2209_SLAVE_ADDR3
    };
    GCONF_reg_t _gconf;
    for (int i=0; i<TMC2209_DEV_QTY; i++) {
        uint8_t slave = dev_addrs[i];
        // ESP_LOGI(TAG, "Try read GCONF of dev-%02X", slave);
        if (ESP_OK == read_reg(slave, TMC2209_REG_GCONF, &_gconf.raw)) {
            ESP_LOGI(TAG, "[%02X] GCONF raw: 0x%08X", slave, _gconf.raw);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    GCONF_reg_t gconf = {0};
    gconf.bits.I_scale_analog   = 0;  // 忽略 VREF 引脚，走内部 5VOUT 基准 + UART 数字控制电流
    gconf.bits.internal_Rsense  = 0;  // 外部 R_sense = 0.1Ω，BRA/BRB 各一颗
    gconf.bits.en_SpreadCycle   = 0;
    gconf.bits.shaft            = 0;
    gconf.bits.index_otpw       = 1;
    gconf.bits.index_step       = 0;
    gconf.bits.pdn_disable      = 1;  // UART控制时须设置为1
    gconf.bits.mstep_reg_select = 1;  // 0: 引脚 MS1, MS2 决定微步数; 1: MSTEP 寄存器决定微步数
    gconf.bits.multistep_filt   = 1;
    gconf.bits.test_mode        = 0;
    if (ESP_OK != write_reg(TMC2209_SLAVE_ADDR3, TMC2209_REG_GCONF, gconf.raw)) {
        ESP_LOGE(TAG, "Failed to write GCONF register");
    }
    if (ESP_OK == read_reg(TMC2209_SLAVE_ADDR3, TMC2209_REG_GCONF, &_gconf.raw)) {
        ESP_LOGI(TAG, "[%02X] GCONF back: 0x%08X", TMC2209_SLAVE_ADDR3, _gconf.raw);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
#else
    GCONF_reg_t _gconf;
    if (ESP_OK == read_reg(TMC2209_ADDR_L, TMC2209_REG_GCONF, &_gconf.raw)) {
        ESP_LOGI(TAG, "[%02X] Motor L - GCONF raw: 0x%08X", TMC2209_ADDR_L, _gconf.raw);
        m_tmc2209_conf.motor_l_connect = true;
    } else {
        ESP_LOGE(TAG, "[%02X] Motor L - GCONF read fail", TMC2209_ADDR_L);
        m_tmc2209_conf.motor_l_connect = false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    if (ESP_OK == read_reg(TMC2209_ADDR_S, TMC2209_REG_GCONF, &_gconf.raw)) {
        ESP_LOGI(TAG, "[%02X] Motor S - GCONF raw: 0x%08X", TMC2209_ADDR_S, _gconf.raw);
        m_tmc2209_conf.motor_s_connect = true;
    } else {
        ESP_LOGE(TAG, "[%02X] Motor S - GCONF read fail", TMC2209_ADDR_S);
        m_tmc2209_conf.motor_s_connect = false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
    if (ESP_OK == read_reg(TMC2209_ADDR_B, TMC2209_REG_GCONF, &_gconf.raw)) {
        ESP_LOGI(TAG, "[%02X] Motor B - GCONF raw: 0x%08X", TMC2209_ADDR_B, _gconf.raw);
        m_tmc2209_conf.motor_b_connect = true;
    } else {
        ESP_LOGE(TAG, "[%02X] Motor B - GCONF read fail", TMC2209_ADDR_B);
        m_tmc2209_conf.motor_b_connect = false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
#endif

/* ===================== GENERAL CONFIGURATION ===================== */

#if 1
    GCONF_reg_t gconf = {0};
    gconf.bits.I_scale_analog   = 0;  // 忽略 VREF，走内部 5VOUT 基准
    gconf.bits.internal_Rsense  = 0;  // 外部 0.1Ω 采样电阻
    gconf.bits.en_SpreadCycle   = 0;  // 0=StealthChop，1=SpreadCycle
    gconf.bits.shaft            = 0;  // 0=正转，1=反转
    gconf.bits.index_otpw       = 1;  // INDEX 引脚输出过温预警
    gconf.bits.index_step       = 0;  // INDEX = step 脉冲（未使用）
    gconf.bits.pdn_disable      = 1;  // UART 模式必须置 1
    gconf.bits.mstep_reg_select = 1;  // 微步由 MSTEP 寄存器控制
    gconf.bits.multistep_filt   = 1;  // 多步滤波（推荐）
    gconf.bits.test_mode        = 0;  // 严禁置 1
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_GCONF, gconf.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_GCONF, gconf.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_GCONF, gconf.raw); }
#endif

#if 0
    /* UART 发送延时（通常不需要改） */
    SLAVECONF_reg_t slaveconf = {0};
    slaveconf.bits.senddelay = 0; // 0 = 8 bit times
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_SLAVECONF, slaveconf.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_SLAVECONF, slaveconf.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_SLAVECONF, slaveconf.raw); }
#endif

/* ===================== CURRENT & POWER ===================== */

#if 1
    /*
     * 外部 Rsense = 0.1Ω
     * vsense = 1 → V_fs = 0.180V（推荐，功耗更低）
     *
     * 经验值（RMS）：
     * IRUN = 24  → ~1.2A
     * IHOLD = 12 → ~0.6A
     */
    IHOLD_IRUN_reg_t ihold_irun = {0};
    ihold_irun.bits.irun       = 24; // 运行电流
    ihold_irun.bits.ihold      = 12; // 保持电流
    ihold_irun.bits.iholddelay = 3;  // 静止后延迟降低电流
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_IHOLD_IRUN, ihold_irun.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_IHOLD_IRUN, ihold_irun.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_IHOLD_IRUN, ihold_irun.raw); }
#endif

#if 0
    /* 静止后多久开始降低电流 */
    TPOWERDOWN_reg_t tpowerdown = {0};
    tpowerdown.bits.tpowerdown = 20; // ~0.5s @ 12MHz
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_TPOWERDOWN, tpowerdown.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_TPOWERDOWN, tpowerdown.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_TPOWERDOWN, tpowerdown.raw); }
#endif

/* ===================== MICROSTEPPING ===================== */

#if 1
    /*
     * mres:
     * 0 = 256
     * 1 = 128
     * 2 =  64
     * 3 =  32
     * 4 =  16
     * 5 =   8
     * 6 =   4
     * 7 =   2
     * 8 =   1 (full step)
     */
    CHOPCONF_reg_t chopconf = {0};
    chopconf.bits.toff      = 3;   // 必须 > 0，否则驱动器关闭
    chopconf.bits.hstrt     = 4;
    chopconf.bits.hend      = 1;
    chopconf.bits.tbl       = 2;   // 24 / 36 / 44 / 54 clk
    chopconf.bits.vsense    = 1;   // 1 = 0.180V（推荐）
    chopconf.bits.mres      = 4;   // 16 微步
    chopconf.bits.intpol    = 1;   // 插值到 256 微步
    chopconf.bits.dedge     = 0;
    chopconf.bits.diss2g    = 0;
    chopconf.bits.diss2vs   = 0;
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_CHOPCONF, chopconf.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_CHOPCONF, chopconf.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_CHOPCONF, chopconf.raw); }
#endif

/* ===================== STEALTHCHOP (PWM) ===================== */

#if 0
    /*
     * StealthChop 参数（自动整定推荐）
     * 典型值来自 Trinamic 官方例程
     */
    PWMCONF_reg_t pwmconf = {0};
    pwmconf.bits.pwm_ofs        = 36;
    pwmconf.bits.pwm_grad       = 14;
    pwmconf.bits.pwm_freq       = 1;     // 1 = f_PWM = 2/683 f_CLK
    pwmconf.bits.pwm_autoscale  = 1;     // 自动缩放（必须）
    pwmconf.bits.pwm_autograd   = 1;     // 自动梯度
    pwmconf.bits.freewheel      = 1;     // 1 = 自由滑行
    pwmconf.bits.pwm_reg        = 8;
    pwmconf.bits.pwm_lim        = 12;
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_PWMCONF, pwmconf.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_PWMCONF, pwmconf.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_PWMCONF, pwmconf.raw); }
#endif

#if 0
    /*
     * StealthChop → SpreadCycle 切换阈值
     * 单位：t_step
     * 数值越小 = 越早切换到 SpreadCycle
     * 0 = 禁用切换（一直 StealthChop）
     */
    TPWMTHRS_reg_t tpwmthrs = {0};
    tpwmthrs.bits.tpwmthrs = 300; // 典型值，可按实际速度调整
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_TPWMTHRS, tpwmthrs.raw); }
    if (m_tmc2209_conf.motor_s_connect) { write_reg(TMC2209_ADDR_S, TMC2209_REG_TPWMTHRS, tpwmthrs.raw); }
    if (m_tmc2209_conf.motor_b_connect) { write_reg(TMC2209_ADDR_B, TMC2209_REG_TPWMTHRS, tpwmthrs.raw); }
#endif

/* ===================== STALLGUARD / COOLSTEP（可选） ===================== */

#if 0  // 不使用 StallGuard 时保持关闭
    TCOOLTHRS_reg_t tcoolthrs = {0};
    tcoolthrs.bits.tcoolthrs = 400;
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_TCOOLTHRS, tcoolthrs.raw); }

    SGTHRS_reg_t sgthrs = {0};
    sgthrs.bits.sgthrs = 80; // 需实测调整
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_SGTHRS, sgthrs.raw); }
#endif

/* ===================== VELOCITY MODE（可选） ===================== */

#if 0  // 仅用于无 STEP/DIR 的恒速模式
    VACTUAL_reg_t vactual = {0};
    vactual.bits.vactual = 0; // 正数正转，负数反转
    if (m_tmc2209_conf.motor_l_connect) { write_reg(TMC2209_ADDR_L, TMC2209_REG_VACTUAL, vactual.raw); }
#endif
    
    // 永久关闭串口
    vTaskDelay(pdMS_TO_TICKS(50));
    m_gptimer_rx_exit();
}
