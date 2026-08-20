#include "adc_calib.h"
#include "app.h"
#include "app_internal.h"
#include "protocol_u7.h"
#include "protocol_u4.h"
#include "bsp_adc.h"
#include <string.h>

/*===== 状态机 =====
 * Phase: 0=idle, 1=MOVE, 4=WAIT, 2=PROC, 3=DONE
 */
static uint8_t  adc_calib_phase;
static uint8_t  adc_calib_dir;                /* 0=CW(U7_MOTOR_CW), 1=CCW */
static uint8_t  adc_calib_cycle;              /* 当前已完成方向数 */
static uint8_t  adc_calib_reads;              /* 当前方向内已完成读数 */
static uint16_t adc_calib_max1, adc_calib_min1; /* CH1 极值 */
static uint16_t adc_calib_max2, adc_calib_min2; /* CH2 极值 */
static uint32_t adc_calib_deadline;           /* 电机到位等待截止时刻 */
static uint8_t  adc_calib_adc_buf[16];        /* U4 ADC 读数缓冲 */

/*===== 公开 API =====*/

void ADC_Calib_Init(void)
{
    adc_calib_phase = 0;
}

uint8_t ADC_Calib_Start(void)
{
    if (adc_calib_phase != 0) return 1;  /* BUSY */

    adc_calib_phase = 1;
    adc_calib_dir   = 0;              /* 先从 CW 开始 */
    adc_calib_cycle = 0;
    adc_calib_reads = 0;
    adc_calib_max1  = 0;
    adc_calib_min1  = 0xFFFF;
    adc_calib_max2  = 0;
    adc_calib_min2  = 0xFFFF;
    return 0;  /* 不立即 ACK, 由 ADC_Calib_Run 异步完成 */
}

uint8_t ADC_Calib_IsBusy(void)
{
    return adc_calib_phase != 0;
}

void ADC_Calib_Run(void)
{
    if (adc_calib_phase == 0) return;

    /*── Phase 1: 电机移动 ──*/
    if (adc_calib_phase == 1) {
        int ret = U7_MotorStep(ADC_CALIB_MOTOR_ID,
                               adc_calib_dir ? U7_MOTOR_CCW : U7_MOTOR_CW,
                               ADC_CALIB_STEPS_PER_READ);
        if (ret == U7_PROTO_PENDING) return;
        /* U7 已收到指令, 等电机物理到位 */
        adc_calib_deadline = HAL_GetTick() + ADC_CALIB_MOTOR_DELAY_MS;
        adc_calib_phase = 4;
        return;
    }

    /*── Phase 4: 等待电机物理到位 ──*/
    if (adc_calib_phase == 4) {
        if (HAL_GetTick() < adc_calib_deadline) return;
        /* [2026-08-14] 改从 U4 报文流取 CH1/CH2 (U4 固件未实现 0x34):
         *   CH1=vo1, CH2=vo2; CH3/CH4 本任务不用, 置 0 */
        memset(adc_calib_adc_buf, 0, 16);
        adc_calib_adc_buf[0 * 4 + 0] = 1;                          /* CH1 = vo1 */
        adc_calib_adc_buf[0 * 4 + 1] = (uint8_t)(u4_report.vo1 >> 8);
        adc_calib_adc_buf[0 * 4 + 2] = (uint8_t)(u4_report.vo1);
        adc_calib_adc_buf[0 * 4 + 3] = 1;
        adc_calib_adc_buf[1 * 4 + 0] = 2;                          /* CH2 = vo2 */
        adc_calib_adc_buf[1 * 4 + 1] = (uint8_t)(u4_report.vo2 >> 8);
        adc_calib_adc_buf[1 * 4 + 2] = (uint8_t)(u4_report.vo2);
        adc_calib_adc_buf[1 * 4 + 3] = 1;
        adc_calib_phase = 2;
        return;
    }

    /*── Phase 2: 处理 ADC 数据 + 决策下一步 ──*/
    if (adc_calib_phase == 2) {
        /* 取 CH1/CH2 raw 值更新极值 */
        {
            uint16_t raw1 = ((uint16_t)adc_calib_adc_buf[1] << 8)
                          |  (uint16_t)adc_calib_adc_buf[2];
            uint16_t raw2 = ((uint16_t)adc_calib_adc_buf[5] << 8)
                          |  (uint16_t)adc_calib_adc_buf[6];
            uint8_t  st1  = adc_calib_adc_buf[3];
            uint8_t  st2  = adc_calib_adc_buf[7];

            if (st1) {
                if (raw1 > adc_calib_max1) adc_calib_max1 = raw1;
                if (raw1 < adc_calib_min1) adc_calib_min1 = raw1;
            }
            if (st2) {
                if (raw2 > adc_calib_max2) adc_calib_max2 = raw2;
                if (raw2 < adc_calib_min2) adc_calib_min2 = raw2;
            }
            /* 不发送中间事件帧, 校准完成后一次性汇总返回 */
        }

        adc_calib_reads++;
        if (adc_calib_reads >= ADC_CALIB_READS_PER_DIR) {
            adc_calib_reads = 0;
            adc_calib_dir   = (adc_calib_dir == 0) ? 1 : 0;  /* CW ↔ CCW */
            adc_calib_cycle++;
        }

        if (adc_calib_cycle >= ADC_CALIB_CYCLES * 2U) {
            adc_calib_phase = 3;  /* 全部完成 */
        } else {
            adc_calib_phase = 1;  /* 继续下一段移动 */
        }
        return;
    }

    /*── Phase 3: 发送最终结果 (8B 极值 ACK) ──*/
    if (adc_calib_phase == 3) {
        uint8_t result[ADC_CALIB_RESULT_LEN];
        result[0] = (uint8_t)(adc_calib_max1 >> 8);
        result[1] = (uint8_t)(adc_calib_max1);
        result[2] = (uint8_t)(adc_calib_min1 >> 8);
        result[3] = (uint8_t)(adc_calib_min1);
        result[4] = (uint8_t)(adc_calib_max2 >> 8);
        result[5] = (uint8_t)(adc_calib_max2);
        result[6] = (uint8_t)(adc_calib_min2 >> 8);
        result[7] = (uint8_t)(adc_calib_min2);

        APP_SendAck(MGR_CMD_ADC_CALIB, result, ADC_CALIB_RESULT_LEN);
        adc_calib_phase = 0;
    }
}
