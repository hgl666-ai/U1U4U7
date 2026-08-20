#include "adc_consistency.h"
#include "app.h"
#include "app_internal.h"
#include "protocol_u4.h"
#include "protocol_u7.h"
#include "bsp_adc.h"
#include <string.h>

/*===== 状态 =====
 * 0=idle, 2=等U7, 3=比对完成 (U4 数据直接取报文流, 无独立等待阶段)
 */
static uint8_t  adc_consist_phase;
static uint8_t  adc_consist_buf[3][16];       /* [0]=U1 [1]=U4 [2]=U7, 各 4ch×4B */
static uint8_t  adc_consist_threshold;        /* PC 下发的 LSB 阈值 */
static uint32_t adc_consist_deadline;         /* 当前阶段超时时刻 */

/*===== ADC 一致性比对 =====
 * 输入: buf[3][16] — U1/U4/U7 各 4 通道 × 4B (id/raw_hi/raw_lo/status)
 * 输出: result[16] — 4 通道 × 4B (id/MaxDiff_hi/MaxDiff_lo/pass)
 *       每通道: MaxDiff = max(raw) - min(raw), 仅取 status=1 的芯片
 *       pass = 1 当 MaxDiff ≤ threshold 且 ≥2 芯片有效, 否则 0
 */
static void ADC_ConsistencyCompare(uint8_t buf[3][16], uint8_t threshold,
                                   uint8_t *result)
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        uint8_t  ch_id   = (uint8_t)(ch + 1);
        uint16_t min_raw = 0xFFFF;
        uint16_t max_raw = 0;
        uint8_t  valid_count = 0;

        /* 扫描 3 个芯片的同一通道 */
        for (uint8_t chip = 0; chip < 3; chip++) {
            uint8_t status = buf[chip][ch * 4 + 3];
            if (status == 0) continue;
            uint16_t raw = ((uint16_t)buf[chip][ch * 4 + 1] << 8)
                         |  (uint16_t)buf[chip][ch * 4 + 2];
            if (raw < min_raw) min_raw = raw;
            if (raw > max_raw) max_raw = raw;
            valid_count++;
        }

        uint16_t diff = (valid_count >= 2) ? (max_raw - min_raw) : 0xFFFF;
        uint8_t  pass = (valid_count >= 2 && diff <= threshold) ? 1 : 0;

        result[ch * 4 + 0] = ch_id;
        result[ch * 4 + 1] = (uint8_t)(diff >> 8);
        result[ch * 4 + 2] = (uint8_t)(diff & 0xFF);
        result[ch * 4 + 3] = pass;
    }
}

/*===== 公开 API =====*/

void ADC_Consistency_Init(void)
{
    adc_consist_phase = 0;
}

uint8_t ADC_Consistency_Start(uint8_t threshold)
{
    if (adc_consist_phase != 0) return 1;  /* BUSY */

    adc_consist_threshold = threshold;

    /* Phase 0: 读 U1 本地 ADC */
    {
        static const uint32_t ch[4] = {ADC_CH_ADC0, ADC_CH_ADC1, ADC_CH_ADC2, ADC_CH_ADC3};
        static const uint8_t  id[4] = {1, 2, 3, 4};
        for (uint8_t i = 0; i < 4; i++) {
            ADC_Value_t v = BSP_ADC_ReadChannel(ch[i], 4);
            adc_consist_buf[0][i * 4 + 0] = id[i];
            adc_consist_buf[0][i * 4 + 1] = (uint8_t)(v.raw >> 8);
            adc_consist_buf[0][i * 4 + 2] = (uint8_t)(v.raw & 0xFF);
            adc_consist_buf[0][i * 4 + 3] = v.valid ? 1 : 0;
        }
    }
    /* [2026-08-14] U4 数据改从报文流取 (U4 固件未实现 0x34):
     *   CH1=vo1, CH2=vo2, CH3=ams_adc; CH4 不在报文流, status=0 待治具完成 */
    memset(adc_consist_buf[1], 0, 16);
    {
        const uint16_t vals[3] = { u4_report.vo1, u4_report.vo2, u4_report.ams_adc };
        for (uint8_t i = 0; i < 3; i++) {
            adc_consist_buf[1][i * 4 + 0] = (uint8_t)(i + 1);
            adc_consist_buf[1][i * 4 + 1] = (uint8_t)(vals[i] >> 8);
            adc_consist_buf[1][i * 4 + 2] = (uint8_t)(vals[i]);
            adc_consist_buf[1][i * 4 + 3] = 1;
        }
        adc_consist_buf[1][3 * 4 + 0] = 4;   /* CH4 不在报文流 */
    }
    memset(adc_consist_buf[2], 0, 16);

    /* U4 数据已取好, 直接启 U7 查询 (phase 2) */
    U7_GetADC(adc_consist_buf[2]);
    adc_consist_phase    = 2;
    adc_consist_deadline = HAL_GetTick() + ADC_CONSISTENCY_TIMEOUT_MS;
    return 0;
}

uint8_t ADC_Consistency_IsBusy(void)
{
    return adc_consist_phase != 0;
}

void ADC_Consistency_Run(void)
{
    if (adc_consist_phase == 0) return;

    /* 超时保护: 等 U7 超时 → 直接比对 (U7 buf 保持全 0) */
    if (HAL_GetTick() > adc_consist_deadline) {
        adc_consist_phase = 3;
    }

    /*── Phase 2: 等 U7 (U4 数据已从报文流取好, 无 Phase 1) ──*/
    if (adc_consist_phase == 2) {
        int ret = U7_GetADC(adc_consist_buf[2]);
        if (ret == U7_PROTO_PENDING) return;
        adc_consist_phase = 3;
        /* 落到 phase 3 */
    }

    /*── Phase 3: 比对 + 组包回传 ──*/
    if (adc_consist_phase == 3) {
        static uint8_t result[ADC_CONSISTENCY_RESULT_LEN];  /* static 防栈优化问题 */
        /* 前 48B: 3 芯片 ADC 数据 */
        memcpy(&result[0],  adc_consist_buf[0], 16);
        memcpy(&result[16], adc_consist_buf[1], 16);
        memcpy(&result[32], adc_consist_buf[2], 16);
        /* 后 16B: 判定区 */
        ADC_ConsistencyCompare(adc_consist_buf, adc_consist_threshold,
                               &result[48]);

        APP_SendAck(MGR_CMD_QUERY_ADC, result, ADC_CONSISTENCY_RESULT_LEN);
        adc_consist_phase = 0;
    }
}
