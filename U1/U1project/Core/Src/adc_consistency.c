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
            /* 注: U4 的 CH3 标度补偿已在 ADC_Consistency_Start() 填缓冲时完成,
             * 本函数拿到的三个芯片数据已统一标度, 直接比对即可。 */
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
     *   CH1=vo1, CH2=vo2, CH3=ams_adc; CH4 不在报文流, status=0 待治具完成
     * ★★[2026-11-15] 增加"新鲜度"判定 ★★
     *   原来无条件把 U4 三路标成 status=1。但 u4_report 只是 RAM 快照 —— U4 下电后
     *   不再有新帧, 旧值一直留在里面, 于是界面上 DUT 那几路"数值保持不变", 看起来
     *   像还在测。现改为: 只有最近 ADC_CONSIST_U4_FRESH_MS 内收到过 0x30 报文,
     *   才认为 U4 的读数有效; 否则清 0 并标 status=0 (比对时会跳过它,
     *   仍可用 U1+U7 两路做一致性判定)。
     * ★这也让"U4 掉线"在结果里能看出来, 而不是被旧数据掩盖。 */
    memset(adc_consist_buf[1], 0, 16);
    adc_consist_buf[1][0]  = 1;   /* 通道号先填好, 数据区留 0 */
    adc_consist_buf[1][4]  = 2;
    adc_consist_buf[1][8]  = 3;
    adc_consist_buf[1][12] = 4;
    if (U4_ReportIsRecent(ADC_CONSIST_U4_FRESH_MS)) {
        const uint16_t vals[3] = { u4_report.vo1, u4_report.vo2, u4_report.ams_adc };
        for (uint8_t i = 0; i < 3; i++) {
            uint16_t v = vals[i];
            /* [2026-11-13] U4 的 CH3(ams_adc) 标度补偿: 实测 U4 只有 U1/U7 同测点的
             * 0.849~0.860 倍 (4 块板实测, 疑为 U4 PA6 处有分压)。此处统一换算到
             * U1/U7 的标度 → ACK 里三芯片的值同标度、可直接互相对照, 判定也基于它。
             * v ≤ 4095, ×117 = 479115 不溢出 uint32; ÷100 后 ≤ 4791 不溢出 uint16。 */
            if (i == 2) {
                v = (uint16_t)((uint32_t)v
                               * ADC_CONSIST_U4_AMS_NUM / ADC_CONSIST_U4_AMS_DEN);
            }
            adc_consist_buf[1][i * 4 + 1] = (uint8_t)(v >> 8);
            adc_consist_buf[1][i * 4 + 2] = (uint8_t)(v);
            adc_consist_buf[1][i * 4 + 3] = 1;      /* 有效性: 有新鲜报文才置 1 */
        }
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
