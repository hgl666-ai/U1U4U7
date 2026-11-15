#include "sn_report.h"
#include "app.h"                /* MGR_CMD_SN_UPLOAD / APP_IsIdle / APP_SnReadUid */
#include "bsp_adc.h"            /* [2026-11-15] BSP_ADC_ReadRaw + ADC_CH_ADC3 */
#include "isp_programmer.h"     /* ISP_ReadMemory */
#include "protocol_frame.h"     /* Frame_Build */
#include "usbd_cdc_if.h"        /* CDC_Transmit_FS */
#include <string.h>

/* 上次上传的 UID, 用于去重 (新板才上传) */
static uint8_t last_uid[SN_UID_LEN];

/*===== 内部 =====*/

/**
  * @brief  组帧并发送 SN 上传帧 (U1→PC, 主动, SEQ=0)
  * @param  uid: 96-bit UID (12 字节)
  * @note   A5 5B 00 24 00 00 00 10 [版本][UID12][校验][预留2] CRC16
  * ★[2026-11-15] 去掉 static, 供"插入自动检测"分支复用(格式完全一致)。
  */
void SN_SendUidFrame(const uint8_t *uid)
{
    uint8_t data[SN_FRAME_LEN];
    uint8_t frame[FRAME_BUF_SIZE];
    uint8_t  xor_c = 0;

    data[0] = 0x01;  /* SN 算法版本 */
    memcpy(&data[1], uid, SN_UID_LEN);
    for (uint8_t i = 0; i < SN_UID_LEN; i++) xor_c ^= uid[i];
    data[13] = xor_c;
    data[14] = 0x00;
    data[15] = 0x00;

    uint16_t total = Frame_Build(frame, MGR_CMD_SN_UPLOAD, 0, data, SN_FRAME_LEN);

    uint32_t tick = HAL_GetTick();
    while (CDC_Transmit_FS(frame, total) == USBD_BUSY) {
        if (HAL_GetTick() - tick > 50) break;
    }
}

/*===== 公开 API =====*/

void SN_Report_Init(void)
{
    memset(last_uid, 0xFF, sizeof(last_uid));  /* 首次必不同, 触发上传 */
}

/**
  * @brief  PROGRAM 握手成功后调用: 读 UID → 去重 → 主动上传 SN
  * @note   读 UID 失败时静默跳过, 不中断烧录流程
  */
void SN_Report_OnProgram(void)
{
    uint8_t uid[SN_UID_LEN];

    if (ISP_ReadMemory(SN_UID_BASE, uid, SN_UID_LEN) != ISP_OK) {
        return;  /* 读 UID 失败不中断烧录 */
    }

    /* 同一块板重复烧录不重复上报 */
    if (memcmp(uid, last_uid, SN_UID_LEN) == 0) return;

    memcpy(last_uid, uid, SN_UID_LEN);
    SN_SendUidFrame(uid);
}

/*===== [2026-11-15] 插入自动检测 + 自动上报 SN =====
 * 背景: 上位机要求"接上一块新的 U4 就主动上传 SN"。不能靠 U4 自己报(空板没有固件),
 *       所以用 U1 自己的 ADC 通道 CH3(PA1) 判断板子在不在:
 *         插着 U4 ≈ 2064, 没接 ≈ 个位数 → 阈值 500/200 滞回, 去抖 4 次。
 * 关键约束(用户要求): **绝不能在测试/烧录过程中复位 U4 去读 SN**
 *       → 必须 app_state == APP_STATE_IDLE 才动手, 且每插入一次只读一次(标志位)。
 * 读 SN 需要 ISP: 空板本来就在 bootloader, 已烧过的板子会被复位进 bootloader,
 *       读完再复位回应用 —— 因此只做一次, 且只在空闲时做。
 */
static uint8_t  sn_done    = 0;   /* 1 = 本次插入已处理过 */
static uint8_t  in_cnt     = 0;   /* 连续"在位"计数 (去抖) */
static uint32_t poll_tick  = 0;

void SN_AutoPoll(void)
{
    uint8_t  okflag;
    uint16_t ch3;
    uint8_t  uid[SN_UID_LEN];

    /* ★★[2026-11-15] 两条都必须在最前面 ★★
     * ① 只在空闲时轮询: 测试/烧录期间绝不能复位 U4 去读 SN。
     * ② 更要紧的是保护 ADC: BSP_ADC_ReadRaw() 每次都会重新配置 Rank1 并启停
     *    转换, 而 ADC 一致性校验(0x0020 LEN=2) 和 ADC 极值校准(0x0010)
     *    也在用同一个 ADC1 —— 轮询插进它们的转换中间会把读数搞乱
     *    (现场表现为"一致性校验的 U1/U7 数值老是变")。
     *    所以非空闲时直接返回, 连 ADC 都不碰。 */
    if (!APP_IsIdle()) { in_cnt = 0; return; }

    if ((uint32_t)(HAL_GetTick() - poll_tick) < SN_DET_POLL_MS) return;
    poll_tick = HAL_GetTick();

    /* 在位判断用【U1 自己的 ADC】读通道 2(PA0), 不是 U4 上报的值。
     * ★[2026-11-15 实测] 拔板 = 1(个位数), 插板 = 2065(2000多) → 用它判断。
     * ★为什么不是通道 3/4: 通道 4(CH3) 拔板后仍有 1367 不回落, 用它会导致
     *   标志位永远不清除 → 表现为"只有第一块板能上报 SN"(现场踩过)。
     * ★为什么不用 U4 上报的 ADC: 4 路经 LM358 跟随器, 输入高阻, U4 下电后
     *   前端电容把最后一次读数记住(DUT 侧拔板后仍停在 2064), 无法检测拔板。 */
    ch3 = BSP_ADC_ReadRaw(SN_DET_ADC_CH, &okflag);
    if (!okflag) return;                       /* ADC 读失败, 下轮再说 */

    if (ch3 > SN_DET_PRESENT_RAW) {
        if (in_cnt < 255U) in_cnt++;
        /* ★三个条件同时满足才动手:
         *   ① 连续 4 次确认在位 (去抖, 防插拔瞬间误触发)
         *   ② 本次插入还没读过
         *   ③ 系统空闲 (上面已判过, 这里保留一次以防被改成异步) */
        if (in_cnt >= SN_DET_DEBOUNCE && !sn_done && APP_IsIdle()) {
            sn_done = 1;                       /* 先置位, 防重入 */
            if (APP_SnReadUid(uid) == 0) {     /* 0 = 读成功 */
                memcpy(last_uid, uid, SN_UID_LEN);
                SN_SendUidFrame(uid);
            } else {
                sn_done = 0;                   /* 读失败 → 清标志, 下次重试 */
            }
        }
    } else if (ch3 < SN_DET_ABSENT_RAW) {
        in_cnt  = 0;
        sn_done = 0;                           /* 拔板 → 清标志, 下次插入重读 */
    }
}
