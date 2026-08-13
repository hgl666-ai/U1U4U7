#include "app.h"
#include "app_internal.h"
#include "app_frame.h"
#include "fw_upload.h"
#include "adc_consistency.h"
#include "adc_calib.h"
#include "diam_calib.h"
#include "usbd_cdc_if.h"
#include "protocol_frame.h"
#include "gd25q128.h"
#include "isp_programmer.h"
#include "protocol_u4.h"
#include "protocol_u7.h"
#include "bsp_gpio.h"
#include "bsp_adc.h"
#include "bsp_uart.h"
#include "sn_report.h"
#include <string.h>
#include <stdio.h>

/*===== 全局状态 =====
 * 共享给各功能模块, 见 app_internal.h */
APP_State_t  app_state    = APP_STATE_INIT;
uint16_t     pc_seq       = 0;
static RGB_Color_t  rgb_color    = RGB_WHITE;
static uint8_t      rgb_blinking = 1;
static uint8_t      rgb_on       = 1;

/* 帧序号检测 */
static uint16_t last_seq     = 0;          /* 上次请求帧序号 (序号错检测) */
static uint8_t  seq_inited   = 0;          /* 是否已建立序号基线 */

/* PROGRAM: 分两段烧录 Bootloader + App */
static int8_t   prog_step    = -1;
static uint8_t  prog_phase   = 0;    /* 0=Bootloader, 1=App */
static uint32_t prog_slotsize[2];    /* 各槽固件大小 */

/* TEST */
static int8_t   test_step    = -1;
static uint8_t  test_fail    = 0;
static uint8_t  test_mode    = 0;

/* 延迟 ACK: 非阻塞 U7/U4 查询完成后回传 */
static uint16_t deferred_cmd    = 0;
static uint16_t deferred_seq    = 0;
static uint32_t deferred_tick   = 0;
static uint8_t  deferred_target = 0;

/*===== LED =====*/
void APP_SetRGB(RGB_Color_t color, uint8_t blinking)
{
    rgb_color = color; rgb_blinking = blinking; rgb_on = 1;
}

void APP_RGB_Update(void)
{
    static uint32_t last_toggle = 0;
    uint32_t now = HAL_GetTick();
    uint32_t period;
    switch (rgb_blinking) {
        case 0:  period = 0;   break;
        case 1:  period = 500; break;
        case 2:  period = 125; break;
        default: period = 0;   break;
    }
    if (period > 0 && (now - last_toggle >= period)) { rgb_on = !rgb_on; last_toggle = now; }
    else if (period == 0) { rgb_on = 1; }

    uint8_t r = 0, g = 0, b = 0;
    if (rgb_on) {
        switch (rgb_color) {
            case RGB_RED:     r = 1; break;
            case RGB_GREEN:   g = 1; break;
            case RGB_BLUE:    b = 1; break;
            case RGB_YELLOW:  r = 1; g = 1; break;
            case RGB_CYAN:    g = 1; b = 1; break;
            case RGB_MAGENTA: r = 1; b = 1; break;
            case RGB_WHITE:   r = 1; g = 1; b = 1; break;
            default: break;
        }
    }
    BSP_RGB_Set(r, g, b);
}

/*===== USB 发送 =====*/
void APP_Print(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    uint32_t tick = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t *)str, len) == USBD_BUSY) {
        if (HAL_GetTick() - tick > 50) break;
    }
}

/* 极简 uint16→十进制字符串打印 (替代 snprintf %u/%d, 省 printf 引擎 ~1.2KB) */
static void APP_PrintU16(uint16_t v)
{
    char b[6];
    int8_t i = 5;
    b[i] = 0;
    do { b[--i] = (char)('0' + v % 10); v /= 10; } while (v > 0);
    APP_Print(&b[i]);
}

/* ACK 帧: SEQ 回显请求序号 */
void APP_SendAck(uint16_t cmd, uint8_t *data, uint16_t len)
{
    uint8_t buf[FRAME_BUF_SIZE];
    uint16_t total = Frame_Build(buf, cmd, pc_seq, data, len);
    uint32_t tick = HAL_GetTick();
    while (CDC_Transmit_FS(buf, total) == USBD_BUSY) {
        if (HAL_GetTick() - tick > 50) break;
    }
}

/* 自发帧: SEQ=0 */
void APP_SendEvent(uint16_t cmd, uint8_t *data, uint16_t len)
{
    uint8_t buf[FRAME_BUF_SIZE];
    uint16_t total = Frame_Build(buf, cmd, 0, data, len);
    CDC_Transmit_FS(buf, total);
}

/*===== Flash 头部辅助: 读取某槽的 [大小 版本 标志] =====
 * @param slot    0=Bootloader, 1=App
 * @param p_size  输出文件大小 (4B little-endian 解码后)
 * @param p_ver   输出 3B 版本号 [主 次修订]
 * @param p_flag  输出完整标志
 */
void Flash_ReadHead(uint8_t slot, uint32_t *p_size, uint8_t p_ver[3], uint8_t *p_flag)
{
    uint8_t hdr[FW_HEAD_SIZE];
    uint32_t base = (slot == 0) ? FW_SLOT0_BASE : FW_SLOT1_BASE;
    GD25Q_ReadBuffer(hdr, base, FW_HEAD_SIZE);
    if (p_size) *p_size = (uint32_t)hdr[0] | ((uint32_t)hdr[1] << 8)
                          | ((uint32_t)hdr[2] << 16) | ((uint32_t)hdr[3] << 24);
    if (p_ver)  { p_ver[0] = hdr[4]; p_ver[1] = hdr[5]; p_ver[2] = hdr[6]; }
    if (p_flag) *p_flag = hdr[7];
}

/*===== Flash 头部辅助: 写入完整标志 (单字节独立页编程, 断电不丢失) =====
 * @return 0=成功, 非0=失败
 */
uint8_t Flash_WriteCompleteFlag(uint8_t slot, uint8_t flag)
{
    uint32_t base = (slot == 0) ? FW_SLOT0_BASE : FW_SLOT1_BASE;
    return GD25Q_WritePage(&flag, base + FW_HEAD_OFF_FLAG, 1) ? 0 : APP_STATUS_ERR_FLASH;
}

/*===== ADC 测试结果发送 (CMD=0x0014, 治具自发, 16B) =====
 * 4 通道 × 4B = 16B
 *   [0]    Channel (1=VO1, 2=VO2, 3=AMS5600, 4=VDD)
 *   [1..2] ADC 原始值 (u16 大端, 12-bit)
 *   [3]    Status  (0=error, 1=OK)
 */
static void APP_SendAdcResult(void)
{
    uint8_t buf[APP_ADC_RESULT_LEN];
    /* 通道定义: 1=VO1(PB0/CH8), 2=VO2(PB1/CH9), 3=AMS5600(PA0/CH0), 4=VDD(PA1/CH1) */
    static const uint32_t adc_ch[APP_ADC_RESULT_CHANNELS] = {
        ADC_CH_ADC0, ADC_CH_ADC1, ADC_CH_ADC2, ADC_CH_ADC3
    };
    static const uint8_t adc_id[APP_ADC_RESULT_CHANNELS] = { 1, 2, 3, 4 };

    for (uint8_t i = 0; i < APP_ADC_RESULT_CHANNELS; i++) {
        ADC_Value_t v = BSP_ADC_ReadChannel(adc_ch[i], 4);
        uint16_t raw = v.valid ? v.raw : 0;
        uint8_t  st  = v.valid ? 1 : 0;
        buf[i * 4 + 0] = adc_id[i];
        buf[i * 4 + 1] = (uint8_t)(raw >> 8);
        buf[i * 4 + 2] = (uint8_t)(raw & 0xFF);
        buf[i * 4 + 3] = st;
    }
    APP_SendEvent(MGR_CMD_ADC_RESULT, buf, APP_ADC_RESULT_LEN);
}

/*===== PROGRAM (非阻塞状态机, 双固件: Bootloader→App) =====*/
static void APP_ProgramStep(void)
{
    /* 当前段偏移和大小(在case内复用) */
    static uint32_t prog_offset = 0;    /* 当前段内已烧录字节 */
    static uint32_t cur_size    = 0;    /* 当前段总大小 */
    static uint32_t cur_base    = 0;    /* 当前段在 Flash 的基地址 */
    static uint32_t cur_target  = 0;    /* 当前段在 U4 的起始地址 */

    switch (prog_step) {
    case 0: /* 握手 */
        U4_ScanPause(1);   /* ISP 期间暂停 U4 报文扫描, 防止吃掉 0x79 响应 */
        if (ISP_Connect() != ISP_OK) {
            U4_ScanPause(0);
            uint8_t e = APP_STATUS_ERR_PROG; APP_SendAck(APP_CMD_PROGRAM, &e, 1);
            app_state = APP_STATE_IDLE; prog_step = -1; APP_SetRGB(RGB_RED, 0); return;
        }
        /* 新 U4 已接入: 读 UID 计算 SN, 主动上传 PC
         * [恢复] 测 UID 任务: 验证 0x11 读 UID 不干扰 G031 bootloader */
        SN_Report_OnProgram();
        prog_step = 1; return;

    case 1: /* 全片擦除 */
        if (ISP_MassErase() != ISP_OK) {
            U4_ScanPause(0);
            uint8_t e = APP_STATUS_ERR_PROG; APP_SendAck(APP_CMD_PROGRAM, &e, 1);
            app_state = APP_STATE_IDLE; prog_step = -1; APP_SetRGB(RGB_RED, 0); return;
        }
        prog_offset = 0; prog_step = 2; return;

    case 2: /* 烧录 Bootloader (slot0) → U4 0x08000000 */
        prog_offset = 0;
        cur_base    = FW_SLOT0_BASE;
        cur_size    = prog_slotsize[0];
        cur_target  = 0x08000000U;
        prog_phase  = 0;
        prog_step   = 10;   /* 跳到写循环 */
        return;

    case 3: /* 烧录 App (slot1) → U4 固定 0x08005000 (固件架构固定偏移, 不能用 bin 大小推算) */
        cur_base   = FW_SLOT1_BASE;
        cur_size   = prog_slotsize[1];
        cur_target = U4_APP_BASE;
        prog_offset = 0;
        prog_phase = 1;
        prog_step  = 10;
        return;

    case 10: /* 逐页写入 (当前段) */
        if (prog_offset < cur_size) {
            uint16_t chunk = (cur_size - prog_offset >= APP_PAGE_SIZE)
                           ? APP_PAGE_SIZE : (uint16_t)(cur_size - prog_offset);
            uint16_t wlen = (chunk + 7U) & ~7U;   /* G0 要求 8 字节对齐, 不足补 0xFF */
            uint8_t buf[APP_PAGE_SIZE];
            GD25Q_ReadBuffer(buf, cur_base + FW_HEAD_SIZE + prog_offset, chunk);
            if (wlen > chunk) memset(&buf[chunk], 0xFF, wlen - chunk);
            if (ISP_WriteMemory(cur_target + prog_offset, buf, wlen) != ISP_OK) {
                U4_ScanPause(0);
                uint8_t e = APP_STATUS_ERR_PROG; APP_SendAck(APP_CMD_PROGRAM, &e, 1);
                app_state = APP_STATE_IDLE; prog_step = -1; APP_SetRGB(RGB_RED, 0); return;
            }
            prog_offset += wlen;
            return;
        }
        /* 当前段写完: Bootloader(phase0)→case3烧App; App(phase1)→case4复位 */
        prog_step = (prog_phase == 0) ? 3 : 4;
        return;

    case 4: /* 跳转运行 */
        ISP_Go(0x08000000);   /* 先试 Go 直接跳转 */
        /* [2026-08-11] ISP_Go 在 G0 可能不可靠; 且 BOOT0(PA8) 全程保持高电平,
         * 须拉低 BOOT0 + 复位, 让 U4 从 flash 正常启动应用 */
        U4_BOOT0_LOW();
        U4_RESET_LOW();
        HAL_Delay(10);
        U4_RESET_HIGH();
        /* [2026-08-12] 切回 8N1 (App 运行时用 8N1, 与 ISP 的 8E1 区分) */
        U4_UART_SetMode(1);
        U4_ScanPause(0);   /* ISP 结束, 恢复 U4 报文扫描 */
        { uint8_t ok = APP_STATUS_OK; APP_SendAck(APP_CMD_PROGRAM, &ok, 1); }
        app_state = APP_STATE_IDLE; prog_step = -1; APP_SetRGB(RGB_GREEN, 0); return;
    }
}

/*===== TEST (非阻塞状态机) =====*/
static void APP_TestStep(void)
{
    int ret;
    switch (test_step) {

    /*── U4 连通 + 校准 ──*/
    case 0:  APP_Print("[TEST] U4 Ping...\r\n");     test_step++; break;
    case 1:
        ret = U4_Ping();
        if (ret == U4_PROTO_PENDING) break;
        if (ret != U4_PROTO_OK) { APP_Print("  FAIL: U4 offline\r\n"); test_fail = 1; }
        else                     { APP_Print("  PASS\r\n"); }
        test_step++; break;
    case 2:  APP_Print("[TEST] U4 Zero Sensor...\r\n"); test_step++; break;
    case 3:
        ret = U4_ZeroSensor();
        if (ret == U4_PROTO_PENDING) break;
        if (ret != U4_PROTO_OK) { APP_Print("  FAIL\r\n"); test_fail = 1; }
        else                     { APP_Print("  PASS\r\n"); }
        test_step++; break;
    case 4:  APP_Print("[TEST] U4 Start Calib...\r\n"); test_step++; break;
    case 5:
        ret = U4_StartCalib();
        if (ret == U4_PROTO_PENDING) break;
        if (ret != U4_PROTO_OK) { APP_Print("  FAIL\r\n"); test_fail = 1; }
        else                     { APP_Print("  PASS\r\n"); }
        test_step++; break;
    case 6:  APP_Print("[TEST] U4 Finish Calib...\r\n"); test_step++; break;
    case 7:
        ret = U4_FinishCalib();
        if (ret == U4_PROTO_PENDING) break;
        if (ret != U4_PROTO_OK) { APP_Print("  FAIL\r\n"); test_fail = 1; }
        else                     { APP_Print("  PASS\r\n"); }
        test_step++; break;
    case 8:  APP_Print("[TEST] U4 Read Flash Param...\r\n"); test_step++; break;
    case 9:
        ret = U4_ReadFlashParam();
        if (ret == U4_PROTO_PENDING) break;
        if (ret != U4_PROTO_OK) { APP_Print("  FAIL\r\n"); test_fail = 1; }
        else                     { APP_Print("  PASS\r\n"); }
        /* U4 在线时采集 4 路 ADC (VO1/VO2/AMS5600/VDD) 并以 CMD=0x0014 上报 */
        if (test_mode == 0 || test_mode == 1) {
            APP_Print("[TEST] ADC Channels (VO1/VO2/AMS5600/VDD)...\r\n");
            APP_SendAdcResult();
        }
        test_step = (test_mode == 0) ? 10 : 20;
        break;

    /*── U7 测试 ──*/
    case 10:
        BSP_U7_Enable();
        APP_Print("[TEST] U7 Ping...\r\n"); test_step++; break;
    case 11:
        ret = U7_Ping();
        if (ret == U7_PROTO_PENDING) break;
        if (ret != U7_PROTO_OK) { APP_Print("  FAIL: U7 offline\r\n"); test_fail = 1; }
        else                     { APP_Print("  PASS\r\n"); }
        test_step++; break;
    case 12: APP_Print("[TEST] U7 Version...\r\n"); test_step++; break;
    case 13: {
        uint8_t maj, min;
        ret = U7_GetVersion(&maj, &min);
        if (ret == U7_PROTO_PENDING) break;
        if (ret != U7_PROTO_OK) { APP_Print("  FAIL\r\n"); test_fail = 1; }
        else { APP_Print("  v"); APP_PrintU16(maj); APP_Print("."); APP_PrintU16(min); APP_Print("\r\n"); }
        test_step++; break;
    }
    case 14: APP_Print("[TEST] U7 SelfTest...\r\n"); test_step++; break;
    case 15: {
        uint8_t r;
        ret = U7_SelfTest(&r);
        if (ret == U7_PROTO_PENDING) break;
        if (ret != U7_PROTO_OK || r != 0) { APP_Print("  FAIL\r\n"); test_fail = 1; }
        else { APP_Print("  PASS\r\n"); }
        test_step = 20; break;
    }

    /*── 结果 ──*/
    case 20:
        BSP_U7_Disable();
        {
            uint16_t result_cmd = (test_mode == 1) ? APP_CMD_TEST_U4
                                : (test_mode == 2) ? APP_CMD_TEST_U7
                                : APP_CMD_TEST_ALL;
            if (test_fail) {
                APP_Print("=== TEST RESULT: FAIL ===\r\n");
                uint8_t e = APP_STATUS_ERR_TEST; APP_SendAck(result_cmd, &e, 1);
                APP_SetRGB(RGB_RED, 0);
            } else {
                APP_Print("=== TEST RESULT: PASS ===\r\n");
                uint8_t ok = APP_STATUS_OK; APP_SendAck(result_cmd, &ok, 1);
                APP_SetRGB(RGB_GREEN, 0);
            }
        }
        app_state = APP_STATE_IDLE; test_step = -1; break;

    default: test_step = 20; break;
    }
}

/*===== 帧命令分发 =====*/

static void APP_Dispatch(uint16_t cmd, uint8_t *data, uint16_t len)
{
    switch (cmd) {

    /*──────────────────────────────────────────────────────
     * 经理协议
     *──────────────────────────────────────────────────────*/

    /* 查询 U1 版本 */
    case MGR_CMD_QUERY_U1_VER: {
        uint8_t resp[3] = { 1, 0, 0 };          /* v1.0.0 */
        APP_SendAck(MGR_CMD_QUERY_U1_VER, resp, 3);
        break;
    }

    /* 查询 U7 版本: U1 向 U7 询问后回传 (非阻塞) */
    case MGR_CMD_QUERY_U7_VER: {
        uint8_t maj, min;
        int ret = U7_GetVersion(&maj, &min);
        if (ret == U7_PROTO_PENDING) {
            deferred_cmd = MGR_CMD_QUERY_U7_VER;
            break;
        }
        uint8_t resp[3];
        if (ret == U7_PROTO_OK) {
            resp[0] = 1; resp[1] = maj; resp[2] = min;
        } else {
            resp[0] = 0; resp[1] = 0; resp[2] = 0xFF;
        }
        APP_SendAck(MGR_CMD_QUERY_U7_VER, resp, 3);
        break;
    }

    /* 统一ADC查询 0x0020:
     *   LEN=1 DATA[0]=1/4/7 → 单芯片查询 (原有)
     *   LEN=2 DATA[0]=0x00 DATA[1]=Threshold → 全芯片一致性校验 (adc_consistency 模块) */
    case MGR_CMD_QUERY_ADC: {
        /*── LEN=2 全芯片模式 ──*/
        if (len >= 2 && data[0] == ADC_CONSISTENCY_TOKEN_ALL) {
            if (ADC_Consistency_Start(data[1]) != 0) {
                uint8_t e = APP_STATUS_BUSY; APP_SendAck(cmd, &e, 1);
            }
            break;  /* 不立即 ACK, 由 ADC_Consistency_Run 异步完成 */
        }

        /*── LEN=1 单芯片模式 (原有逻辑) ──*/
        if (len < 1) { uint8_t e = APP_STATUS_ERR_PARAM; APP_SendAck(cmd, &e, 1); break; }
        uint8_t target = data[0];
        uint8_t buf[16];
        int     ret;

        if (target == 1) {
            /* U1 自身: 直接读 */
            static const uint32_t ch[4] = {ADC_CH_ADC0, ADC_CH_ADC1, ADC_CH_ADC2, ADC_CH_ADC3};
            static const uint8_t  id[4] = {1, 2, 3, 4};
            for (uint8_t i = 0; i < 4; i++) {
                ADC_Value_t v = BSP_ADC_ReadChannel(ch[i], 4);
                buf[i * 4 + 0] = id[i];
                buf[i * 4 + 1] = (uint8_t)(v.raw >> 8);
                buf[i * 4 + 2] = (uint8_t)(v.raw & 0xFF);
                buf[i * 4 + 3] = v.valid ? 1 : 0;
            }
            APP_SendAck(MGR_CMD_QUERY_ADC, buf, 16);
            break;
        }

        if (target == 7) ret = U7_GetADC(buf);
        else if (target == 4) ret = U4_ReadAllADC(buf);
        else { uint8_t e = APP_STATUS_ERR_PARAM; APP_SendAck(cmd, &e, 1); break; }

        if (ret == U7_PROTO_PENDING || ret == U4_PROTO_PENDING) {
            deferred_cmd    = MGR_CMD_QUERY_ADC;
            deferred_target = target;
            deferred_tick   = HAL_GetTick();
            break;
        }
        if (ret == U7_PROTO_OK || ret == U4_PROTO_OK)
            APP_SendAck(MGR_CMD_QUERY_ADC, buf, 16);
        else {
            memset(buf, 0, 16);  /* 失败: 全通道status=0 */
            APP_SendAck(MGR_CMD_QUERY_ADC, buf, 16);
        }
        break;
    }

    /* 查询 U4 运行时版本: U1 向 U4 询问后回传 (非阻塞) */
    case MGR_CMD_QUERY_U4_VER: {
        uint8_t maj, min, rev;
        int ret = U4_GetVersion(&maj, &min, &rev);
        if (ret == U4_PROTO_PENDING) {
            deferred_cmd  = MGR_CMD_QUERY_U4_VER;
            deferred_tick = HAL_GetTick();
            break;
        }
        uint8_t resp[4];
        if (ret == U4_PROTO_OK) {
            resp[0] = 1; resp[1] = maj; resp[2] = min; resp[3] = rev;
        } else {
            resp[0] = 0; resp[1] = 0; resp[2] = 0xFF; resp[3] = 0xFF;
        }
        APP_SendAck(MGR_CMD_QUERY_U4_VER, resp, 4);
        break;
    }

    /* U4 设置报文周期 (U1 转发 0x18 到 U4)
     * PC → U1: LEN=2, DATA=[period_ms 2B 大端]; 实际生效以 U4 报文间隔为准 */
    case MGR_CMD_U4_SET_PERIOD: {
        if (len < 2) { uint8_t e = APP_STATUS_ERR_PARAM; APP_SendAck(MGR_CMD_U4_SET_PERIOD, &e, 1); break; }
        uint16_t period = ((uint16_t)data[0] << 8) | data[1];
        int ret = U4_SetReportPeriod(period);
        if (ret == U4_PROTO_PENDING) {
            deferred_cmd  = MGR_CMD_U4_SET_PERIOD;
            deferred_tick = HAL_GetTick();
            break;
        }
        uint8_t resp = (ret == U4_PROTO_OK) ? APP_STATUS_OK : APP_STATUS_ERR_PROG;
        APP_SendAck(MGR_CMD_U4_SET_PERIOD, &resp, 1);
        break;
    }

    /* U4 固件上传 — 起始/数据/结束 (fw_upload 模块) */
    case MGR_CMD_FW_START:
    case MGR_CMD_FW_DATA:
    case MGR_CMD_FW_END:
        FwUpload_Handle(cmd, data, len);
        break;

    /* 诊断: 读两槽头部 (检测双槽固件完整性)
     * ACK 16B: [0..7]=slot0@0x10000, [8..15]=slot1@0x20000
     *   每槽 8B: [4B 大小 LE][3B 版本][1B 完整标志 0xA5=COMPLETE] */
    case MGR_CMD_FLASH_HEAD: {
        uint8_t hdr[16];
        GD25Q_ReadBuffer(hdr, FW_SLOT0_BASE, FW_HEAD_SIZE);
        GD25Q_ReadBuffer(hdr + FW_HEAD_SIZE, FW_SLOT1_BASE, FW_HEAD_SIZE);
        APP_SendAck(MGR_CMD_FLASH_HEAD, hdr, 16);
        break;
    }

    /* 诊断: 读回 U4 flash 16B (验证烧录是否正确)
     * 请求 LEN=4, DATA=[4B 完整地址 大端]; 会进 U4 bootloader 读, 读完 U4 停在 bootloader
     * [2026-08-11] 修正: 原 LEN=3 把 0x08000000 错拼成 0x00080000 被 bootloader NACK */
    case MGR_CMD_U4_FLASH_READ: {
        if (len < 4) { uint8_t e = APP_STATUS_ERR_PARAM; APP_SendAck(MGR_CMD_U4_FLASH_READ, &e, 1); break; }
        uint32_t addr = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16)
                      | ((uint32_t)data[2] << 8) | (uint32_t)data[3];
        U4_ScanPause(1);
        U4_UART_SetMode(0);   /* 8E1 用于进 bootloader 读 (默认是 8N1) */
        if (ISP_Connect() != ISP_OK) {
            U4_UART_SetMode(1);   /* 恢复 8N1 */
            U4_ScanPause(0);
            uint8_t e = APP_STATUS_ERR_PROG; APP_SendAck(MGR_CMD_U4_FLASH_READ, &e, 1); break;
        }
        uint8_t buf[16];
        ISP_Result_t r = ISP_ReadMemory(addr, buf, 16);
        U4_UART_SetMode(1);   /* 恢复 8N1 */
        U4_ScanPause(0);
        if (r != ISP_OK) {
            uint8_t e = APP_STATUS_ERR_PROG; APP_SendAck(MGR_CMD_U4_FLASH_READ, &e, 1); break;
        }
        APP_SendAck(MGR_CMD_U4_FLASH_READ, buf, 16);
        break;
    }

    /* 诊断: 检测 U4 是否卡在 bootloader
     * 不改 BOOT0/不复位, 直接发 0x7F, 看是否回 0x79
     * ACK 1B: 1=U4 在 bootloader, 0=不在 (跑应用/挂死/断电) */
    case MGR_CMD_U4_BOOTSTATE: {
        uint8_t resp = 0;
        U4_ScanPause(1);
        U4_UART_SetMode(0);   /* 8E1 用于检测 bootloader */
        HAL_UART_DMAStop(&huart2);   /* 停 DMA, 用阻塞接收 */
        __HAL_UART_CLEAR_FLAG(&huart2, USART_SR_ORE | USART_SR_FE | USART_SR_NE);
        (void)huart2.Instance->DR;
        UART_SendByte(&huart2, 0x7F);
        uint32_t tick = HAL_GetTick();
        uint8_t b;
        while ((HAL_GetTick() - tick) < 100) {
            if (HAL_UART_Receive(&huart2, &b, 1, 50) == HAL_OK) {
                if (b == 0x79) { resp = 1; }
                break;   /* 收到字节即判断 (0x79=bootloader, 其他=不是) */
            }
        }
        U4_UART_SetMode(1);   /* 恢复 8N1 */
        U4_ScanPause(0);
        APP_SendAck(MGR_CMD_U4_BOOTSTATE, &resp, 1);
        break;
    }

    /* 查询固件版本: 从 SPI Flash 头部读取 (不依赖 U4 上电)
     * ACK 4B: [1B 状态码][3B 版本号]
     *   状态码 0x00=OK  0x06=无固件  0x07=固件不完整
     * 优先返回 App 槽 (slot1); 若 App 槽空, 返回 Bootloader 槽 (slot0);
     * 两槽都空时返回 NO_FW
     */
    case MGR_CMD_QUERY_FW_VER: {
        uint8_t  slot   = 1;
        uint32_t size   = 0;
        uint8_t  ver[3] = {0};
        uint8_t  flag   = FW_FLAG_INCOMPLETE;
        Flash_ReadHead(slot, &size, ver, &flag);
        if (size == 0 || size > FW_SLOT_SIZE - FW_HEAD_SIZE) {
            /* App 槽无固件, 退回查 Bootloader 槽 */
            slot = 0;
            Flash_ReadHead(slot, &size, ver, &flag);
        }
        uint8_t resp[4];
        if (size == 0 || size > FW_SLOT_SIZE - FW_HEAD_SIZE) {
            resp[0] = APP_STATUS_ERR_NO_FW;
            resp[1] = resp[2] = resp[3] = 0;
        } else if (flag != FW_FLAG_COMPLETE) {
            resp[0] = APP_STATUS_ERR_INCOMPLETE;
            resp[1] = ver[0]; resp[2] = ver[1]; resp[3] = ver[2];
        } else {
            resp[0] = APP_STATUS_OK;
            resp[1] = ver[0]; resp[2] = ver[1]; resp[3] = ver[2];
        }
        APP_SendAck(MGR_CMD_QUERY_FW_VER, resp, 4);
        break;
    }

    /* 暂未实现 */
    case MGR_CMD_U1_IAP_START:
    case MGR_CMD_U1_IAP_DATA:
    case MGR_CMD_U1_IAP_END:
    case MGR_CMD_U7_IAP_START:
    case MGR_CMD_U7_IAP_DATA:
    case MGR_CMD_U7_IAP_END: {
        uint8_t e = APP_STATUS_ERR_NIY; APP_SendAck(cmd, &e, 1); break;
    }

    /* ADC 极值校准 (adc_calib 模块) */
    case MGR_CMD_ADC_CALIB: {
        if (ADC_Calib_Start() != 0) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(cmd, &e, 1);
        }
        break;  /* 不立即 ACK, 由 ADC_Calib_Run 异步完成 */
    }

    /* 测径精度校准 (diam_calib 模块) */
    case MGR_CMD_DIAM_CALIB: {
        if (DiamCalib_Start() != 0) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(cmd, &e, 1);
        }
        break;  /* 不立即 ACK, 由 DiamCalib_Run 异步完成 */
    }

    /*──────────────────────────────────────────────────────
     * 治具测试命令
     *──────────────────────────────────────────────────────*/

    case APP_CMD_PING: {
        uint8_t ok = APP_STATUS_OK;
        APP_SendAck(APP_CMD_PING, &ok, 1);
        break;
    }

    case APP_CMD_PROGRAM: {
        if (app_state != APP_STATE_IDLE) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(APP_CMD_PROGRAM, &e, 1); break;
        }
        /* 读两槽头部: [大小 版本 完整标志] (双槽: slot0=Bootloader, slot1=App) */
        uint8_t  ver0[3], ver1[3], flag0, flag1;
        Flash_ReadHead(0, &prog_slotsize[0], ver0, &flag0);
        Flash_ReadHead(1, &prog_slotsize[1], ver1, &flag1);

        /* 检查"有无固件" (两槽都须有效) */
        if (prog_slotsize[0] == 0 || prog_slotsize[0] > FW_SLOT_SIZE - FW_HEAD_SIZE
         || prog_slotsize[1] == 0 || prog_slotsize[1] > FW_SLOT_SIZE - FW_HEAD_SIZE) {
            uint8_t e = APP_STATUS_ERR_NO_FW; APP_SendAck(APP_CMD_PROGRAM, &e, 1); break;
        }
        /* 检查"完整标志" (两槽都须 COMPLETE) */
        if (flag0 != FW_FLAG_COMPLETE || flag1 != FW_FLAG_COMPLETE) {
            uint8_t e = APP_STATUS_ERR_INCOMPLETE; APP_SendAck(APP_CMD_PROGRAM, &e, 1); break;
        }
        /* Bootloader 实际大小不得超过分配区 0x5000, 否则与 App 区域 (0x08005000) 重叠 */
        if (prog_slotsize[0] > U4_BOOT_REGION) {
            uint8_t e = APP_STATUS_ERR_SIZE; APP_SendAck(APP_CMD_PROGRAM, &e, 1); break;
        }
        /* [2026-08-12] 切 8E1 用于 ISP (G0 bootloader 用 8E1, App 运行时 8N1) */
        U4_UART_SetMode(0);
        prog_phase = 0; prog_step = 0; app_state = APP_STATE_PROGRAMMING;
        APP_SetRGB(RGB_YELLOW, 2);
        break;
    }

    case APP_CMD_TEST_U4:
        if (app_state != APP_STATE_IDLE) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(APP_CMD_TEST_U4, &e, 1); break;
        }
        test_step = 0; test_fail = 0; test_mode = 1;
        app_state = APP_STATE_TESTING; APP_SetRGB(RGB_CYAN, 2);
        break;

    case APP_CMD_TEST_U7:
        if (app_state != APP_STATE_IDLE) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(APP_CMD_TEST_U7, &e, 1); break;
        }
        test_step = 10; test_fail = 0; test_mode = 2;
        app_state = APP_STATE_TESTING; APP_SetRGB(RGB_CYAN, 2);
        break;

    case APP_CMD_TEST_ALL:
        if (app_state != APP_STATE_IDLE) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(APP_CMD_TEST_ALL, &e, 1); break;
        }
        test_step = 0; test_fail = 0; test_mode = 0;
        app_state = APP_STATE_TESTING; APP_SetRGB(RGB_CYAN, 2);
        break;

    default: {
        uint8_t e = APP_STATUS_ERR_NIY; APP_SendAck(cmd, &e, 1); break;
    }
    }
}

/*===== 公开 API =====*/

void APP_Init(void)
{
    app_state   = APP_STATE_INIT;
    prog_step   = -1;
    test_step   = -1;
    last_seq    = 0;
    seq_inited  = 0;
    deferred_cmd = 0;

    APP_Frame_Init();
    FwUpload_Init();
    ADC_Consistency_Init();
    ADC_Calib_Init();
    DiamCalib_Init();
    SN_Report_Init();

    APP_SetRGB(RGB_WHITE, 0);
    APP_Print("\r\n=================================\r\n");
    APP_Print("  U1 Jig System Ready\r\n");
    APP_Print("=================================\r\n");
    app_state = APP_STATE_IDLE;
    APP_SetRGB(RGB_BLUE, 1);
}

void APP_Run(void)
{
    if (APP_Frame_Pending()) {
        uint16_t f_len;
        uint8_t *fb = APP_Frame_Consume(&f_len);
        uint16_t cmd, len, seq;
        uint8_t data[FRAME_MAX_DATA];
        /* Frame_Parse 已做 CRC 校验, 失败代表帧错或 CRC 错 */
        if (Frame_Parse(fb, f_len, &cmd, &seq, data, &len)) {
            /* 帧序号检测: seq != 0 时应 = last_seq + 1 (溢出回绕到 1)
             * 不拒绝帧, 仅在 USB 端口打印警告便于 PC 调试 */
            if (seq != 0) {
                if (seq_inited) {
                    uint16_t expect = (last_seq == 0xFFFF) ? 1 : (last_seq + 1);
                    if (seq != expect) {
                        APP_Print("[WARN] SEQ mismatch: expect=");
                        APP_PrintU16(expect);
                        APP_Print(", got=");
                        APP_PrintU16(seq);
                        APP_Print("\r\n");
                    }
                }
                last_seq = seq;
                seq_inited = 1;
            }
            pc_seq = seq;
            APP_Dispatch(cmd, data, len);
        } else {
            /* CRC 错或帧长度异常: 因 SEQ 不可信, 无法回标准 ACK, 仅记录诊断信息 */
            APP_Print("[WARN] Frame CRC/length error, dropped\r\n");
        }
    }

    APP_RGB_Update();

    /* 诊断: 每3秒自动U7 PING (校准时跳过, 避免与电机控制冲突) */
    if (!ADC_Calib_IsBusy() && !DiamCalib_IsBusy()) {
        static uint32_t pt = 0; static uint8_t ps = 0;
        if (HAL_GetTick() - pt > 3000 && ps == 0) { U7_Ping(); pt = HAL_GetTick(); ps = 1; }
        if (ps == 1) { int r = U7_Ping(); if (r != U7_PROTO_PENDING) { ps = 0; } }
    }

    /* 非阻塞协议状态机推进 */
    U7_Proto_Run();
    U4_Proto_Run();

    /* ADC 一致性校验非阻塞推进 (CMD 0x0020 LEN=2 全芯片模式) */
    ADC_Consistency_Run();

    /* ADC 极值校准非阻塞推进 (CMD 0x0010) */
    ADC_Calib_Run();

    /* 测径精度校准非阻塞推进 (CMD 0x0011) */
    DiamCalib_Run();

    /* 延迟 ACK: U7/U4 版本查询需要等待对方回复 */
    if (deferred_cmd == MGR_CMD_QUERY_U7_VER) {
        uint8_t maj, min;
        int ret = U7_GetVersion(&maj, &min);
        if (ret != U7_PROTO_PENDING) {
            uint8_t resp[3];
            if (ret == U7_PROTO_OK) {
                resp[0] = 1; resp[1] = maj; resp[2] = min;
            } else {
                resp[0] = 0; resp[1] = 0; resp[2] = 0xFF;
            }
            APP_SendAck(MGR_CMD_QUERY_U7_VER, resp, 3);
            deferred_cmd = 0;
        }
    } else if (deferred_cmd == MGR_CMD_QUERY_U4_VER) {
        uint8_t maj, min, rev;
        int ret = U4_GetVersion(&maj, &min, &rev);
        /* U4不在线时3秒超时返回离线 */
        if (ret == U4_PROTO_PENDING && HAL_GetTick() - deferred_tick > 3000) {
            uint8_t resp[4] = {0, 0, 0xFF, 0xFF};
            APP_SendAck(MGR_CMD_QUERY_U4_VER, resp, 4);
            deferred_cmd = 0;
        }
        if (ret != U4_PROTO_PENDING) {
            uint8_t resp[4];
            if (ret == U4_PROTO_OK) {
                resp[0] = 1; resp[1] = maj; resp[2] = min; resp[3] = rev;
            } else {
                resp[0] = 0; resp[1] = 0; resp[2] = 0xFF; resp[3] = 0xFF;
            }
            APP_SendAck(MGR_CMD_QUERY_U4_VER, resp, 4);
            deferred_cmd = 0;
        }
    } else if (deferred_cmd == MGR_CMD_U4_SET_PERIOD) {
        int ret = U4_SetReportPeriod(0);   /* period 参数仅在首调(IDLE)时用, 会话已发出 */
        if (ret == U4_PROTO_PENDING && HAL_GetTick() - deferred_tick > 3000) {
            uint8_t e = APP_STATUS_ERR_TIMEOUT; APP_SendAck(MGR_CMD_U4_SET_PERIOD, &e, 1);
            deferred_cmd = 0;
        }
        if (ret != U4_PROTO_PENDING) {
            uint8_t resp = (ret == U4_PROTO_OK) ? APP_STATUS_OK : APP_STATUS_ERR_PROG;
            APP_SendAck(MGR_CMD_U4_SET_PERIOD, &resp, 1);
            deferred_cmd = 0;
        }
    } else if (deferred_cmd == MGR_CMD_QUERY_ADC) {
        uint8_t buf[16]; int ret;
        if (deferred_target == 7) ret = U7_GetADC(buf);
        else                       ret = U4_ReadAllADC(buf);
        if (ret == U7_PROTO_PENDING || ret == U4_PROTO_PENDING) {
            if (HAL_GetTick() - deferred_tick > 3000) {
                memset(buf, 0, 16);
                APP_SendAck(MGR_CMD_QUERY_ADC, buf, 16);
                deferred_cmd = 0;
            }
        } else {
            if (ret != U7_PROTO_OK && ret != U4_PROTO_OK)
                memset(buf, 0, 16);
            APP_SendAck(MGR_CMD_QUERY_ADC, buf, 16);
            deferred_cmd = 0;
        }
    }

    /* 固件上传超时检查 (fw_upload 模块) */
    FwUpload_TimeoutCheck();

    switch (app_state) {
        case APP_STATE_PROGRAMMING: APP_ProgramStep(); break;
        case APP_STATE_TESTING:     APP_TestStep();    break;
        default: break;
    }
}

void APP_USB_Receive(uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        APP_Frame_Feed(data[i]);
    }
}
