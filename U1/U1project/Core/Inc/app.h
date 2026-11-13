#ifndef __APP_H
#define __APP_H

#include "main.h"
#include <stdint.h>

/*===== 系统状态 =====*/
typedef enum {
    APP_STATE_INIT,
    APP_STATE_IDLE,
    APP_STATE_UPLOADING,
    APP_STATE_PROGRAMMING,
    APP_STATE_TESTING,
} APP_State_t;

/*===== RGB 颜色 =====*/
typedef enum {
    RGB_OFF     = 0,
    RGB_RED     = 1,
    RGB_GREEN   = 2,
    RGB_BLUE    = 3,
    RGB_YELLOW  = 4,
    RGB_CYAN    = 5,
    RGB_MAGENTA = 6,
    RGB_WHITE   = 7,
} RGB_Color_t;

/*===== 经理协议 命令码 =====*/
#define MGR_CMD_QUERY_U1_VER    0x0004   /* 查询 U1 版本 */
#define MGR_CMD_QUERY_U7_VER    0x0008   /* 查询 U7 版本 */
#define MGR_CMD_FW_START        0x0009   /* U4 固件 起始帧 (含3B版本号) */
#define MGR_CMD_FW_DATA         0x000A   /* U4 固件 数据帧 */
#define MGR_CMD_FW_END          0x000B   /* U4 固件 结束帧 */
#define MGR_CMD_QUERY_FW_VER    0x000C   /* 查询固件版本 (从Flash读取) */
#define MGR_CMD_QUERY_U4_VER    0x000D   /* 查询U4运行时版本 (经U4协议0x32) */
#define MGR_CMD_ADC_CALIB       0x0010   /* ADC 极值校准 */
#define MGR_CMD_DIAM_CALIB      0x0011   /* 测径精度校准 (汇总25B ACK) */
#define MGR_CMD_MOTOR_HOME      0x0012   /* [2026-08-20] 电机回零 (转发 U7 0x29) */
#define MGR_CMD_MOTOR_SEQ       0x0013   /* [2026-08-20] 定点移动序列: 回零→1mm→1.5mm→2mm */
#define MGR_CMD_ADC_RESULT      0x0014   /* ADC 测试结果 (治具自发, 16B) */
#define MGR_CMD_QUERY_ADC       0x0020   /* 查询ADC (统一, Target:1=U1/4=U4/7=U7) */
#define MGR_CMD_SN_UPLOAD       0x0024   /* U4 SN 码上传 (治具自发, 16B) */
#define MGR_CMD_MOTOR_FWD       0x0025   /* [2026-11-13] 电机前进固定步数 (不回零, 点动用) */
#define MGR_CMD_MOTOR_BACK      0x0026   /* [2026-11-13] 电机后退固定步数 (不回零, 点动用) */
#define MGR_CMD_QUERY_U4_RAW    0x0027   /* [2026-11-13] 调试: 读最后一帧 U4 0x30 原始 24B 载荷
                                          * ACK: LEN=00 19, DATA=[1B status][24B 原样载荷] */
#define MGR_CMD_RESET_U4        0x0028   /* [2026-11-14] 调试: 单独给 U4 做一次硬复位
                                          * (BOOT0低 + 复位脉冲), 用于隔离验证"复位是否生效"。
                                          * 不编程、不擦除, 只复位。ACK 1B: 0x00 已执行 */
#define MOTOR_JOG_STEPS           800U   /* 点动步数 = 0.5mm (800步 ÷ 1600步/mm)
                                          * [2026-11-13 ★重新标定] 原 1600 步按 3200步/mm 算,
                                          * 实测实际标定是 1600步/mm(见 ADC_CALIB_SWEEP_STEPS 注释),
                                          * 所以 1600 步实际走了 1mm, 不是 0.5mm → 改为 800。 */
#define MGR_CMD_U4_SET_PERIOD   0x002B   /* U4 设置报文周期 (转发 0x18, LEN=2 [period_ms 2B 大端]) */

/*===== 治具测试 命令码 (0x011x, 避开经理协议) =====*/
#define APP_CMD_PING            0x0111
#define APP_CMD_PROGRAM         0x0112   /* SPI Flash → U4 烧录 */
#define APP_CMD_TEST_U4         0x0113
#define APP_CMD_TEST_U7         0x0114
#define APP_CMD_TEST_ALL        0x0115

/*===== 通用状态码 =====*/
#define APP_STATUS_OK              0x00
#define APP_STATUS_BUSY            0x01
#define APP_STATUS_ERR_SIZE        0x02   /* 文件大小异常 */
#define APP_STATUS_ERR_FLASH       0x03   /* Flash 读写失败 */
#define APP_STATUS_ERR_PROG        0x04   /* 烧录失败 */
#define APP_STATUS_ERR_TEST        0x05   /* 测试失败 */
#define APP_STATUS_ERR_NO_FW       0x06   /* 固件不存在 (大小为0或未写) */
#define APP_STATUS_ERR_INCOMPLETE  0x07   /* 固件不完整 (完整标志缺失/字节数不符) */
#define APP_STATUS_ERR_CRC         0x08   /* 帧 CRC 错误 */
#define APP_STATUS_ERR_SEQ         0x09   /* 帧序号错误 (非递增) */
#define APP_STATUS_ERR_COUNT       0x0A   /* 帧数/字节数不匹配 */
#define APP_STATUS_ERR_PARAM       0x0B   /* 参数错误 */
#define APP_STATUS_ERR_TIMEOUT     0x0C   /* 超时 */
#define APP_STATUS_ERR_OVERFLOW    0x0D   /* 上传溢出 (超出声明大小) */
#define APP_STATUS_ERR_NIY         0xFF   /* Not Implemented Yet */

/*===== SPI Flash 双槽布局 =====
 * 每槽头部 8 字节, 数据从 base+8 开始存放
 *   [0..3]  文件大小 (u32, little-endian, 断电不丢失)
 *   [4..6]  版本号 (3B: 主.次.修订, 大端展示)
 *   [7]     完整标志 (0xA5=完整, 0x00=未完成/写入中)
 */
#define FW_SLOT_SIZE               0x10000U    /* 64KB 每槽 */
#define FW_SLOT0_BASE              0x10000U    /* Bootloader 存储区 */
#define FW_SLOT1_BASE              0x20000U    /* App 存储区 */

/* U4 (DUT) 固件内存布局 (固件架构固定, 2026-08-11 用户确认):
 *   Bootloader: 0x08000000, 分配区 0x5000 (20KB)
 *   App:        0x08005000, 分配区 0xA000 (40KB)
 * U1 烧录 App 必须用固定 U4_APP_BASE, 不能用实际 Bootloader bin 大小推算 */
#define U4_BOOT_REGION             0x5000U     /* U4 Bootloader 分配区大小 */
#define U4_APP_BASE                0x08005000U /* U4 App 起始地址 (固定) */

#define FW_HEAD_SIZE               8U
#define FW_HEAD_OFF_SIZE           0U          /* 4B 文件大小 */
#define FW_HEAD_OFF_VER            4U          /* 3B 版本号 */
#define FW_HEAD_OFF_FLAG           7U          /* 1B 完整标志 */
#define FW_FLAG_COMPLETE           0xA5U
#define FW_FLAG_INCOMPLETE         0xFFU  /* 全1=擦除态, END时写0xA5只需1→0 */

/*===== ADC 测试结果格式 (CMD=0x0014) =====
 * 4 通道 × 4B = 16B
 *   [0]    Channel (1=VO1, 2=VO2, 3=AMS5600, 4=VDD)
 *   [1..2] ADC 原始值 (u16, 大端, 12-bit 0..4095)
 *   [3]    Status  (0=error, 1=OK)
 */
#define APP_ADC_RESULT_CHANNELS    4U
#define APP_ADC_RESULT_LEN         (APP_ADC_RESULT_CHANNELS * 4U)   /* 16 */

/*===== ADC 一致性校验 (CMD 0x0020 LEN=2 全芯片模式) =====*/
#define ADC_CONSISTENCY_THRESHOLD_DEFAULT  20U   /* 默认 LSB 容忍阈值 */
#define ADC_CONSISTENCY_TIMEOUT_MS         5000U /* 单芯片收集超时 */
#define ADC_CONSISTENCY_RESULT_LEN         64U   /* 48B ADC + 16B 判定区 */
#define ADC_CONSISTENCY_TOKEN_ALL          0x00U /* LEN=2 时 DATA[0] 为 0x00 表示全芯片 */
/* [2026-11-13] U4 CH3(ams_adc) 标度补偿: 实测 U4 只有 U1/U7 同测点的 0.849~0.860 倍
 * (4 块板实测, 离散 ±0.7%), 疑为 U4 PA6 处有分压。比对前乘 117/100 归一化,
 * 否则 CH3 恒偏 ~290 LSB 必判 FAIL。★仅影响一致性比对, ACK 里仍回 U4 原始值。 */
#define ADC_CONSIST_U4_AMS_NUM            117U   /* 分子 (实测反比 1/0.855 ≈ 1.17) */
#define ADC_CONSIST_U4_AMS_DEN            100U   /* 分母 */

/*===== ADC 极值自动校准 (CMD 0x0010) =====
 * 流程: [回零] → U4开始校准(0x11) → CCW去3mm → 等到位 → CW回4mm → 等到位
 *       → U4完成校准(0x12) → 延迟 → 读U4报文 min/max → 阈值判定 → 回9B ACK
 */
#define ADC_CALIB_MOTOR_ID                 0U  /* 电机编号, 与 U7 MOTOR_ID 一致 */
#define ADC_CALIB_SKIP_HOME                1   /* [2026-11-13] 1=跳过开头的回零
                                                * 原因①: ③ 执行前物件已贴在后挡块(零点),
                                                *        再回零只会以 0.28A 空顶挡块约 8 秒;
                                                * 原因②: 即使物件不在挡块上, 回零的 2.5mm 轻压
                                                *        也会让起点偏离零点, 反而更糟。
                                                * ★前提: 操作员须先把物件推到后挡块。
                                                * 自校正: 回程比去程多走 1mm(4mm vs 3mm), 故意
                                                *        顶住后挡块 → 每轮结束自动复位零点,
                                                *        下一轮起点仍是零点, 不会累积漂移。 */
#define ADC_CALIB_SWEEP_UM              3000U  /* 3mm 去程 (um) */
#define ADC_CALIB_SWEEP_STEPS           4800U  /* [2026-11-13 ★重新标定] 3000um × 1600步/mm = 4800
                                                * 实测标定: 5次点动(8000步)走完 4.2mm 全程 → 1905 步/mm;
                                                * 稳定段 1600步走 1033um → 1549 步/mm。取 1600 步/mm。
                                                * ★原按 3200步/mm 算的 9600 步, 实际会走 6mm,
                                                *   直接冲出 4.2mm 的机构行程 → 撞限位! */
#define ADC_CALIB_RETURN_STEPS          5120U  /* [2026-11-13 ★重新标定] 回程 3.2mm × 1600步/mm = 5120
                                                * (比去程 3mm 多 0.2mm, 顶住后挡块自校正零点) */
#define ADC_CALIB_MOTOR_DELAY_MS          500U  /* 电机物理到位等待 (每次移动后) */
#define ADC_CALIB_READ_DELAY_MS           200U  /* 完成校准后读极值前延迟 */
/* 阈值判定 (逐通道, 任一条件成立即该通道失败)
 * [2026-08-24] MAX-MIN 方向: ">2000 失败" (摆幅过大视为异常)
 * ★[2026-11-13] MAX_DIFF 2000→3500:
 *   2000 是按【旧旋转辊机构 15.5mm/圈】标定的 —— 当时走 3mm 只有 0.19 圈,
 *   传感器摆幅很小。换成【丝杆 1mm/圈】后, 走 3mm = 3 圈, 摆幅是原来的约 15 倍。
 *   实测两通道摆幅 2801 / 2582 → 2000 必然误判。
 *   取 3500 (实测 2801 上留约 25% 余量), 实测 MAX 3366/3085 (≤4000 ✓), MIN 565/503 (≥50 ✓)。
 *   注意: 放宽后本条测试实际拦的是 ADC 饱和(>4000)/贴底(<50)/U4未填数据,
 *   对"摆幅"的约束力减弱。若要拦异常摆动, 应把 ADC 换算成位移再判, 另做。 */
#define ADC_CALIB_MAX_DIFF              3500U  /* 同通道 MAX-MIN 上限, 超过则失败 */
#define ADC_CALIB_MAX_VAL               4000U  /* MAX 上限 */
#define ADC_CALIB_MIN_VAL                 50U  /* MIN 下限 */
#define ADC_CALIB_RESULT_LEN               9U  /* [status] + 4 极值 × 2B */

/*===== 缓冲区 / 时序 =====*/
#define APP_PAGE_SIZE              256
#define APP_FRAME_BUF_SIZE         (8 + 256 + 2 + 8)   /* 头+数据+CRC+余量 = 274 */
#define APP_UPLOAD_TIMEOUT_MS      10000U              /* 上传 10s 无数据超时 */

/*===== API =====*/
void APP_Init(void);
void APP_Run(void);
void APP_USB_Receive(uint8_t *data, uint32_t len);

#endif
