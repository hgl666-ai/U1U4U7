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
/* U1/U7 IAP (0x0001-0x0003, 0x0005-0x0007) 已删除, ★ NIY 未实现未使用 */
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
/* 0x0025/0x0026/0x0027/0x0029/0x002A 诊断命令已删除 (腾 Flash) */
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

/*===== ADC 极值自动校准 (CMD 0x0010) =====*/
#define ADC_CALIB_STEPS_PER_READ         100U  /* 每次 ADC 采样前电机步数 */
#define ADC_CALIB_READS_PER_DIR            4U  /* 每个方向采样次数 */
#define ADC_CALIB_CYCLES                   3U  /* 往复循环数 */
#define ADC_CALIB_MOVE_TIMEOUT_MS      10000U  /* 单次电机移动超时 */
#define ADC_CALIB_MOTOR_DELAY_MS          500U  /* 电机物理到位等待 */
#define ADC_CALIB_ADC_TIMEOUT_MS        1000U  /* ADC 读取超时 */
#define ADC_CALIB_MOTOR_ID                 0U  /* 电机编号, 与 U7 MOTOR_ID 一致 */
#define ADC_CALIB_RESULT_LEN               8U  /* 4 极值 × 2B */

/*===== 缓冲区 / 时序 =====*/
#define APP_PAGE_SIZE              256
#define APP_FRAME_BUF_SIZE         (8 + 256 + 2 + 8)   /* 头+数据+CRC+余量 = 274 */
#define APP_UPLOAD_TIMEOUT_MS      10000U              /* 上传 10s 无数据超时 */

/*===== API =====*/
void APP_Init(void);
void APP_Run(void);
void APP_USB_Receive(uint8_t *data, uint32_t len);

#endif
