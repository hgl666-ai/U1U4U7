#include "fw_upload.h"
#include "app.h"
#include "app_internal.h"
#include "gd25q128.h"
#include <string.h>

/*===== 固件上传状态 (CMD 0x0009/0x000A/0x000B) =====
 * 双固件支持, slot0=Bootloader, slot1=App
 */
static uint32_t upload_addr         = 0;   /* Flash 写入地址 (base+8 起) */
static uint32_t upload_total        = 0;   /* 声明的文件大小 (字节) */
static uint32_t upload_count        = 0;   /* 已写入字节数 */
static uint16_t upload_total_frames = 0;   /* 声明的总帧数 */
static uint16_t upload_data_frames  = 0;   /* 已收到的数据帧数 */
static uint8_t  upload_version[3]   = {0}; /* 当前槽固件版本号 */
static uint8_t  page_buf[APP_PAGE_SIZE];
static uint16_t page_idx             = 0;
static uint8_t  upload_slot          = 0;  /* 0=Bootloader, 1=App */
static uint32_t upload_last_tick     = 0;  /* 上次收到数据的时间戳 */
static uint8_t  upload_error         = 0;  /* 0=正常, 非0=错误码 (写入失败/溢出) */

/*===== 固件数据写入 SPI Flash =====
 * @return 0=成功, 非0=错误码 (APP_STATUS_ERR_OVERFLOW / APP_STATUS_ERR_FLASH)
 * 边缘处理:
 *   1) 写入前先比对 upload_count+len 是否超过 upload_total, 防止 PC 多发
 *   2) GD25Q_WritePage 返回值检查, 失败立即停止并上报
 *   3) 残页缓冲区写满后才提交到 Flash, 避免每字节都触发页编程
 */
static uint8_t UploadData(uint8_t *data, uint16_t len)
{
    if (upload_count + (uint32_t)len > upload_total)
        return APP_STATUS_ERR_OVERFLOW;

    for (uint16_t i = 0; i < len; i++) {
        page_buf[page_idx++] = data[i];
        upload_count++;
        if (page_idx >= APP_PAGE_SIZE) {
            if (!GD25Q_WritePage(page_buf, upload_addr, APP_PAGE_SIZE))
                return APP_STATUS_ERR_FLASH;
            upload_addr += APP_PAGE_SIZE;
            page_idx = 0;
        }
    }
    return APP_STATUS_OK;
}

/*===== 公开 API =====*/

void FwUpload_Init(void)
{
    upload_slot          = 0;
    upload_count         = 0;
    upload_total         = 0;
    upload_total_frames  = 0;
    upload_data_frames   = 0;
    upload_error         = 0;
    upload_last_tick     = 0;
    page_idx             = 0;
}

uint8_t FwUpload_Handle(uint16_t cmd, uint8_t *data, uint16_t len)
{
    switch (cmd) {

    /* 起始帧 (slot0=Bootloader, slot1=App)
     * 新格式 (LEN=9):
     *   [0..1] u16 总帧数 (大端)
     *   [2..5] u32 文件大小 (大端)
     *   [6..8] 3B  版本号 [主.次.修订]
     */
    case MGR_CMD_FW_START: {
        if (app_state != APP_STATE_IDLE) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(MGR_CMD_FW_START, &e, 1); break;
        }
        if (len < 9) {
            uint8_t e = APP_STATUS_ERR_SIZE; APP_SendAck(MGR_CMD_FW_START, &e, 1); break;
        }
        uint16_t frames = ((uint16_t)data[0] << 8) | data[1];
        uint32_t bytes  = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16)
                        | ((uint32_t)data[4] << 8)  |  (uint32_t)data[5];
        if (bytes == 0 || bytes > FW_SLOT_SIZE - FW_HEAD_SIZE || frames == 0) {
            uint8_t e = APP_STATUS_ERR_SIZE; APP_SendAck(MGR_CMD_FW_START, &e, 1); break;
        }

        /* 选择存储槽位, 计算基地址 (双槽: slot0=Bootloader, slot1=App)
         * [2026-08-10] FlashMap 诊断实测全地址掉电持久, 原"slot0硬件缺陷"结论错误,
         * 实为旧上传流程 bug; 恢复双槽。 */
        uint32_t base = (upload_slot == 0) ? FW_SLOT0_BASE : FW_SLOT1_BASE;
        upload_addr         = base + FW_HEAD_SIZE;     /* 数据从头部之后开始 */
        upload_total        = bytes;
        upload_count        = 0;
        upload_total_frames = frames;
        upload_data_frames  = 0;
        upload_version[0]   = data[6];
        upload_version[1]   = data[7];
        upload_version[2]   = data[8];
        page_idx            = 0;
        upload_error        = 0;
        upload_last_tick    = HAL_GetTick();
        memset(page_buf, 0, sizeof(page_buf));   /* 清残页, 防跨会话污染 */

        /* 解除块保护 (BP 位可能保护低地址区域, 否则擦除/写入静默失败)
         * 注: SoftwareReset(0x66+0x99) 曾试用于修复首写不持久, 实测无效已移除 */
        GD25Q_Unprotect();

        /* 擦除对应扇区 (头部 + 数据 + 余量), 失败必须中止 */
        uint32_t sec_start = base / 4096;
        uint32_t sec_count = (bytes + FW_HEAD_SIZE + 4095) / 4096;
        for (uint32_t s = sec_start; s < sec_start + sec_count; s++) {
            if (!GD25Q_EraseSector(s * 4096)) {
                uint8_t e = APP_STATUS_ERR_FLASH; APP_SendAck(MGR_CMD_FW_START, &e, 1);
                return 1;   /* 擦除失败, 不继续写头部 */
            }
        }

        /* 写入头部 7B: [4B 大小 LE][3B 版本号], 跳过标志位(END时单独写) */
        uint8_t hdr[FW_HEAD_SIZE];
        hdr[0] = (uint8_t)(bytes);
        hdr[1] = (uint8_t)(bytes >> 8);
        hdr[2] = (uint8_t)(bytes >> 16);
        hdr[3] = (uint8_t)(bytes >> 24);
        hdr[4] = upload_version[0];
        hdr[5] = upload_version[1];
        hdr[6] = upload_version[2];
        if (!GD25Q_WritePage(hdr, base, FW_HEAD_SIZE - 1)) {
            uint8_t e = APP_STATUS_ERR_FLASH; APP_SendAck(MGR_CMD_FW_START, &e, 1); break;
        }

        app_state = APP_STATE_UPLOADING;
        APP_SetRGB(RGB_BLUE, 2);
        { uint8_t ok = APP_STATUS_OK; APP_SendAck(MGR_CMD_FW_START, &ok, 1); }
        break;
    }

    /* 数据帧 */
    case MGR_CMD_FW_DATA: {
        if (app_state != APP_STATE_UPLOADING) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(MGR_CMD_FW_DATA, &e, 1); break;
        }
        if (upload_error) {
            /* 已出错, 拒绝继续写入, 仅回错误码 */
            uint8_t e = upload_error; APP_SendAck(MGR_CMD_FW_DATA, &e, 1); break;
        }
        upload_last_tick = HAL_GetTick();
        upload_data_frames++;
        uint8_t r = UploadData(data, len);
        if (r != APP_STATUS_OK) {
            upload_error = r;
            APP_SetRGB(RGB_RED, 0);
        }
        { uint8_t resp = (upload_error == 0) ? APP_STATUS_OK : upload_error;
          APP_SendAck(MGR_CMD_FW_DATA, &resp, 1); }
        break;
    }

    /* 结束帧
     * 收尾: 1) 残页刷入; 2) 比对字节数; 3) 写完整标志 (断电不丢失)
     */
    case MGR_CMD_FW_END: {
        if (app_state != APP_STATE_UPLOADING) {
            uint8_t e = APP_STATUS_BUSY; APP_SendAck(MGR_CMD_FW_END, &e, 1); break;
        }

        uint8_t  resp = APP_STATUS_OK;
        uint32_t cur_slot = upload_slot;   /* 双槽: 当前上传的槽写完整标志 */

        /* 若上传中已出错, 直接回错误码, 不写完整标志 */
        if (upload_error) {
            resp = upload_error;
        } else if (page_idx > 0) {
            /* 残页刷入 */
            if (!GD25Q_WritePage(page_buf, upload_addr, page_idx)) {
                resp = APP_STATUS_ERR_FLASH;
            }
        }

        /* 比对字节数: 仅在前面步骤都成功时检查, 否则保留原错误码 */
        if (resp == APP_STATUS_OK && upload_count != upload_total) {
            resp = APP_STATUS_ERR_COUNT;
        }

        /* 写完整标志 (仅在字节数匹配时) */
        if (resp == APP_STATUS_OK) {
            if (Flash_WriteCompleteFlag((uint8_t)cur_slot, FW_FLAG_COMPLETE) != 0)
                resp = APP_STATUS_ERR_FLASH;
        }

        /* 切到下一个槽, 为第二次上传做准备 (双槽: slot0→slot1→slot0 交替) */
        upload_slot = (upload_slot == 0) ? 1 : 0;

        app_state = APP_STATE_IDLE;
        APP_SetRGB((resp == APP_STATUS_OK) ? RGB_BLUE : RGB_RED, 1);
        APP_SendAck(MGR_CMD_FW_END, &resp, 1);
        break;
    }

    default:
        return 0;  /* 非上传命令, 未处理 */
    }
    return 1;  /* 已处理 */
}

/* 上传超时: 10s 无新数据, 回 IDLE 并上报错误
 * 注意: 若 upload_error 已被设置 (溢出/写Flash失败), 不再覆盖为 TIMEOUT,
 * 避免给 PC 发两次不同错误码造成混淆 */
void FwUpload_TimeoutCheck(void)
{
    if (app_state != APP_STATE_UPLOADING) return;

    if (HAL_GetTick() - upload_last_tick > APP_UPLOAD_TIMEOUT_MS) {
        uint8_t e = upload_error ? upload_error : APP_STATUS_ERR_TIMEOUT;
        app_state = APP_STATE_IDLE;
        if (!upload_error) upload_error = APP_STATUS_ERR_TIMEOUT;
        APP_SetRGB(RGB_RED, 0);
        APP_SendAck(MGR_CMD_FW_DATA, &e, 1);
        APP_Print("[ERROR] Upload timeout, aborted\r\n");
    }
}
