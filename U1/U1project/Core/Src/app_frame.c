#include "app_frame.h"
#include "app.h"
#include "protocol_frame.h"

/*===== 帧接收缓冲 =====*/
static uint8_t  frame_buf[APP_FRAME_BUF_SIZE];
static uint16_t frame_pos    = 0;
static uint16_t frame_dlen   = 0;
static uint8_t  frame_ready  = 0;

/*===== 帧接收状态机 (ISR 上下文) =====
 * 收完一整帧后置 frame_ready=1, 由 APP_Run 解析。
 * LEN 严格按协议上限 FRAME_MAX_DATA (256) 卡死, 防止 PC 乱发巨型帧打爆缓冲区。
 */
void APP_Frame_Feed(uint8_t byte)
{
    switch (frame_pos) {
    case 0:
        if (byte == FRAME_HDR0) frame_buf[frame_pos++] = byte;
        break;
    case 1:
        if (byte == FRAME_HDR1) frame_buf[frame_pos++] = byte;
        else frame_pos = 0;
        break;
    case 2: case 3: case 4: case 5:
        frame_buf[frame_pos++] = byte; break;
    case 6:
        frame_buf[6] = byte; frame_pos++; break;
    case 7:
        frame_buf[7] = byte;
        frame_dlen = ((uint16_t)frame_buf[6] << 8) | byte;
        if (frame_dlen > FRAME_MAX_DATA) { frame_pos = 0; break; }
        frame_pos++;
        if (frame_dlen == 0) frame_pos = 8;
        break;
    default:
        if (frame_pos < 8 + frame_dlen)
            frame_buf[frame_pos++] = byte;
        else if (frame_pos == 8 + frame_dlen)
            frame_buf[frame_pos++] = byte;  /* CRC LO */
        else {
            frame_buf[frame_pos++] = byte;  /* CRC HI */
            frame_ready = 1;
        }
        break;
    }
}

/*===== 公开 API =====*/

void APP_Frame_Init(void)
{
    frame_pos   = 0;
    frame_ready = 0;
}

uint8_t APP_Frame_Pending(void)
{
    return frame_ready;
}

uint8_t *APP_Frame_Consume(uint16_t *len)
{
    if (len) *len = frame_pos;
    frame_ready = 0;
    frame_pos   = 0;
    return frame_buf;
}
