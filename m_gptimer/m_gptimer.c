/*
 * m_gptimer.c
 *
 * 基于 GPTimer 的 bit-bang 串口驱动，适用于 ESP32-P4。
 * 8N1 协议，auto-reload alarm 逐 bit 输出，GPIO 中断检测起始位。
 */

#include "m_gptimer.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

/* ─────────────────────── 默认配置（按实际硬件修改）─────────────────── */
#define M_GPTIMER_BAUD_RATE  9600
#define M_GPTIMER_TX_PIN     GPIO_NUM_22
#define M_GPTIMER_RX_PIN     GPIO_NUM_23

static const char *TAG = "m_gptimer";

/* ───────────────────────── TX 状态机 ───────────────────────────────── */

/* TX 状态枚举 */
typedef enum {
    TX_IDLE,        /* 空闲，没有在发送 */
    TX_START_BIT,   /* 正在输出起始位 */
    TX_DATA_BITS,   /* 正在输出数据位 + 停止位 */
} tx_state_t;

/* TX 上下文 */
static struct {
    tx_state_t    state;        /* 当前状态 */
    uint8_t       bit_idx;      /* 当前输出到帧内第几个 bit（0~9） */
    uint16_t      shift_reg;    /* 10-bit 移位寄存器: [stop|D7..D0|start] */
    const uint8_t *buf;         /* 指向用户数据的下一个待发字节 */
    size_t        remaining;    /* 当前帧之后剩余待发字节数 */
} s_tx;

/* ───────────────────────── RX 状态机 ───────────────────────────────── */

/* RX 状态枚举 */
typedef enum {
    RX_IDLE,        /* 空闲，等待起始位 */
    RX_DATA_BITS,   /* 正在采样数据位 */
} rx_state_t;

/* RX 上下文 */
static struct {
    rx_state_t state;           /* 当前状态 */
    uint8_t    bit_idx;         /* 当前采到第几个数据位（0~7） */
    uint8_t    shift_reg;       /* 采到的 bit 逐位移入，LSB 先入 */
    uint8_t   *buf;             /* 指向用户缓冲区的下一个槽位 */
    size_t     remaining;       /* 剩余待收字节数 */
    bool       frame_error;     /* stop bit 不为高时置 true */
} s_rx;

/* ───────────────────────── 全局句柄 ─────────────────────────────────── */

static gptimer_handle_t   s_timer  = NULL;      /* GPTimer 句柄 */
static TaskHandle_t       s_task   = NULL;      /* 调用者的任务句柄，用于 task notification */
static SemaphoreHandle_t  s_mutex  = NULL;      /* send/receive 互斥锁 */
static gpio_num_t         s_tx_pin = GPIO_NUM_NC;
static gpio_num_t         s_rx_pin = GPIO_NUM_NC;

/* ─────────────────────── TX 辅助函数 ───────────────────────────────── */

/**
 * @brief 把一个字节打包成 10-bit 帧，写入移位寄存器并启动 timer
 *
 * 移位寄存器布局（从低位开始逐 bit 移出）:
 *   bit[0] = start (0)
 *   bit[1] = D0
 *   ...
 *   bit[8] = D7
 *   bit[9] = stop  (1)
 */
static void tx_load_byte_and_start(uint8_t byte)
{
    /* 组装帧: stop(1) | data | start(0) */
    s_tx.shift_reg = (1 << 9) | ((uint16_t)byte << 1) | 0;
    s_tx.bit_idx   = 0;
    s_tx.state     = TX_START_BIT;

    /* 配置 auto-reload alarm，每 1 tick（1 bit 周期）触发一次 */
    gptimer_alarm_config_t alarm = {
        .alarm_count                = 1,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(s_timer, &alarm);
    gptimer_start(s_timer);
}

/* ─────────────────────── RX 辅助函数 ───────────────────────────────── */

/**
 * @brief 使能 GPIO 中断，等待下一个起始位（下降沿）
 */
static void rx_arm_isr(void)
{
    s_rx.state = RX_IDLE;
    gpio_intr_enable(s_rx_pin);
}

/* ─────────────────────── GPTimer alarm 回调（ISR 上下文）────────────── */

/* 调试用: 记录 alarm 回调触发次数 */
static volatile uint32_t s_alarm_count = 0;

/**
 * @brief 每个 bit 周期触发一次，负责输出/采样一个 bit
 *
 * TX 和 RX 不会同时激活（mutex 保护），所以两个分支不会冲突。
 */
static bool IRAM_ATTR alarm_cb(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_data)
{
    BaseType_t yield = pdFALSE;
    s_alarm_count++;

    /* ── TX 路径 ── */
    if (s_tx.state != TX_IDLE) {
        /* 输出移位寄存器的最低位 */
        gpio_set_level(s_tx_pin, s_tx.shift_reg & 1);
        s_tx.shift_reg >>= 1;
        s_tx.bit_idx++;

        if (s_tx.bit_idx >= 10) {
            /* 一帧（start + 8 data + stop）全部输出完毕 */
            if (s_tx.remaining > 0) {
                /* 还有字节: 从 buf 取下一个，重新组装帧 */
                uint8_t next = *s_tx.buf++;
                s_tx.remaining--;
                s_tx.shift_reg = (1 << 9) | ((uint16_t)next << 1) | 0;
                s_tx.bit_idx   = 0;
                s_tx.state     = TX_START_BIT;
            } else {
                /* 全部发完: 停 timer，通知调用者 */
                s_tx.state = TX_IDLE;
                gptimer_stop(s_timer);
                if (s_task) {
                    xTaskNotifyFromISR(s_task, 0, eNoAction, &yield);
                }
            }
        }
    }

    /* ── RX 路径 ── */
    if (s_rx.state != RX_IDLE) {
        uint8_t sample = gpio_get_level(s_rx_pin);

        if (s_rx.bit_idx < 8) {
            /* 数据位: LSB 先入，逐位移入移位寄存器 */
            s_rx.shift_reg |= (sample << s_rx.bit_idx);
            s_rx.bit_idx++;
        } else {
            /* 停止位: 必须为高电平 */
            if (sample != 1) {
                s_rx.frame_error = true;
            }
            /* 存入用户缓冲区 */
            *s_rx.buf++ = s_rx.shift_reg;
            s_rx.remaining--;

            if (s_rx.remaining > 0) {
                /* 还有字节: 重新使能 GPIO 中断，等待下一个起始位 */
                s_rx.state     = RX_IDLE;
                s_rx.bit_idx   = 0;
                s_rx.shift_reg = 0;
                gptimer_stop(s_timer);
                gpio_intr_enable(s_rx_pin);
            } else {
                /* 全部收完: 停 timer，通知调用者 */
                s_rx.state = RX_IDLE;
                gptimer_stop(s_timer);
                if (s_task) {
                    xTaskNotifyFromISR(s_task, 0, eNoAction, &yield);
                }
            }
        }
    }

    return (yield == pdTRUE);
}

/* ─────────────────────── GPIO ISR: 起始位检测 ──────────────────────── */

/**
 * @brief GPIO 下降沿中断，检测到起始位后启动 timer 开始采样
 *
 * 从下降沿到 ISR 执行约有 3~5us 延迟。
 * alarm_count = 1 → timer 运行 1 个 bit 周期后首次触发。
 * 总延迟 ≈ ISR 延迟 + 1 bit ≈ 1.5 bit，接近 D0 中心。
 */
/* 调试用: 记录 RX 起始位中断触发次数 */
static volatile uint32_t s_rx_isr_count = 0;

static void IRAM_ATTR rx_start_bit_isr(void *arg)
{
    s_rx_isr_count++;

    /* 如果 RX 不在等待状态，忽略（防止虚假中断） */
    if (s_rx.state != RX_IDLE || s_rx.remaining == 0) {
        return;
    }

    /* 先关中断，防止这个字节还没收完又被触发 */
    gpio_intr_disable(s_rx_pin);

    /* 准备 RX 状态，开始接收 8 个数据位 */
    s_rx.state     = RX_DATA_BITS;
    s_rx.bit_idx   = 0;
    s_rx.shift_reg = 0;

    /* 启动 timer: 计数归零，alarm 设为 1 个 bit 周期 */
    gptimer_set_raw_count(s_timer, 0);
    gptimer_alarm_config_t alarm = {
        .alarm_count                = 1,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(s_timer, &alarm);
    gptimer_start(s_timer);
}

/* ═══════════════════════════ 公共 API ═════════════════════════════════ */

esp_err_t m_gptimer_init(uint32_t baud_rate, gpio_num_t tx_pin, gpio_num_t rx_pin)
{
    /* 防止重复初始化 */
    if (s_timer != NULL) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    s_task   = xTaskGetCurrentTaskHandle();

    /* 创建 timer: resolution = 波特率，1 tick = 1 个 bit 周期 */
    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = baud_rate,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &s_timer));

    /* 注册 alarm 回调 */
    gptimer_event_callbacks_t cbs = {
        .on_alarm = alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));

    /* TX 引脚: 输出模式，默认高电平（空闲态） */
    gpio_config_t tx_conf = {
        .pin_bit_mask = 1ULL << tx_pin,
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&tx_conf));
    ESP_ERROR_CHECK(gpio_set_level(tx_pin, 1));

    /* RX 引脚: 输入模式，内部上拉，中断先关闭（receive 时再开） */
    gpio_config_t rx_conf = {
        .pin_bit_mask = 1ULL << rx_pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,  /* 下降沿触发（start bit） */
    };
    ESP_ERROR_CHECK(gpio_config(&rx_conf));
    /* 配置时打开中断类型，但马上关掉，等 receive 时再开 */
    gpio_intr_disable(rx_pin);

    /* 安装 GPIO ISR 服务，注册起始位检测回调 */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(rx_pin, rx_start_bit_isr, NULL));

    /* 创建互斥锁，保护 send/receive 不会同时操作 timer */
    s_mutex = xSemaphoreCreateMutex();

    /* 清零所有状态 */
    memset(&s_tx, 0, sizeof(s_tx));
    memset(&s_rx, 0, sizeof(s_rx));

    ESP_LOGI(TAG, "Initialized: %lu baud, TX=GPIO%d, RX=GPIO%d",
             (unsigned long)baud_rate, tx_pin, rx_pin);
    return ESP_OK;
}

esp_err_t m_gptimer_send(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (s_timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len == 0) {
        return ESP_OK;
    }

    /* 获取 mutex，防止和 receive 同时操作 timer */
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 设置 TX 上下文 */
    s_task         = xTaskGetCurrentTaskHandle();
    s_tx.buf       = data + 1;      /* 指向第二个字节（第一个马上发） */
    s_tx.remaining = len - 1;

    /* 加载第一个字节并启动 timer */
    tx_load_byte_and_start(data[0]);

    /* 阻塞等待全部发完或超时 */
    esp_err_t ret = ESP_OK;
    if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Send timeout");
        gptimer_stop(s_timer);
        s_tx.state = TX_IDLE;
        ret = ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t m_gptimer_receive(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (s_timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len == 0) {
        return ESP_OK;
    }

    /* 获取 mutex，防止和 send 同时操作 timer */
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* 设置 RX 上下文 */
    s_task          = xTaskGetCurrentTaskHandle();
    s_rx.buf        = buf;
    s_rx.remaining  = len;
    s_rx.frame_error = false;
    s_rx.bit_idx    = 0;
    s_rx.shift_reg  = 0;

    /* 使能 GPIO 中断，开始等待起始位 */
    rx_arm_isr();

    /* 阻塞等待全部收完或超时 */
    esp_err_t ret = ESP_OK;
    if (xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Receive timeout");
        gpio_intr_disable(s_rx_pin);
        gptimer_stop(s_timer);
        s_rx.state = RX_IDLE;
        ret = ESP_ERR_TIMEOUT;
    } else if (s_rx.frame_error) {
        ESP_LOGW(TAG, "Frame error (bad stop bit)");
        ret = ESP_ERR_INVALID_RESPONSE;
    }

    xSemaphoreGive(s_mutex);
    return ret;
}

esp_err_t m_gptimer_deinit(void)
{
    if (s_timer == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 先关 GPIO 中断，再停 timer */
    gpio_intr_disable(s_rx_pin);
    gpio_isr_handler_remove(s_rx_pin);

    gptimer_stop(s_timer);
    gptimer_disable(s_timer);
    gptimer_del_timer(s_timer);
    s_timer = NULL;

    if (s_mutex) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }

    ESP_LOGI(TAG, "Deinitialized");
    return ESP_OK;
}

/* ═══════════════════════════ 测试函数 ═════════════════════════════════ */

/**
 * @brief GPTimer 模拟串口测试（通过外接 USB 转串口）
 *
 * 硬件连接:
 *   ESP32-P4 GPIO23 (TX) → USB 转串口 RXD → COM4
 *   ESP32-P4 GPIO22 (RX) ← USB 转串口 TXD ← COM4
 *
 * 测试步骤:
 *   1. ESP32 发送数据 → 在 COM4 串口工具上能看到收到的数据
 *   2. 在 COM4 串口工具上发送数据 → ESP32 接收并打印
 *   3. 逐字节比对收发是否一致
 */
void m_gptimer_test(void)
{
    ESP_LOGI(TAG, "=== GPTimer bit-bang UART test ===");
    ESP_LOGI(TAG, "TX=GPIO%d -> COM4 RXD", M_GPTIMER_TX_PIN);
    ESP_LOGI(TAG, "RX=GPIO%d <- COM4 TXD", M_GPTIMER_RX_PIN);
    ESP_LOGI(TAG, "baud rate=%d", M_GPTIMER_BAUD_RATE);

    esp_err_t ret = m_gptimer_init(M_GPTIMER_BAUD_RATE, M_GPTIMER_TX_PIN, M_GPTIMER_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %d", ret);
        return;
    }

/* ================================================================
 * test1: single byte 0x55
 * TX: ESP32 sends 0x55 -> check COM4
 * RX: send 0x55 from COM4 -> check COM9
 * ================================================================ */
#if 1
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== test1: single byte 0x55 ===");

        uint8_t tx1 = 0x55;
        uint8_t rx1 = 0;

        /* TX */
        s_alarm_count = 0;
        int64_t t_start = esp_timer_get_time();
        ret = m_gptimer_send(&tx1, 1, 2000);
        int64_t t_end = esp_timer_get_time();
        ESP_LOGI(TAG, "TX: %s, alarm=%lu, time=%lld us",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_alarm_count, (long long)(t_end - t_start));

        /* RX: send 0x55 from COM4 */
        ESP_LOGI(TAG, "RX: waiting for 0x55 from COM4 (timeout 10s)...");
        s_rx_isr_count = 0;
        s_alarm_count = 0;
        ret = m_gptimer_receive(&rx1, 1, 10000);
        ESP_LOGI(TAG, "RX: %s, isr=%lu, alarm=%lu, got=0x%02X",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_rx_isr_count, (unsigned long)s_alarm_count, rx1);
    }
#endif

/* ================================================================
 * test2: multi-byte 6 bytes
 * TX: ESP32 sends 55 AA 0F F0 00 FF -> check COM4
 * RX: send 55 AA 0F F0 00 FF from COM4 -> check COM9
 * ================================================================ */
#if 1
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== test2: multi-byte 6 bytes ===");

        uint8_t tx6[] = {0x55, 0xAA, 0x0F, 0xF0, 0x00, 0xFF};
        uint8_t rx6[6] = {0};
        int len = sizeof(tx6);

        for (int i = 0; i < len; i++) {
            ESP_LOGI(TAG, "  TX[%d] = 0x%02X", i, tx6[i]);
        }

        /* TX */
        s_alarm_count = 0;
        int64_t t_start = esp_timer_get_time();
        ret = m_gptimer_send(tx6, len, 5000);
        int64_t t_end = esp_timer_get_time();
        ESP_LOGI(TAG, "TX: %s, alarm=%lu, time=%lld us",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_alarm_count, (long long)(t_end - t_start));
        ESP_LOGI(TAG, "check COM4 for: 55 AA 0F F0 00 FF");

        /* RX: send hex 55 AA 0F F0 00 FF from COM4 */
        ESP_LOGI(TAG, "RX: waiting for 6 bytes from COM4 (timeout 15s)...");
        ESP_LOGI(TAG, "send hex: 55 AA 0F F0 00 FF");
        s_rx_isr_count = 0;
        s_alarm_count = 0;
        ret = m_gptimer_receive(rx6, len, 15000);
        ESP_LOGI(TAG, "RX: %s, isr=%lu, alarm=%lu",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_rx_isr_count, (unsigned long)s_alarm_count);

        if (ret == ESP_OK) {
            bool match = true;
            for (int i = 0; i < len; i++) {
                bool ok = (rx6[i] == tx6[i]);
                if (!ok) match = false;
                ESP_LOGI(TAG, "  RX[%d] = 0x%02X (expect 0x%02X) %s",
                         i, rx6[i], tx6[i], ok ? "OK" : "MISMATCH");
            }
            ESP_LOGI(TAG, "result: %s", match ? "ALL PASS" : "HAS ERROR");
        } else if (ret == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "frame error");
        } else {
            ESP_LOGW(TAG, "timeout, isr=%lu", (unsigned long)s_rx_isr_count);
        }
    }
#endif

/* ================================================================
 * test3: interval send (1 byte every 100ms x 10)
 * TX: ESP32 sends 00~09 with 100ms delay between each -> check COM4
 * RX: send 00 01 02 03 04 05 06 07 08 09 from COM4 -> check COM9
 * ================================================================ */
#if 1
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== test3: interval send 10 bytes (100ms apart) ===");

        uint8_t tx10[10];
        uint8_t rx10[10] = {0};
        for (int i = 0; i < 10; i++) {
            tx10[i] = i;
        }

        /* TX: 1 byte every 100ms */
        int tx_ok = 0;
        for (int i = 0; i < 10; i++) {
            ret = m_gptimer_send(&tx10[i], 1, 2000);
            ESP_LOGI(TAG, "TX[%d] = 0x%02X %s", i, tx10[i],
                     (ret == ESP_OK) ? "OK" : "FAIL");
            if (ret == ESP_OK) tx_ok++;
            if (i < 9) vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "TX done: %d/10 OK", tx_ok);

        /* RX: send hex 00 01 02 03 04 05 06 07 08 09 from COM4 */
        ESP_LOGI(TAG, "RX: waiting for 10 bytes from COM4 (timeout 30s)...");
        ESP_LOGI(TAG, "send hex: 00 01 02 03 04 05 06 07 08 09");
        s_rx_isr_count = 0;
        s_alarm_count = 0;
        ret = m_gptimer_receive(rx10, 10, 30000);
        ESP_LOGI(TAG, "RX: %s, isr=%lu, alarm=%lu",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_rx_isr_count, (unsigned long)s_alarm_count);

        if (ret == ESP_OK) {
            bool match = true;
            for (int i = 0; i < 10; i++) {
                bool ok = (rx10[i] == tx10[i]);
                if (!ok) match = false;
                ESP_LOGI(TAG, "  RX[%d] = 0x%02X (expect 0x%02X) %s",
                         i, rx10[i], tx10[i], ok ? "OK" : "MISMATCH");
            }
            ESP_LOGI(TAG, "result: %s", match ? "ALL PASS" : "HAS ERROR");
        } else if (ret == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "frame error");
        } else {
            ESP_LOGW(TAG, "timeout, isr=%lu", (unsigned long)s_rx_isr_count);
        }
    }
#endif

/* ================================================================
 * test4: large packet 100 bytes
 * TX: ESP32 sends 100 bytes (0x00~0x63) -> check COM4
 * RX: send 100 bytes (0x00~0x63) from COM4 -> check COM9
 * NOTE: COM4 tool must send all 100 bytes at once (hex)
 * ================================================================ */
#if 1
    {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== test4: large packet 100 bytes ===");

        #define LARGE_LEN 100
        uint8_t tx100[LARGE_LEN];
        uint8_t rx100[LARGE_LEN];
        for (int i = 0; i < LARGE_LEN; i++) {
            tx100[i] = (uint8_t)i;  /* 0x00, 0x01, ..., 0x63 */
        }
        memset(rx100, 0, LARGE_LEN);

        /* TX: send 100 bytes */
        s_alarm_count = 0;
        int64_t t_start = esp_timer_get_time();
        ret = m_gptimer_send(tx100, LARGE_LEN, 10000);
        int64_t t_end = esp_timer_get_time();
        ESP_LOGI(TAG, "TX: %s, alarm=%lu, time=%lld us",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_alarm_count, (long long)(t_end - t_start));
        ESP_LOGI(TAG, "check COM4 for 100 bytes: 00 01 02 ... 63");

        /* RX: send 100 bytes from COM4 (hex: 00 01 02 ... 63) */
        ESP_LOGI(TAG, "RX: waiting for 100 bytes from COM4 (timeout 30s)...");
        ESP_LOGI(TAG, "send hex: 00 01 02 03 ... 61 62 63");
        s_rx_isr_count = 0;
        s_alarm_count = 0;
        t_start = esp_timer_get_time();
        ret = m_gptimer_receive(rx100, LARGE_LEN, 30000);
        t_end = esp_timer_get_time();
        ESP_LOGI(TAG, "RX: %s, isr=%lu, alarm=%lu, time=%lld us",
                 (ret == ESP_OK) ? "OK" : "FAIL",
                 (unsigned long)s_rx_isr_count, (unsigned long)s_alarm_count,
                 (long long)(t_end - t_start));

        if (ret == ESP_OK) {
            int err_count = 0;
            for (int i = 0; i < LARGE_LEN; i++) {
                if (rx100[i] != tx100[i]) {
                    ESP_LOGW(TAG, "  RX[%d] = 0x%02X (expect 0x%02X) MISMATCH",
                             i, rx100[i], tx100[i]);
                    err_count++;
                }
            }
            ESP_LOGI(TAG, "result: %s (%d/%d matched)",
                     (err_count == 0) ? "ALL PASS" : "HAS ERROR",
                     LARGE_LEN - err_count, LARGE_LEN);
        } else if (ret == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "frame error");
        } else {
            ESP_LOGW(TAG, "timeout, isr=%lu", (unsigned long)s_rx_isr_count);
        }
    }
#endif

    /* cleanup */
    m_gptimer_deinit();
    ESP_LOGI(TAG, "=== test done ===");
}
