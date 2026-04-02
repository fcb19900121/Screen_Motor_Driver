#include "usb_cmd.h"
#include "usbd_cdc_if.h"
#include "motor_app.h"
#include <string.h>

/* ---------- 环形接收缓冲区 ---------- */
#define RX_RING_SIZE  128
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;  /* ISR 写入位置 */
static volatile uint16_t rx_tail = 0;  /* 主循环读取位置 */

/* 当前解析行缓冲 */
static char line_buf[USB_CMD_BUF_SIZE];
static uint16_t line_pos = 0;

/* ------------------------------------------------------------------ */
void USB_Cmd_Feed(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {          /* 满了就丢弃 */
            rx_ring[rx_head] = data[i];
            rx_head = next;
        }
    }
}

/* ------------------------------------------------------------------ */
void USB_Cmd_SendStr(const char *str)
{
    CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
}

/* ---------- 内部：简单十进制解析 ---------- */
static int parse_int(const char *s, int *val)
{
    int v = 0, neg = 0, found = 0;
    if (*s == '-') { neg = 1; s++; }
    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (*s - '0');
        s++;
        found = 1;
    }
    if (!found) return 0;
    *val = neg ? -v : v;
    return 1;
}

/* ---------- 内部：处理一行命令 ---------- */
static void process_line(char *cmd, uint16_t len)
{
    /* 去掉行尾 \r\n */
    while (len > 0 && (cmd[len - 1] == '\r' || cmd[len - 1] == '\n'))
        len--;
    cmd[len] = '\0';
    if (len == 0) return;

    /* ---- PITCH:xxx ---- */
    if (len > 6 && strncmp(cmd, "PITCH:", 6) == 0) {
        int duty_pct = 0;
        if (parse_int(&cmd[6], &duty_pct)) {
            if (duty_pct < 0) duty_pct = 0;
            if (duty_pct > 100) duty_pct = 100;
            /* 0~100 映射到 0~40960 */
            int32_t target = (int32_t)duty_pct * 40960 / 100;
            if (MotorApp_PitchMoveTo(target)) {
                USB_Cmd_SendStr("OK\r\n");
            } else {
                USB_Cmd_SendStr("ERR:NOT_READY\r\n");
            }
        } else {
            USB_Cmd_SendStr("ERR:BAD_VALUE\r\n");
        }
        return;
    }

    /* ---- SCREEN:PORTR ---- */
    if (len == 12 && strncmp(cmd, "SCREEN:PORTR", 12) == 0) {
        MotorApp_ScreenRotateTo(SCREEN_PORTRAIT);
        USB_Cmd_SendStr("OK\r\n");
        return;
    }

    /* ---- SCREEN:LANDS ---- */
    if (len == 12 && strncmp(cmd, "SCREEN:LANDS", 12) == 0) {
        MotorApp_ScreenRotateTo(SCREEN_LANDSCAPE);
        USB_Cmd_SendStr("OK\r\n");
        return;
    }

    /* ---- RESET ---- */
    if (len == 5 && strncmp(cmd, "RESET", 5) == 0) {
        MotorApp_StartHoming();
        MotorApp_ScreenDetect();
        USB_Cmd_SendStr("OK\r\n");
        return;
    }

    /* 未识别命令 */
    USB_Cmd_SendStr("ERR:UNKNOWN\r\n");
}

/* ------------------------------------------------------------------ */
void USB_Cmd_Poll(void)
{
    while (rx_tail != rx_head) {
        uint8_t ch = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % RX_RING_SIZE;

        if (ch == '\n') {
            process_line(line_buf, line_pos);
            line_pos = 0;
        } else {
            if (line_pos < USB_CMD_BUF_SIZE - 1) {
                line_buf[line_pos++] = (char)ch;
            } else {
                /* 行太长，丢弃 */
                line_pos = 0;
            }
        }
    }
}
