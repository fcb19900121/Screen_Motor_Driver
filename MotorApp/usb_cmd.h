#ifndef USB_CMD_H
#define USB_CMD_H

#include <stdint.h>

#define USB_CMD_BUF_SIZE  64  /* 单条命令最大长度 */

/**
 * @brief  将 USB 收到的原始数据送入命令缓冲区（在 CDC_Receive_FS 中调用）
 * @param  data  收到的字节数组
 * @param  len   字节数
 */
void USB_Cmd_Feed(const uint8_t *data, uint32_t len);

/**
 * @brief  在主循环中轮询，解析并执行完整命令
 *         识别 \n 结尾的命令行后处理，非阻塞
 */
void USB_Cmd_Poll(void);

/**
 * @brief  通过 USB CDC 发送字符串响应
 */
void USB_Cmd_SendStr(const char *str);

#endif /* USB_CMD_H */
