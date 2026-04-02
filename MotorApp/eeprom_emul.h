#ifndef EEPROM_EMUL_H
#define EEPROM_EMUL_H

#include <stdint.h>

/*
 * Flash-based EEPROM emulation for STM32F103RC (AN2594 style).
 *
 * Uses last 2 pages of 256KB flash (each 2KB).
 * Page0: 0x0803F000  Page1: 0x0803F800
 *
 * Each virtual variable = 32-bit record: [16-bit VirtAddr | 16-bit Data]
 * Supports up to NB_OF_VAR virtual 16-bit variables.
 */

/* ---------- Flash geometry for STM32F103RC (high-density) ---------- */
#define PAGE_SIZE               ((uint32_t)0x0800)  /* 2 KB */
#define EEPROM_PAGE0_BASE       ((uint32_t)0x0803F000)
#define EEPROM_PAGE1_BASE       ((uint32_t)0x0803F800)

/* ---------- Page status markers (written at page base address) ---------- */
#define ERASED                  ((uint16_t)0xFFFF)
#define RECEIVE_DATA            ((uint16_t)0xEEEE)
#define VALID_PAGE              ((uint16_t)0x0000)

/* ---------- Virtual variable definitions ---------- */
/* Add new variables here; virtual addresses must be non-zero */
#define EE_VAR_SCREEN_ORIENT    ((uint16_t)0x0001)  /* 屏幕方向 */
#define EE_VAR_PITCH_TARGET_LO  ((uint16_t)0x0002)  /* 俯仰目标位置 低16位 */
#define EE_VAR_PITCH_TARGET_HI  ((uint16_t)0x0003)  /* 俯仰目标位置 高16位 */

#define NB_OF_VAR               ((uint16_t)3)        /* 虚拟变量总数 */

/* ---------- Return codes ---------- */
#define EE_OK                   ((uint16_t)0)
#define EE_PAGE_FULL            ((uint16_t)1)
#define EE_NO_VALID_PAGE        ((uint16_t)2)
#define EE_VAR_NOT_FOUND        ((uint16_t)3)
#define EE_FLASH_ERROR          ((uint16_t)4)

/* ---------- API ---------- */

/**
 * @brief  Initialize EEPROM emulation. Must be called once at startup.
 * @retval EE_OK on success.
 */
uint16_t EE_Init(void);

/**
 * @brief  Read a 16-bit virtual variable.
 * @param  VirtAddress  Virtual address (non-zero, must be in variable table)
 * @param  Data         Pointer to receive value
 * @retval EE_OK on success, EE_VAR_NOT_FOUND if never written.
 */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t *Data);

/**
 * @brief  Write a 16-bit virtual variable.
 * @param  VirtAddress  Virtual address
 * @param  Data         16-bit value to store
 * @retval EE_OK on success.
 */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);

#endif /* EEPROM_EMUL_H */
