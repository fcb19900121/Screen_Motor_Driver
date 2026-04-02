/*
 * Flash-based EEPROM emulation for STM32F1xx (AN2594 style).
 *
 * Dual-page scheme with wear levelling and power-fail safety.
 */

#include "eeprom_emul.h"
#include "stm32f1xx_hal.h"

/* ---- Virtual address table (order must match NB_OF_VAR) ---- */
static const uint16_t VirtAddVarTab[NB_OF_VAR] = {
    EE_VAR_SCREEN_ORIENT,
    EE_VAR_PITCH_TARGET_LO,
    EE_VAR_PITCH_TARGET_HI,
};

/* ---- Internal helpers ---- */
static uint16_t EE_Format(void);
static uint16_t EE_FindValidPage(uint8_t Operation);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);

#define READ_FROM_VALID_PAGE   0
#define WRITE_IN_VALID_PAGE    1

/* ------------------------------------------------------------------ */
/*  EE_Init                                                           */
/* ------------------------------------------------------------------ */
uint16_t EE_Init(void)
{
    uint16_t PageStatus0, PageStatus1;
    uint16_t status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;

    HAL_FLASH_Unlock();

    PageStatus0 = *(__IO uint16_t *)EEPROM_PAGE0_BASE;
    PageStatus1 = *(__IO uint16_t *)EEPROM_PAGE1_BASE;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.NbPages   = 1;

    switch (PageStatus0) {
    /* ---- Page0: ERASED ---- */
    case ERASED:
        if (PageStatus1 == VALID_PAGE) {
            /* Page0 not formatted → erase it cleanly */
            eraseInit.PageAddress = EEPROM_PAGE0_BASE;
            if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
                HAL_FLASH_Lock();
                return EE_FLASH_ERROR;
            }
        } else if (PageStatus1 == RECEIVE_DATA) {
            /* Page1 was receiving → erase Page0, mark Page1 valid */
            eraseInit.PageAddress = EEPROM_PAGE0_BASE;
            if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
                HAL_FLASH_Lock();
                return EE_FLASH_ERROR;
            }
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  EEPROM_PAGE1_BASE, VALID_PAGE) != HAL_OK) {
                HAL_FLASH_Lock();
                return EE_FLASH_ERROR;
            }
        } else {
            /* Both erased → full format */
            status = EE_Format();
            if (status != EE_OK) {
                HAL_FLASH_Lock();
                return status;
            }
        }
        break;

    /* ---- Page0: RECEIVE_DATA (was receiving when interrupted) ---- */
    case RECEIVE_DATA:
        if (PageStatus1 == VALID_PAGE) {
            /* Transfer was in progress from Page1 → Page0. Re-transfer. */
            /* Copy latest values from Page1 to Page0 */
            for (uint16_t i = 0; i < NB_OF_VAR; i++) {
                uint16_t val;
                if (*(__IO uint16_t *)(EEPROM_PAGE0_BASE + 6) == VirtAddVarTab[i]) {
                    /* Already transferred */
                } else {
                    if (EE_ReadVariable(VirtAddVarTab[i], &val) == EE_OK) {
                        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                          EEPROM_PAGE0_BASE + 4, val);
                        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                          EEPROM_PAGE0_BASE + 6, VirtAddVarTab[i]);
                    }
                }
            }
            /* Mark Page0 valid */
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              EEPROM_PAGE0_BASE, VALID_PAGE);
            /* Erase Page1 */
            eraseInit.PageAddress = EEPROM_PAGE1_BASE;
            if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
                HAL_FLASH_Lock();
                return EE_FLASH_ERROR;
            }
        } else if (PageStatus1 == ERASED) {
            /* Erase Page1, mark Page0 valid */
            eraseInit.PageAddress = EEPROM_PAGE1_BASE;
            if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
                HAL_FLASH_Lock();
                return EE_FLASH_ERROR;
            }
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              EEPROM_PAGE0_BASE, VALID_PAGE);
        } else {
            status = EE_Format();
            if (status != EE_OK) {
                HAL_FLASH_Lock();
                return status;
            }
        }
        break;

    /* ---- Page0: VALID_PAGE ---- */
    case VALID_PAGE:
        if (PageStatus1 == VALID_PAGE) {
            /* Shouldn't happen, format */
            status = EE_Format();
            if (status != EE_OK) {
                HAL_FLASH_Lock();
                return status;
            }
        } else if (PageStatus1 == ERASED) {
            /* Normal case */
            /* Verify Page1 is clean */
        } else {
            /* Page1 was receiving → erase it */
            eraseInit.PageAddress = EEPROM_PAGE1_BASE;
            if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
                HAL_FLASH_Lock();
                return EE_FLASH_ERROR;
            }
        }
        break;

    default:
        status = EE_Format();
        if (status != EE_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        break;
    }

    HAL_FLASH_Lock();
    return EE_OK;
}

/* ------------------------------------------------------------------ */
/*  EE_ReadVariable                                                   */
/* ------------------------------------------------------------------ */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t *Data)
{
    uint16_t validPage;
    uint32_t pageBase, pageEnd, addr;

    validPage = EE_FindValidPage(READ_FROM_VALID_PAGE);
    if (validPage == EE_NO_VALID_PAGE)
        return EE_NO_VALID_PAGE;

    pageBase = (validPage == 0) ? EEPROM_PAGE0_BASE : EEPROM_PAGE1_BASE;
    pageEnd  = pageBase + PAGE_SIZE - 2;

    /* Scan from end to start to find latest value (skip page header at +0) */
    for (addr = pageEnd; addr >= pageBase + 4; addr -= 4) {
        if (*(__IO uint16_t *)addr == VirtAddress) {
            *Data = *(__IO uint16_t *)(addr - 2);
            return EE_OK;
        }
    }
    return EE_VAR_NOT_FOUND;
}

/* ------------------------------------------------------------------ */
/*  EE_WriteVariable                                                  */
/* ------------------------------------------------------------------ */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data)
{
    uint16_t status;

    HAL_FLASH_Unlock();
    status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
    if (status == EE_PAGE_FULL) {
        status = EE_PageTransfer(VirtAddress, Data);
    }
    HAL_FLASH_Lock();
    return status;
}

/* ================================================================== */
/*  Internal functions                                                */
/* ================================================================== */

/* Format both pages: erase both, mark Page0 as VALID */
static uint16_t EE_Format(void)
{
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.NbPages   = 1;

    eraseInit.PageAddress = EEPROM_PAGE0_BASE;
    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
        return EE_FLASH_ERROR;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          EEPROM_PAGE0_BASE, VALID_PAGE) != HAL_OK)
        return EE_FLASH_ERROR;

    eraseInit.PageAddress = EEPROM_PAGE1_BASE;
    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
        return EE_FLASH_ERROR;

    return EE_OK;
}

/* Find which page (0 or 1) is currently valid for the requested operation */
static uint16_t EE_FindValidPage(uint8_t Operation)
{
    uint16_t s0 = *(__IO uint16_t *)EEPROM_PAGE0_BASE;
    uint16_t s1 = *(__IO uint16_t *)EEPROM_PAGE1_BASE;

    switch (Operation) {
    case WRITE_IN_VALID_PAGE:
        if (s0 == VALID_PAGE)       return 0;
        else if (s1 == VALID_PAGE)  return 1;
        else                        return EE_NO_VALID_PAGE;

    case READ_FROM_VALID_PAGE:
        if (s0 == VALID_PAGE)       return 0;
        else if (s1 == VALID_PAGE)  return 1;
        else                        return EE_NO_VALID_PAGE;

    default:
        return EE_NO_VALID_PAGE;
    }
}

/* Write variable to valid page; return EE_PAGE_FULL if page exhausted */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data)
{
    uint16_t validPage;
    uint32_t pageBase, addr, pageEnd;

    validPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);
    if (validPage == EE_NO_VALID_PAGE)
        return EE_NO_VALID_PAGE;

    pageBase = (validPage == 0) ? EEPROM_PAGE0_BASE : EEPROM_PAGE1_BASE;
    pageEnd  = pageBase + PAGE_SIZE;

    /* Find first free 32-bit slot (both halfwords == 0xFFFF) */
    for (addr = pageBase + 4; addr < pageEnd; addr += 4) {
        if (*(__IO uint32_t *)addr == 0xFFFFFFFF) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, Data) != HAL_OK)
                return EE_FLASH_ERROR;
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2, VirtAddress) != HAL_OK)
                return EE_FLASH_ERROR;
            return EE_OK;
        }
    }
    return EE_PAGE_FULL;
}

/* Transfer latest values to new page when current page is full */
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
    uint16_t validPage;
    uint32_t newPageBase, oldPageBase;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError;
    uint16_t status;

    validPage = EE_FindValidPage(READ_FROM_VALID_PAGE);
    if (validPage == EE_NO_VALID_PAGE)
        return EE_NO_VALID_PAGE;

    if (validPage == 0) {
        newPageBase = EEPROM_PAGE1_BASE;
        oldPageBase = EEPROM_PAGE0_BASE;
    } else {
        newPageBase = EEPROM_PAGE0_BASE;
        oldPageBase = EEPROM_PAGE1_BASE;
    }

    /* Mark new page as RECEIVE_DATA */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          newPageBase, RECEIVE_DATA) != HAL_OK)
        return EE_FLASH_ERROR;

    /* Write the new variable first */
    status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
    if (status != EE_OK)
        return status;

    /* Transfer other variables' latest values */
    for (uint16_t i = 0; i < NB_OF_VAR; i++) {
        if (VirtAddVarTab[i] != VirtAddress) {
            uint16_t val;
            uint16_t readSt = EE_ReadVariable(VirtAddVarTab[i], &val);
            if (readSt == EE_OK) {
                /* EE_ReadVariable now reads from old valid page.
                   EE_VerifyPageFullWriteVariable writes to new page
                   (which is marked RECEIVE_DATA, not VALID yet).
                   We need to write directly to new page. */
                /* Find free slot in new page */
                uint32_t addr;
                for (addr = newPageBase + 4; addr < newPageBase + PAGE_SIZE; addr += 4) {
                    if (*(__IO uint32_t *)addr == 0xFFFFFFFF) {
                        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, val) != HAL_OK)
                            return EE_FLASH_ERROR;
                        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2, VirtAddVarTab[i]) != HAL_OK)
                            return EE_FLASH_ERROR;
                        break;
                    }
                }
            }
        }
    }

    /* Erase old page */
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.NbPages     = 1;
    eraseInit.PageAddress = oldPageBase;
    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
        return EE_FLASH_ERROR;

    /* Mark new page as VALID */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          newPageBase, VALID_PAGE) != HAL_OK)
        return EE_FLASH_ERROR;

    return EE_OK;
}
