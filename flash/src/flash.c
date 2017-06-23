#include "flash.h"

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

typedef struct {
    volatile uint32_t FLASH_ACR;
    volatile uint32_t FLASH_KEYR;
    volatile uint32_t FLASH_OPTKEYR;
    volatile uint32_t FLASH_SR;
    volatile uitn32_t FLASH_CR;
    volatile uint32_t FLASH_AR;
    volatile uint32_t reserved;
    volatile uint32_t FLASH_OBR;
    volatile uint32_t FLASH_WRPR;
} flash_regs_t;

typedef struct {
    flash_regs_t regs;
    bool_t lock;
} flash_drv_t;

flash_drv_ptr_t initFlash(uint32_t* base_address) {
    flash_drv_ptr_t flash_drv_ptr;
    if (NULL == base_address) {
        flash_drv_ptr = (flash_drv_ptr_t)FLASH_CONFIG_BASE_ADDRESS;
    } else {
        flash_drv_ptr = (flash_drv_ptr_t)base_address;
    }

    return flash_drv_ptr;
}
