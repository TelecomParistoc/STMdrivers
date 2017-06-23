#ifndef FLASH_H
#define FLASH_H

/******************************************************************************/
/*                             Constants                                      */
/******************************************************************************/
#define FLASH_CONFIG_BASE_ADDRESS 0x40022000

/******************************************************************************/
/*                               Types                                        */
/******************************************************************************/
typedef flash_drv_ptr_t flash_drv_t*;

/******************************************************************************/
/*                             Public prototypes                              */
/******************************************************************************/

/**
 * @brief Init the flash driver.
 *
 * @param[in] base_address The address of the flash config registers or NULL to
 *                         use the default address.
 *
 * @return A pointer to the flash driver (for subsequent use).
 */
extern flash_drv_ptr_t initFlash (uint32_t* base_address);

extern void unlockFlash (void);
extern void lockFlash(void);

extern bool_t writeFlash(uint32_t* addr, uint16_t value[]);
extern bool_t eraseFlashPage(uint32_t* addr);

#endif /* FLASH_H */
