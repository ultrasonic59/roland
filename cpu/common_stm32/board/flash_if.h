
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/// =========== STM32F205RB 128 KB flash =================================
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
void _FLASH_If_Init(void);
uint32_t _FLASH_If_Erase(uint32_t StartSector);
uint32_t _FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data, uint32_t DataLength);
uint16_t _FLASH_If_GetWriteProtectionStatus(void);
uint32_t _FLASH_If_DisableWriteProtection(void);

#endif  

