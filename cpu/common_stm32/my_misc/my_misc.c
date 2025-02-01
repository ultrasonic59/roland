#include <stdint.h>
#include "board.h"
#include "my_misc.h"

#include "can_cmds.h"
#include "emul_eeprom.h"
#include "printk.h"
static uint32_t GetSector(uint32_t Address);

///=======================================================================
void uDelay (const uint32_t usec)
{
uint32_t count = 0;
const uint32_t utime = 45*usec;////(120 * usec / 7);
do
  {
   if ( ++count > utime )
    {
    return ;
    }
  }
while (1);
}
///=======================================================================

pFunction Jump_To_Application;
uint32_t jumpAddress;

void goto_booter(void)
{
__disable_irq();
NVIC_SetVectorTable(NVIC_VectTab_FLASH, BOOT_BASE_ADDRESS);
  
jumpAddress = *(__IO uint32_t*) (BOOT_BASE_ADDRESS + 4);
////Jump_To_Application = (pFunction)JumpAddress;
////printk(" Jamp addres=[%x:%x]\r\n",jumpAddress,BOOT_BASE_ADDRESS);	
   Jump_To_Application = (pFunction) jumpAddress;
/* Initialize user application's Stack Pointer */
__set_CONTROL(0) ;
__set_MSP(*(__IO uint32_t*) BOOT_BASE_ADDRESS);
Jump_To_Application();
  
}
void goto_app(void)
{
////uint32_t sp_tst=0;  
__disable_irq();
NVIC_SetVectorTable(NVIC_VectTab_FLASH, APP_BASE_ADDRESS-NVIC_VectTab_FLASH);
////NVIC_SetVectorTable(APP_BASE_ADDRESS,0);
jumpAddress = *(__IO uint32_t*) (APP_BASE_ADDRESS + 4);
////sp_tst= *(__IO uint32_t*) (APP_BASE_ADDRESS);
///printk(" Jamp addres=[%x:%x:%x]\r\n",jumpAddress,APP_BASE_ADDRESS,sp_tst);	
///printk(" Jamp addres=[%x:%x:%x]\r\n",jumpAddress,APP_BASE_ADDRESS,sp_tst);	
///printk(" Jamp addres=[%x:%x:%x]\r\n",jumpAddress,APP_BASE_ADDRESS,sp_tst);	
////printk(" Jamp addres=[%x:%x:%x]\r\n",jumpAddress,APP_BASE_ADDRESS,sp_tst);	
Jump_To_Application = (pFunction)jumpAddress;
/* Initialize user application's Stack Pointer */
__set_CONTROL(0) ;
__set_MSP(*(__IO uint32_t*) APP_BASE_ADDRESS);
Jump_To_Application();
  
}
////=================================================
#if 1
void FLASH_If_Init(void)
{ 
FLASH_Unlock(); 

  /* Clear pending flags (if any) */  
FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
}

uint32_t FLASH_If_Erase(uint32_t StartSector)
{
uint32_t rez;  
FLASH_Unlock(); 
///printk("\n\r FLASH_If_Erase[%x] =>",StartSector); 

uint32_t UserStartSector = GetSector(APP_BASE_ADDRESS);

if (_FLASH_EraseSector(UserStartSector, VoltageRange_3) != FLASH_COMPLETE)
    {
    rez= ERROR_ERRASE;
    }
else
  rez= ERROR_OK;
FLASH_Lock(); 

return rez;
}
uint32_t FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data ,uint32_t DataLength)
{
  uint32_t i = 0;
////  printk("\n\r FLASH_If_Write[%x:%x] =>",FlashAddress,DataLength); 

  for (i = 0; (i < DataLength) && (*FlashAddress <= (APP_END_ADDRESS-4)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    if (_FLASH_ProgramWord(*FlashAddress, *(uint32_t*)(Data+i)) == FLASH_COMPLETE)
    {
     /* Check the written value */
      if (*(uint32_t*)*FlashAddress != *(uint32_t*)(Data+i))
      {
        /* Flash content doesn't match SRAM content */
        return(2);
      }
      /* Increment FLASH destination address */
      *FlashAddress += 4;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return (1);
    }
  }

  return (0);
}

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if(Address >= ADDR_FLASH_SECTOR_4)
  {
    sector = FLASH_Sector_4;  
  }
    return sector;
}
static uint32_t curr_addr_prg=0;
void set_curr_addr_prg(uint32_t *iaddr)
{
  curr_addr_prg= *iaddr;
}

uint8_t erase_sectors(uint8_t *data)
{
return FLASH_If_Erase(0);  
}
#define APP_BASE_ADDRESS        ((uint32_t)0x08010000)
#define APP_PAGE_SIZE           ((uint32_t)0x10000)           ////64 KB
#define APP_END_ADDRESS         ((uint32_t)(APP_BASE_ADDRESS + (APP_PAGE_SIZE - 1)))

uint8_t check_erase_sectors(uint8_t *data)
{
uint32_t ii;
uint8_t tdata;
for(ii=APP_BASE_ADDRESS;ii<APP_END_ADDRESS;ii++)
{
 tdata= *(uint8_t*)(ii); 
 if(tdata!=0xff)
   return ERROR_ERRASE;
}
return ERROR_OK;  
}

uint8_t prg_dat(uint8_t *data)
{
uint8_t ii;   
FLASH_Status t_fl_stat=FLASH_COMPLETE;  
uint8_t num_words;
////uint32_t addr_prg;
prg_flash_cmd_t *p_prg_flash_cmd=(prg_flash_cmd_t *)data;
num_words=p_prg_flash_cmd->num_bytes/2;
////  printk("\n\r prg_dat[%x] =>",num_words); 

if((num_words>MAX_NUM_WORDS_PRG)||(num_words==0))
  return ERROR_NUM_BYTES_PRG;
FLASH_Unlock();
for(ii=0;ii<num_words;ii++)
  {
  t_fl_stat=_FLASH_ProgramHalfWord(curr_addr_prg, p_prg_flash_cmd->data[ii]); 
  if(t_fl_stat!=FLASH_COMPLETE ) 
    break;
  curr_addr_prg+=2;
  }
FLASH_Lock();

if(t_fl_stat==FLASH_COMPLETE ) 
  return 0;  
else
  return ERROR_FLAH_PRG;

}
#endif
uint8_t check_ks_app(void)
{
uint16_t rd_ks=0;
uint16_t tmp_ks=0;
uint32_t size_app=0;
uint32_t ii;
uint16_t tmp=0;

if(EE_ReadVariable(ADDR_EEPROM_SIZEH_APP, &tmp)!=0)
  return 0;
size_app=tmp;
size_app<<=16;
if(EE_ReadVariable(ADDR_EEPROM_SIZEL_APP, &tmp)!=0)
  return 0;
size_app|=tmp;
if(EE_ReadVariable(ADDR_KS_APP, &rd_ks)!=0)
  return 0;
for(ii=0;ii< size_app;ii+=2)
  {
   tmp_ks+= *(uint16_t*)(APP_BASE_ADDRESS+ii); 
  }
if(tmp_ks!=rd_ks)
  return 0;
return 1;
}
////=================================================
