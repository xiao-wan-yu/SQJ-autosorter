#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

/**
  * @brief 读取stm32内部flash指定地址的一个字节的数据
  */
uint8_t MYFLASH_ReadByte(uint32_t address){
  return *((__IO uint8_t *)(address));
}

/**
  * @brief 读取stm32内部flash指定地址的一个半字的数据
  */
uint16_t MYFLASH_ReadHalfWord(uint32_t address){
  return *((__IO uint16_t *)(address));
}

/**
  * @brief 读取stm32内部flash指定地址的一个字的数据
  */
uint32_t MYFLASH_ReadWord(uint32_t address){
  return *((__IO uint32_t *)(address));
}

/**
  * @brief 擦除stm32内部flash指定的扇区
  * @param Sector FLASH_SECTOR_x x取值为：0~7（针对stm32f407vet6）
  * @retval 擦除成功返回0 擦除失败返回1
  */
uint8_t MYFLASH_EraseSector(uint32_t Sector){
  uint8_t retval;
  HAL_FLASH_Unlock(); //flash解锁
  FLASH_EraseInitTypeDef EraseInitStruct = {
    .TypeErase = FLASH_TYPEERASE_SECTORS, //扇区擦除
    .Sector = Sector, //扇区编号
    .NbSectors = 1  //擦除一个扇区
  };
  uint32_t SectorError = 0;
  __disable_irq();    //擦除前关闭中断
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) retval = 0;//擦除成功
  else retval = 1;  //擦除失败
  __enable_irq();     //擦除后使能中断
  HAL_FLASH_Lock(); //flash上锁
  return retval;
}

/**
  * @brief 编程（写入）stm32内部flash指定地址的一个字节的数据
  * @attention 使用该函数前，需确保指定地址已进行过擦除操作
  */
void MYFLASH_ProgramByte(uint32_t address, uint8_t Data){
  HAL_FLASH_Unlock(); //flash解锁
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, Data); //编程一个字
  HAL_FLASH_Lock();   //flash上锁
}

/**
  * @brief 编程（写入）stm32内部flash指定地址的一个半字的数据
  * @attention 使用该函数前，需确保指定地址已进行过擦除操作
  */
void MYFLASH_ProgramHalfWord(uint32_t address, uint16_t Data){
  HAL_FLASH_Unlock(); //flash解锁
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, Data); //编程一个字
  HAL_FLASH_Lock();   //flash上锁
}

/**
  * @brief 编程（写入）stm32内部flash指定地址的一个字的数据
  * @attention 使用该函数前，需确保指定地址已进行过擦除操作
  */
void MYFLASH_ProgramWord(uint32_t address, uint32_t Data){
  HAL_FLASH_Unlock(); //flash解锁
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, Data); //编程一个字
  HAL_FLASH_Lock();   //flash上锁
}







