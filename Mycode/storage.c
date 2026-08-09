/**
  * @attention 本存储模块文件基于myflash文件实现，使用前需确保已包含myflash文件
  */
#include "stm32f4xx_hal.h"
#include "myflash.h"
#include <stdint.h>
#include "storage.h"

STORAGE_TYPE STORAGE_Data;//存储模块的数据放在该结构体

/**
  * @brief 存储模块初始化，把标志位和数据从flash读取到sram结构体中，如果是第一次使用存储模块，则会做好准备工作后再进行读取
  */
void STORAGE_Init(void){
  //读取第一个字的标志位，if成立，则执行第一次使用的初始化
  if(MYFLASH_ReadWord(STORAGE_START_ADDRESS) != STORAGE_Flag){
    MYFLASH_EraseSector(FLASH_SECTOR_7);  //擦除扇区7
    MYFLASH_ProgramWord(STORAGE_START_ADDRESS, STORAGE_Flag); //存储标志位
    for(uint32_t i = 1; i <= STORAGE_COUNT-1; i++){ //存储数据
      MYFLASH_ProgramWord(STORAGE_START_ADDRESS + i*4, *((uint32_t *)(&STORAGE_Data) + i));
    }
  }
  //把标志位和数字从flash读取到sram数组中
  for(uint32_t i = 0; i <= STORAGE_COUNT-1; i++){
    *((uint32_t *)(&STORAGE_Data) + i) = MYFLASH_ReadWord(STORAGE_START_ADDRESS + i*4);
  }
}

/**
  * @brief 存储模块保存，把标志位和数据从sram数组存储到flash中
  */
void STORAGE_Save(void){
  MYFLASH_EraseSector(FLASH_SECTOR_7);  //擦除扇区7
  for(uint32_t i = 0; i <= STORAGE_COUNT-1; i++){ //存储标志位和数据
    MYFLASH_ProgramWord(STORAGE_START_ADDRESS + i*4, *((uint32_t *)(&STORAGE_Data) + i));
  }
}

/**
  * @brief 存储模块清空，把存储模块的有效数据清空
  */
void STORAGE_Clear(void){
  for(uint32_t i = 1; i<= STORAGE_COUNT-1; i++){//把sram数组的有效数据清空
    *((uint32_t *)(&STORAGE_Data) + i) = 0x00000000;
  }
  STORAGE_Save();//存储模块保存
}
