#ifndef __MYFLASH_H
#define __MYFLASH_H

uint8_t MYFLASH_ReadByte(uint32_t address);
uint16_t MYFLASH_ReadHalfWord(uint32_t address);
uint32_t MYFLASH_ReadWord(uint32_t address);
uint8_t MYFLASH_EraseSector(uint32_t Sector);
void MYFLASH_ProgramByte(uint32_t address, uint8_t Data);
void MYFLASH_ProgramHalfWord(uint32_t address, uint16_t Data);
void MYFLASH_ProgramWord(uint32_t address, uint32_t Data);

#endif
