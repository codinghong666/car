#ifndef __SPI_H_
#define __SPI_H_

#include "config.h"

#define SPI_SCLK P32
#define SPI_MISO P33
#define SPI_MOSI P34
#define SPI_CS P35

void SPI_init(void);
void FlashCheckID(void);
u8 CheckFlashBusy(void);
void FlashWriteEnable(void);
void FlashChipErase(void);
void FlashSectorErase(u32 addr);
void SPI_Read_Nbytes(u32 addr, u8 *buffer, u16 size);
u8 SPI_Read_Compare(u32 addr, u8 *buffer, u16 size);
void SPI_Write_Nbytes(u32 addr, u8 *buffer, u8 size);
#endif