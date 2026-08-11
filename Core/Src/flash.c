/*
 * flash.c
 *
 *  Created on: Nov 9, 2025
 *      Author: samfishy
 */

#include "main.h"
#include "flash.h"

extern SPI_HandleTypeDef hspi1;
#define FLASH_SPI hspi1

#define blocks 64

void W25_rst (void)
{
	uint8_t temp[2];
	temp[0] = 0x66;
	temp[1] = 0x99;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, temp, 2, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	HAL_Delay(100);
}

uint32_t W25_devID (void)
{
	uint8_t temp = 0x9F;
	uint8_t rdata[3];

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, &temp, 1, 100);
	HAL_SPI_Receive(&FLASH_SPI, rdata, 3, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
	return ((rdata[0]<<16)|(rdata[1]<<8)|rdata[2]);
}

void W25_nRead (uint32_t stPage, uint8_t offset,uint32_t size,uint8_t *rData)
{
	uint8_t temp[4] ;
	uint32_t memAddr = (stPage*256) + offset;

	temp[0] = 0x03;
	temp[1] = (memAddr>>16)& 0xFF;
	temp[2] = (memAddr>>8)& 0xFF;
	temp[3] = (memAddr)& 0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, temp, 4, 100);
	HAL_SPI_Receive(&FLASH_SPI,rData,size,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
}

void W25_fRead (uint32_t stPage, uint8_t offset,uint32_t size,uint8_t *rData)
{
	uint8_t temp[5] ;
	uint32_t memAddr = (stPage*256) + offset;

	temp[0] = 0x0B;
	temp[1] = (memAddr>>16)& 0xFF;
	temp[2] = (memAddr>>8)& 0xFF;
	temp[3] = (memAddr)& 0xFF;
	temp[4] = 0;


	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, temp, 5, 100);
	HAL_SPI_Receive(&FLASH_SPI,rData,size,100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
}

void W25_writeENB()
{
	uint8_t temp = 0x06 ;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, &temp, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
}

void W25_writeDIS()
{
	uint8_t temp = 0x04 ;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, &temp, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
}

void W25_erase(uint16_t numSec)
{
	uint8_t temp[5] ;
	uint32_t memAddr = numSec*16*256;

	W25_writeENB();

	temp[0] = 0x20;
	temp[1] = (memAddr>>16)& 0xFF;
	temp[2] = (memAddr>>8)& 0xFF;
	temp[3] = (memAddr)& 0xFF;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_SPI_Transmit(&FLASH_SPI, temp, 4, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);

	HAL_Delay(450);

	W25_writeDIS();
}

uint32_t bytestowrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}

void W25Q_Write_Page (uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage  = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage-startPage+1;

	uint16_t startSector  = startPage/16;
	uint16_t endSector  = endPage/16;
	uint16_t numSectors = endSector-startSector+1;

	for (uint16_t i=0; i<numSectors; i++)
	{
		W25_erase(startSector++);
	}


	uint32_t dataPosition = 0;

	// write the data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256)+offset;
		uint16_t bytesremaining  = bytestowrite(size, offset);
		uint32_t indx = 0;

		W25_writeENB();

		tData[0] = 0x02;  // page program
		tData[1] = (memAddr>>16)&0xFF;  // MSB of the memory Address
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr)&0xFF; // LSB of the memory Address

		indx = 4;
		uint16_t bytestosend  = bytesremaining + indx;

		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}

		if (bytestosend > 200)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
			HAL_SPI_Transmit(&FLASH_SPI,tData, 100,100);
			HAL_SPI_Transmit(&FLASH_SPI,tData+100, bytestosend-100,100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
		}

		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
			HAL_SPI_Transmit(&FLASH_SPI,tData, bytestosend,100);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
		}

		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;

		HAL_Delay(5);
		W25_writeDIS();

	}
}

