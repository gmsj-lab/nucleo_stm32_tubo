/*
 * flash.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include <flash.h>
#include <main.h>
#include "stm32f7xx_hal_flash.h"

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) /* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) /* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) /* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) /* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) /* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000) /* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000) /* Base address of Sector 7, 256 Kbytes */

#define FLASH_STORE_START_ADDR  (ADDR_FLASH_SECTOR_3)   /* Start @ of user Flash store area */
#define FLASH_STORE_END_ADDR    (ADDR_FLASH_SECTOR_4-1) /* End @ of user Flash store area */

Flash::Flash () {
	FirstSector = 0 ;
	NbOfSectors = 0 ;
	Address 	= 0 ;
	SECTORError = 0 ;
	data32 		= 0 ;
	MemoryProgramStatus = 0;
}

Flash::~Flash () {
}

bool	Flash::get ( uint32_t * data, uint32_t size ) {
	bool status = true ;
	uint32_t *flashAdress = (uint32_t *) FLASH_STORE_START_ADDR ;


	for ( uint32_t i = 0 ; i < ( size / 4 ) ; i ++  ) {
		data [ i ] = flashAdress [ i ] ;
	}
	HAL_Delay( 0 ) ;

	return status ;
}

bool	Flash::set ( uint32_t * data, uint32_t size ) {
	bool status = true ;

	if ( FLASH_If_Erase () == false ) {
		status = false ;
	}
	else if ( FLASH_If_Erase_Verify () == false ) {
		status = false ;
	}
	else if ( FLASH_If_Write  ( data , size / 4 ) == false ) {
		status = false ;
	}
	else if ( FLASH_If_Verify ( data , size / 4 ) == false ) {
		status = false ;
	}
	return status ;
}

uint32_t Flash::GetSector ( uint32_t Address )
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
	{
		sector = FLASH_SECTOR_7;
	}
	return sector;
}
bool Flash::FLASH_If_Erase ( void )
{
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock () ;

	/* Erase the user Flash area
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	/* Get the 1st sector to erase */
	FirstSector = GetSector(FLASH_STORE_START_ADDR);
	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(FLASH_STORE_END_ADDR) - FirstSector + 1;
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FirstSector;
	EraseInitStruct.NbSectors     = NbOfSectors;

	if (HAL_FLASHEx_Erase ( &EraseInitStruct, &SECTORError ) != HAL_OK)
	{
		HAL_FLASH_Lock () ;
		return false ;
	}
	HAL_FLASH_Lock () ;
	HAL_Delay( 0 ) ;
	return true ;
}

bool Flash::FLASH_If_Erase_Verify () {
	for ( uint32_t *flashAdress = (uint32_t *)FLASH_STORE_START_ADDR ; flashAdress < (uint32_t *)FLASH_STORE_END_ADDR ; flashAdress ++ ) {
		if ( *flashAdress != 0xFFFFFFFF ) {
			return false ;
		}
	}
	HAL_Delay( 0 ) ;
	return true ;
}

bool Flash::FLASH_If_Verify ( uint32_t * source , uint32_t length ) {

	uint32_t *flashAdress = (uint32_t *) FLASH_STORE_START_ADDR ;

	for ( uint32_t i = 0 ; i < length ; i ++  ) {

		if ( * (uint32_t*) ( flashAdress + i ) != * (uint32_t*) ( source + i ) ) {
			return false ;
		}
	}
	HAL_Delay( 0 ) ;
	return true ;
}
/* Public functions ---------------------------------------------------------*/
/**
 * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
 * @note   After writing data buffer, the flash content is checked.
 * @param  destination: start address for target location
 * @param  p_source: pointer on buffer with data to write
 * @param  length: length of data buffer (unit is 32-bit word)
 * @retval uint32_t 0: Data successfully written to Flash memory
 *         1: Error occurred while writing data in Flash memory
 *         2: Written Data in flash memory is different from expected one
 */
bool Flash::FLASH_If_Write ( uint32_t *source , uint32_t length )
{
	uint32_t destination = FLASH_STORE_START_ADDR ;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock () ;

	for ( uint32_t i = 0 ; (i < length) && (destination <= (FLASH_STORE_END_ADDR-4)) ; i++ )
	{
		/* Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word */
		if (HAL_FLASH_Program ( FLASH_TYPEPROGRAM_WORD, destination, *(uint32_t*)(source+i) ) == HAL_OK)
		{
			/* Check the written value */
			if (*(uint32_t*)destination != *(uint32_t*)(source+i))
			{
				/* Flash content doesn't match SRAM content */
				return ( false ) ;
			}
			/* Increment FLASH destination address */
			destination += 4 ;
		}
		else
		{
			/* Error occurred while writing data in Flash memory */
			return ( false ) ;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock () ;
	HAL_Delay( 0 ) ;
	return ( true ) ;
}


