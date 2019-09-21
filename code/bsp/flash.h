/*
 * flash.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_BSP_FLASH_H_
#define CODE_BSP_FLASH_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f7xx_nucleo_144.h"
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_flash.h"

class Flash {

private:

	uint32_t 			FirstSector ;
	uint32_t			NbOfSectors ;
	uint32_t 			Address  	;
	uint32_t 			SECTORError ;
	volatile uint32_t 	data32 		;
	volatile uint32_t 	MemoryProgramStatus ;

	// Variable used for Erase procedure
	FLASH_EraseInitTypeDef EraseInitStruct ;

	uint32_t 	GetSector 				( uint32_t Address ) ;
	bool 		FLASH_If_Erase 			( void ) ;
	bool 		FLASH_If_Erase_Verify 	( void ) ;
	bool 		FLASH_If_Verify 		( uint32_t * source , uint32_t length ) ;
	bool 		FLASH_If_Write 			( uint32_t * source , uint32_t length ) ;
public:
	Flash () ;
	virtual ~Flash () ;
	bool	get ( uint32_t * data, uint32_t size ) ;	// Returns true if ok
	bool	set ( uint32_t * data, uint32_t size ) ;	// Returns true if ok
};

#ifdef __cplusplus
}
#endif

#endif /* CODE_BSP_FLASH_H_ */
