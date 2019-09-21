/*
 * eeprom_store.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_APP_EEPROM_STORE_H_
#define CODE_APP_EEPROM_STORE_H_

#include "sensor.h"
#include "radio_control.h"

#define EEPROM_VALIDITY_MARK_UP	0xFADECAFE

typedef struct _STORE {

	uint32_t validity ;
	double tiltKp ;
	double tiltKi ;
	double tiltKd ;

	double orientKp	;
	double orientKi	;
	double orientKd	;

	double velocityKp ;

	double positionKp ;
	double positionKi ;
	double positionKd ;

	double_3D_t 		offset ;

	int16_3D_t  		magnetoOffset 	   ;
	double_3D_t 		magnetoScaleFactor ;

	RadioCalibration    radioCalibration   ;
} Store ;

#endif /* CODE_APP_EEPROM_STORE_H_ */
