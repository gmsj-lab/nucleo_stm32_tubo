/*
 * magnetometer.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "magnetometer.h"

struct MagnetoData {
	uint8_t xhi ;
	uint8_t xlo ;
	uint8_t zhi ;
	uint8_t zlo ;
	uint8_t yhi ;
	uint8_t ylo ;
} ;

Magnetometer::Magnetometer () : Sensor ( (int32_t)LSM303_ADDRESS_MAG ) {
	rate 					= LSM303_MAGRATE_75 ;
	gain 					= LSM303_MAGGAIN_1_3 ;
	enableTemperatureSensor = CRA_REG_M_TEMPERATURE_DISABLE ;
}

void Magnetometer::init ( I2cDriver & i2c, const int16_3D_t &calibrationOffset, const double_3D_t &calibrationScale, bool enableTemperature) {
	Sensor::init ( i2c ) ;

	// Enable the Temperature Sensor if requested
	enableTemperatureSensor = (enableTemperatureSensor) ? CRA_REG_M_TEMPERATURE_ENABLE : CRA_REG_M_TEMPERATURE_DISABLE ;

	// Enable the magnetometer, set continuous conversion mode
	writeSensor ( LSM303_REGISTER_MAG_MR_REG_M, MR_REG_M_CONTINUOUS_CONVERSION_MODE ) ;

	// LSM303DLHC has no WHOAMI register so read CRA_REG_M to check the default value (0b00010000/0x10)
	// or the value programed  in case of re-init. Warning : assume temperature disabled (MSB=0)
//	uint8_t craRegM = readSensor ( LSM303_REGISTER_MAG_CRA_REG_M ) ;
//	if ( ( craRegM != 0x10 ) && ( craRegM != 0x1c ) ) {
//		app_error ( MAGNETOMETER_CONFIG ) ;
//	}

	setMagGain 			( gain ) ;
	setMagRate 			( rate ) ;
	defaultCalibration 	( calibrationOffset, calibrationScale ) ;
}

void Magnetometer::enableTemperature ( bool enableTemperature ) {
	enableTemperatureSensor = (enableTemperature) ? CRA_REG_M_TEMPERATURE_ENABLE : CRA_REG_M_TEMPERATURE_DISABLE ;
	setMagRate ( rate ) ;
}

void Magnetometer::setMagRate ( lsm303MagRate rate ) {

	uint8_t craRegM = ( (uint8_t) rate & CRA_REG_M_MAGRATE_MASK ) << CRA_REG_M_MAGRATE_POS ;
	craRegM |= enableTemperatureSensor ;

	writeSensor ( LSM303_REGISTER_MAG_CRA_REG_M, craRegM ) ;
}

void Magnetometer::setMagGain ( lsm303MagGain gain ) {
	writeSensor ( LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t) gain ) ;

	switch ( gain )
	{
	case LSM303_MAGGAIN_1_3:
		scaleFactor.x = 1 / (double) MAGGAIN_1_3_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_1_3_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_1_3_LSB_Z ;

		// Values issue from home made calibration
		scaleFactor.x =  0.000919156 ;
		scaleFactor.y =  0.000925663 ;
		scaleFactor.z =  0.0012022 ;
		break;
	case LSM303_MAGGAIN_1_9:
		scaleFactor.x = 1 / (double) MAGGAIN_1_9_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_1_9_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_1_9_LSB_Z ;
		break;
	case LSM303_MAGGAIN_2_5:
		scaleFactor.x = 1 / (double) MAGGAIN_2_5_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_2_5_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_2_5_LSB_Z ;
		break;
	case LSM303_MAGGAIN_4_0:
		scaleFactor.x = 1 / (double) MAGGAIN_4_0_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_4_0_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_4_0_LSB_Z ;
		break;
	case LSM303_MAGGAIN_4_7:
		scaleFactor.x = 1 / (double) MAGGAIN_4_7_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_4_7_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_4_7_LSB_Z ;
		break;
	case LSM303_MAGGAIN_5_6:
		scaleFactor.x = 1 / (double) MAGGAIN_5_6_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_5_6_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_5_6_LSB_Z ;
		break;
	case LSM303_MAGGAIN_8_1:
		scaleFactor.x = 1 / (double) MAGGAIN_8_1_LSB_XY ;
		scaleFactor.y = 1 / (double) MAGGAIN_8_1_LSB_XY ;
		scaleFactor.z = 1 / (double) MAGGAIN_8_1_LSB_Z ;
		break;
	}
}

double_3D_t & Magnetometer::read ( void ) {
	MagnetoData magnetoData ;

	// Check that new data available
	if ( ! ( readSensor ( LSM303_REGISTER_MAG_SR_REG_Mg ) & LSM303_MAG_STATUS_REG_DATA_READY ) ) {
		//BSP.ledGreen.on () ;
	}

	readSensor ( LSM303_REGISTER_MAG_OUT_X_H_M  | I2C_MULTIPLE_READ , (uint8_t *) & magnetoData , (uint8_t) sizeof ( magnetoData ) ) ;

	// Shift values to create properly formed integer (low uint8_t first)
	rawData.x = (int16_t) magnetoData.xlo | ( (int16_t) magnetoData.xhi << 8 ) ;
	rawData.y = (int16_t) magnetoData.ylo | ( (int16_t) magnetoData.yhi << 8 ) ;
	rawData.z = (int16_t) magnetoData.zlo | ( (int16_t) magnetoData.zhi << 8 ) ;

	return scale () ;
}

void Magnetometer::calibrateRange	( void ) {
	int16_3D_t scale ;
	double averageScale ;
	if ( rawData.x > max.x ) { max.x = rawData.x ;	BSP.ledErreur.on () ; }
	if ( rawData.y > max.y ) { max.y = rawData.y ;	BSP.ledM_A	 .on () ; }
	if ( rawData.z > max.z ) { max.z = rawData.z ;	BSP.ledWifi	 .on () ; }

	if ( rawData.x < min.x ) { min.x = rawData.x ;	BSP.ledErreur.on () ; }
	if ( rawData.y < min.y ) { min.y = rawData.y ;	BSP.ledM_A	 .on () ; }
	if ( rawData.z < min.z ) { min.z = rawData.z ;	BSP.ledWifi	 .on () ; }

	// Get hard iron correction estimate
	offset.x = ( max.x + min.x ) / 2.0 ;
	offset.y = ( max.y + min.y ) / 2.0 ;
	offset.z = ( max.z + min.z ) / 2.0 ;

	// Get soft iron correction estimate
	scale.x = ( max.x - min.x ) / 2.0 ;
	scale.y = ( max.y - min.y ) / 2.0 ;
	scale.z = ( max.z - min.z ) / 2.0 ;

	averageScale = ( scale.x + scale.y + scale.z ) / 3.0 ;

	// Set scale factor, Avoid NaN if scale == 0 or averageScale == 0
	if ( ( scale.x != 0 ) && ( scale.y != 0 ) && ( scale.z != 0 ) && ( averageScale != 0 ) && (! isnan(averageScale) ) ) {
		scaleFactor.x = averageScale / (double) scale.x / 1000.0 ;
		scaleFactor.y = averageScale / (double) scale.y / 1000.0 ;
		scaleFactor.z = averageScale / (double) scale.z / 1000.0 ;
	}
}

void Magnetometer::defaultCalibration ( const int16_3D_t & calibrationOffset, const double_3D_t & calibrationScale ) {

	offset.x 	  = calibrationOffset.x ;
	offset.y 	  = calibrationOffset.y ;
	offset.z 	  = calibrationOffset.z ;

	scaleFactor.x = calibrationScale.x ;
	scaleFactor.y = calibrationScale.y ;
	scaleFactor.z =	calibrationScale.z ;
}


