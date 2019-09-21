/*
 * accelrometer.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "accelerometer.h"

Accelerometer::Accelerometer () : Sensor ( LSM303_ADDRESS_ACCEL ) {
	rate = ACCEL_CTRL_REG1_A_400HZ_XYZ_NON_LOW_POWER ;
}

void Accelerometer::init ( I2cDriver & i2c ) {
	Sensor::init ( i2c ) ;

	// Enable the Accelerometer, set freq and power (normal/low)
	writeSensor ( LSM303_REGISTER_ACCEL_CTRL_REG1_A, rate ) ;

	// LSM303DLHC has no WHOAMI register so read CTRL_REG1_A back to check if we are connected or not
	uint8_t reg1_a = readSensor ( LSM303_REGISTER_ACCEL_CTRL_REG1_A ) ;

	if ( reg1_a != rate ) {
		app_error ( ACCELOROMETER_CONFIG ) ;
	}

	writeSensor ( LSM303_REGISTER_ACCEL_CTRL_REG4_A, ( 	( ACCEL_CTRL_REG4_A_BDU_CONTINUOUS_UPDATE << ACCEL_CTRL_REG4_A_BDU_POS ) |
														( ACCEL_CTRL_REG4_A_FS_2_G 			 	  << ACCEL_CTRL_REG4_A_FS_POS ) |
														( ACCEL_CTRL_REG4_A_HIGH_RES_ENABLED	  << ACCEL_CTRL_REG4_A_HR_POS ) ) ) ;

	setScaleFactor 		() ;
	//defaultCalibration 	() ;
}
void Accelerometer::setScaleFactor ( void ) {
	scaleFactor.x = 2.0 / 32768.0 ;
	scaleFactor.y = 2.0 / 32768.0 ;
	scaleFactor.z = 2.0 / 32768.0 ;
}

double_3D_t & Accelerometer::read ( void ) {

	// Check that new data available
	if (readSensor ( LSM303_REGISTER_ACCEL_STATUS_REG_A ) & LSM303_ACCEL_STATUS_REG_ZYX_DATA_READY ) {
		 BSP.ledBlue.off  () ;
	}
	else {
		 BSP.ledBlue.on  () ;
	}

	readSensor ( LSM303_REGISTER_ACCEL_OUT_X_L_A | I2C_MULTIPLE_READ , (uint8_t *) & rawData , (uint8_t) sizeof ( rawData ) ) ;
	return scale () ;
}
void Accelerometer::calibrateZero ( void ) {

	sensorCalibrateZero ( 0, 0, 32768 / 2 ) ;
}
void Accelerometer::defaultCalibration ( void ) {
	offset.x =  -13 ; //     0 ;
	offset.y =  430 ; //   170 ;
	offset.z = -402 ; // - 450 ;
}

