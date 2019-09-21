/*
 * gyrometer.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */
#include "gyrometer.h"

#include <limits.h>

Gyrometer::Gyrometer ()  : Sensor ( L3GD20_ADDRESS ) {
	range	= GYRO_RANGE_250DPS ;
	rate 	= CTRL_REG1_ODR_400_HZ_CUT_110; //CTRL_REG1_ODR_200_HZ_CUT_12_5 ;
}

struct GyroData {
	uint8_t xlo ;
	uint8_t xhi ;
	uint8_t ylo ;
	uint8_t yhi ;
	uint8_t zlo ;
	uint8_t zhi ;
} ;
void Gyrometer::init ( I2cDriver & i2c , gyroRange_t range )
{
	Sensor::init ( i2c ) ;

	/* Set the range the an appropriate value */
	this->range = range ;

	setScaleFactor 		() ;
	defaultCalibration 	() ;

	/* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
	uint8_t id = readSensor ( GYRO_REGISTER_WHO_AM_I ) ;

	if ( id != L3GD20H_ID ) {
		app_error ( GYROMETER_CONFIG ) ;
	}

	/* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

	/* Reset then switch to normal mode and enable all three channels */
	writeSensor ( GYRO_REGISTER_CTRL_REG1, 0x00 ) ;

	writeSensor ( GYRO_REGISTER_CTRL_REG1, rate ) ;

	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

	/* Nothing to do ... keep default values */
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

	/* Nothing to do ... keep default values */
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

	/* Adjust resolution if requested */
	switch ( range ) {
	case GYRO_RANGE_250DPS :
		writeSensor ( GYRO_REGISTER_CTRL_REG4, 0x00 ) ;
		break ;
	case GYRO_RANGE_500DPS :
		writeSensor ( GYRO_REGISTER_CTRL_REG4, 0x10 ) ;
		break ;
	case GYRO_RANGE_2000DPS :
		writeSensor ( GYRO_REGISTER_CTRL_REG4, 0x20 ) ;
		break ;
	}
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

	/* Nothing to do ... keep default values */
	/* ------------------------------------------------------------------ */

}

double_3D_t & Gyrometer::read ( void ) {
	GyroData gyroData ;

	// Check that new data available and no overrun
	uint8_t status = readSensor ( GYRO_REGISTER_STATUS_REG ) ;
	if ( status & GYRO_REGISTER_STATUS_ZYX_DATA_READY ) {
		 BSP.ledRed .off () ;
	}
	else {
		 BSP.ledRed .on () ;
	}
	if( status & GYRO_REGISTER_STATUS_ZYX_DATA_OVERRUN )
		 BSP.ledGreen.on  () ;
	else BSP.ledGreen.off () ;

	/* Read 6 uint8_ts from the sensor */
	readSensor ( GYRO_REGISTER_OUT_X_L | I2C_MULTIPLE_READ , (uint8_t *) & gyroData , (uint8_t) sizeof ( GyroData ) ) ;

	/* Shift values to create properly formed integer (low uint8_t first) */
	rawData.x = (int16_t) ( gyroData.xlo | ( gyroData.xhi << 8 ) ) ;
	rawData.y = (int16_t) ( gyroData.ylo | ( gyroData.yhi << 8 ) ) ;
	rawData.z = (int16_t) ( gyroData.zlo | ( gyroData.zhi << 8 ) ) ;

	return scale () ;
}
void Gyrometer::setScaleFactor ( void ) {
	// Compensate values depending on the resolution and make the scaling convert values to rad/s
	switch ( range ) {
	case GYRO_RANGE_250DPS :
		scaleFactor.x = GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS ;
		scaleFactor.y = GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS ;
		scaleFactor.z = GYRO_SENSITIVITY_250DPS * SENSORS_DPS_TO_RADS ;
		break;
	case GYRO_RANGE_500DPS :
		scaleFactor.x = GYRO_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS ;
		scaleFactor.y = GYRO_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS ;
		scaleFactor.z = GYRO_SENSITIVITY_500DPS * SENSORS_DPS_TO_RADS ;
		break;
	case GYRO_RANGE_2000DPS :
		scaleFactor.x = GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS ;
		scaleFactor.y = GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS ;
		scaleFactor.z = GYRO_SENSITIVITY_2000DPS * SENSORS_DPS_TO_RADS ;
		break;
	}
}
void Gyrometer::calibrateZero ( void ) {

	sensorCalibrateZero ( 0, 0, 0 ) ;
}

void Gyrometer::defaultCalibration ( void ) {
	offset.x 		=  -60 ;
	offset.y 		=   96 ;
	offset.z 		=   37 ;
}


