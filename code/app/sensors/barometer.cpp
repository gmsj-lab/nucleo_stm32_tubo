/*
 * barometer.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#include "barometer.h"

#include <math.h>
#include <limits.h>

#define BMP180_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */


void Barometer::read16 ( uint8_t reg, uint16_t * value ) {

	readSensor ( reg , (uint8_t *) value , (uint8_t) sizeof ( uint16_t ) ) ;
}

void Barometer::readS16 ( uint8_t reg, int16_t *value ) {
	uint16_t i ;
	read16 ( reg, &i ) ;
	*value = (int16_t) i ;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
 */
/**************************************************************************/
void Barometer::readCoefficients ( void ) {
#if BMP180_USE_DATASHEET_VALS
	calib_coeffs.ac1 = 408;
	calib_coeffs.ac2 = -72;
	calib_coeffs.ac3 = -14383;
	calib_coeffs.ac4 = 32741;
	calib_coeffs.ac5 = 32757;
	calib_coeffs.ac6 = 23153;
	calib_coeffs.b1  = 6190;
	calib_coeffs.b2  = 4;
	calib_coeffs.mb  = -32768;
	calib_coeffs.mc  = -8711;
	calib_coeffs.md  = 2868;
	mode        	 = 0;
#else
	readS16 ( BMP180_REGISTER_CAL_AC1, &calib_coeffs.ac1 ) ;
	readS16 ( BMP180_REGISTER_CAL_AC2, &calib_coeffs.ac2 ) ;
	readS16 ( BMP180_REGISTER_CAL_AC3, &calib_coeffs.ac3 ) ;
	read16 ( BMP180_REGISTER_CAL_AC4, &calib_coeffs.ac4 ) ;
	read16 ( BMP180_REGISTER_CAL_AC5, &calib_coeffs.ac5 ) ;
	read16 ( BMP180_REGISTER_CAL_AC6, &calib_coeffs.ac6 ) ;
	readS16 ( BMP180_REGISTER_CAL_B1, &calib_coeffs.b1 ) ;
	readS16 ( BMP180_REGISTER_CAL_B2, &calib_coeffs.b2 ) ;
	readS16 ( BMP180_REGISTER_CAL_MB, &calib_coeffs.mb ) ;
	readS16 ( BMP180_REGISTER_CAL_MC, &calib_coeffs.mc ) ;
	readS16 ( BMP180_REGISTER_CAL_MD, &calib_coeffs.md ) ;
#endif
}

void Barometer::readRawTemperature ( int32_t * temperature ) {
#if BMP180_USE_DATASHEET_VALS
	*temperature = 27898 ;
#else
	uint16_t t ;
	writeSensor ( BMP180_REGISTER_CONTROL, BMP180_REGISTER_READTEMPCMD ) ;
	delay ( 5 ) ;
	read16 ( BMP180_REGISTER_TEMPDATA, & t ) ;
	*temperature = t ;
#endif
}

void Barometer::readRawPressure ( int32_t * pressure ) {
#if BMP180_USE_DATASHEET_VALS
	*pressure = 23843 ;
#else
	uint8_t  p8 ;
	uint16_t p16 ;
	int32_t  p32 ;

	writeSensor ( BMP180_REGISTER_CONTROL, BMP180_REGISTER_READPRESSURECMD + ( mode << 6 ) ) ;
	switch ( mode )
	{
	case BMP180_MODE_ULTRALOWPOWER :
		delay ( 5 ) ;
		break;
	case BMP180_MODE_STANDARD :
		delay ( 8 ) ;
		break;
	case BMP180_MODE_HIGHRES :
		delay ( 14 ) ;
		break;
	case BMP180_MODE_ULTRAHIGHRES :
	default:
		delay ( 26 ) ;
		break;
	}

	read16 ( BMP180_REGISTER_PRESSUREDATA, & p16 ) ;
	p32 = (uint32_t) p16 << 8 ;
	p8 = readSensor ( BMP180_REGISTER_PRESSUREDATA + 2 ) ;
	p32 += p8 ;
	p32 >>= ( 8 - mode ) ;

	*pressure = p32 ;
#endif
}

/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
 */
/**************************************************************************/
int32_t Barometer::computeB5 ( int32_t ut ) {
	int32_t X1 = (ut - (int32_t)calib_coeffs.ac6) * ((int32_t)calib_coeffs.ac5) >> 15;
	int32_t X2 = ((int32_t)calib_coeffs.mc << 11) / (X1+(int32_t)calib_coeffs.md ) ;
	return X1 + X2;
}

Barometer::Barometer ()  : Sensor ( BMP180_ADDRESS ) {
	mode = 0 ;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

void Barometer::init ( I2cDriver & i2c , bmp180_mode_t mode ) {
	Sensor::init ( i2c ) ;

	/* Mode boundary check */
	if ((mode > BMP180_MODE_ULTRAHIGHRES ) || ( mode < 0 ) ) {
		mode = BMP180_MODE_ULTRAHIGHRES ;
	}

	/* Make sure we have the right device */
	uint8_t id = readSensor ( BMP180_REGISTER_CHIPID ) ;
	if ( id != BMP180_CHIPID ) {
		app_error ( BAROMETER_CONFIG ) ;
	}
	/* Set the mode indicator */
	this->mode = mode ;

	/* Coefficients need to be read once */
	readCoefficients () ;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
 */
/**************************************************************************/
double Barometer::getPressure ( void ) {
	int32_t  ut = 0, up = 0, compp = 0;
	int32_t  x1, x2, b5, b6, x3, b3, p;
	uint32_t b4, b7;

	/* Get the raw pressure and temperature values */
	readRawTemperature(&ut ) ;
	readRawPressure(&up ) ;

	/* Temperature compensation */
	b5 = computeB5(ut ) ;

	/* Pressure compensation */
	b6 = b5 - 4000;
	x1 = (calib_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
	x2 = (calib_coeffs.ac2 * b6) >> 11;
	x3 = x1 + x2;
	b3 = (((((int32_t) calib_coeffs.ac1) * 4 + x3) << mode) + 2) >> 2;
	x1 = (calib_coeffs.ac3 * b6) >> 13;
	x2 = (calib_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (calib_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) (up - b3) * (50000 >> mode) ) ;

	if (b7 < 0x80000000)
	{
		p = (b7 << 1) / b4;
	}
	else
	{
		p = (b7 / b4) << 1;
	}

	x1 = (p >> 8) * (p >> 8 ) ;
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	compp = p + ((x1 + x2 + 3791) >> 4 ) ;

	/* Assign compensated pressure value */
	return compp ;
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
 */
/**************************************************************************/
double Barometer::getTemperature ( void ) {
	int32_t UT, B5 ;     // following ds convention
	double t ;

	readRawTemperature ( &UT ) ;

#if BMP180_USE_DATASHEET_VALS
	// use datasheet numbers!
	UT = 27898;
	calib_coeffs.ac6 = 23153;
	calib_coeffs.ac5 = 32757;
	calib_coeffs.mc = -8711;
	calib_coeffs.md = 2868;
#endif

	B5 = computeB5(UT ) ;
	t = ( B5 + 8 ) >> 4 ;
	t /= 10 ;

	return t ;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
 */
/**************************************************************************/
double Barometer::pressureToAltitude ( double seaLevel, double atmospheric ) {
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude.  See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return 44330.0 * ( 1.0 - pow ( atmospheric / seaLevel, 0.1903 ) ) ;
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
 */
/**************************************************************************/
double Barometer::seaLevelForAltitude ( double altitude, double atmospheric ) {
	// Equation taken from BMP180 datasheet (page 17):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude.  See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return atmospheric / pow ( 1.0 - ( altitude / 44330.0 ) , 5.255 ) ;
}

double_3D_t & Barometer::read ( void ) {

	scaledData.pressure = getPressure () / 100.0F ;
	return scaledData ;
}


