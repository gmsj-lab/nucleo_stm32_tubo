/*
 * BMP180.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_SENSORS_BMP180_H_
#define CODE_APP_SENSORS_BMP180_H_

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP085_ADDRESS                	( 0x77 << 1 )
#define BMP180_ADDRESS           	  	BMP085_ADDRESS

#define	BMP180_CHIPID 		  			0x55
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
	BMP180_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
	BMP180_REGISTER_CHIPID             = 0xD0,
	BMP180_REGISTER_VERSION            = 0xD1,
	BMP180_REGISTER_SOFTRESET          = 0xE0,
	BMP180_REGISTER_CONTROL            = 0xF4,
	BMP180_REGISTER_TEMPDATA           = 0xF6,
	BMP180_REGISTER_PRESSUREDATA       = 0xF6,
	BMP180_REGISTER_READTEMPCMD        = 0x2E,
	BMP180_REGISTER_READPRESSURECMD    = 0x34
};
/*=========================================================================*/

/*=========================================================================
    MODE SETTINGS
    -----------------------------------------------------------------------*/
typedef enum
{
	BMP180_MODE_ULTRALOWPOWER          = 0,
	BMP180_MODE_STANDARD               = 1,
	BMP180_MODE_HIGHRES                = 2,
	BMP180_MODE_ULTRAHIGHRES           = 3
} bmp180_mode_t;
/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
typedef struct
{
	int16_t  ac1;
	int16_t  ac2;
	int16_t  ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t  b1;
	int16_t  b2;
	int16_t  mb;
	int16_t  mc;
	int16_t  md;
} bmp180_calib_data;
/*=========================================================================*/

#endif /* CODE_APP_SENSORS_BMP180_H_ */
