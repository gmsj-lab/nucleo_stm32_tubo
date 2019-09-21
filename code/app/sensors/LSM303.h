/*
 * LSM303.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_SENSORS_LSM303_H_
#define CODE_APP_SENSORS_LSM303_H_

//=========================================================================
//   						 ACCELEROMETER
//=========================================================================

//_________________________________________________________________________
//    I2C ADDRESS
#define LSM303_ADDRESS_ACCEL          (0x32)         // 0011001x

//_________________________________________________________________________
//  ACCELEROMETER REGISTERS ADDRESS
typedef enum
{                                                     // DEFAULT    TYPE
	LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,   // 00000111   rw
	LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,   // 00000000   rw
	LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,   // 00000000   rw
	LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,   // 00000000   r
	LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,   // 00000000   r
	LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
	LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
	LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
	LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
	LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
	LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
	LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
	LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
	LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
	LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
	LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
	LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
	LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
	LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
	LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
	LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
	LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
	LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
	LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
	LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
	LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
	LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
} lsm303AccelRegisters_t;

//_________________________________________________________________________
//  ACCELEROMETER REGISTERS VALUES

// LSM303_REGISTER_ACCEL_CTRL_REG1_A :
// Configure :
//		- Bits 0,1,2	=> Enable X, Y, and Z axis
//		- Bits 3		=> Power-down mode (value 1), normal mode (value 0)
//		- Bits 4,5,6,7	=> Output Data Rate :
//			- Normal or Low-power mode  : 1,10,25,50,100,200,400 Hz
//			- Low-power mode 			: 1.620 kHz
//			- Normal mode 				: 1.344 kHz / low-power mode :5.376 kHz

#define ACCEL_CTRL_REG1_A_100HZ_XYZ_NON_LOW_POWER	0x57
#define ACCEL_CTRL_REG1_A_200HZ_XYZ_NON_LOW_POWER	0x67
#define ACCEL_CTRL_REG1_A_400HZ_XYZ_NON_LOW_POWER	0x77
#define ACCEL_CTRL_REG1_A_1344HZ_XYZ_NON_LOW_POWER	0x97

// LSM303_REGISTER_ACCEL_CTRL_REG4_A :
// BDU: Block data update. Default value: 0  (0: continuous update, 1: output registers not updated until MSB and LSB have been read
// BLE: Big/little endian data selection. Default value 0. (0: data LSB @ lower address, 1: data MSB @ lower address)
// FS[1:0]: Full-scale selection. Default value: 00 (00: ±2 g, 01: ±4 g, 10: ±8 g, 11: ±16 g)
// HR: High-resolution output mode: Default value: 0 (0: high-resolution disable, 1: high-resolution enable)
// SIM: SPI serial interface mode selection. Default value: 0 (0: 4-wire interface, 1: 3-wire interface).
#define ACCEL_CTRL_REG4_A_BDU_POS						7
#define ACCEL_CTRL_REG4_A_FS_POS						4
#define ACCEL_CTRL_REG4_A_HR_POS						3

#define ACCEL_CTRL_REG4_A_BDU_CONTINUOUS_UPDATE			0x0
#define ACCEL_CTRL_REG4_A_BDU_WAIT_FOR_UPDATE			0x1		// Wait for MSB and LSB to be read before update

#define ACCEL_CTRL_REG4_A_FS_2_G						0x00
#define ACCEL_CTRL_REG4_A_FS_4_G						0x01
#define ACCEL_CTRL_REG4_A_FS_8_G						0x02
#define ACCEL_CTRL_REG4_A_FS_16_G						0x03

#define ACCEL_CTRL_REG4_A_HIGH_RES_DISABLED				0x00
#define ACCEL_CTRL_REG4_A_HIGH_RES_ENABLED				0x01

#define LSM303_ACCEL_STATUS_REG_ZYX_DATA_OVERRUN		0x80
#define LSM303_ACCEL_STATUS_REG_ZYX_DATA_READY			0x08

//=========================================================================
//   						 MAGNETOMETER
//=========================================================================

//_________________________________________________________________________
//    I2C ADDRESS
#define LSM303_ADDRESS_MAG            (0x3C)         // 0011110x

//_________________________________________________________________________
//  MAGNETOMETER REGISTERS ADDRESS

typedef enum
{
	LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
	LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
	LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
	LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
	LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
	LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
	LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
	LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
	LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
	LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
	LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
	LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
	LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
	LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
	LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
} lsm303MagRegisters_t;

//=========================================================================
//    MAGNETOMETER GAIN SETTINGS
typedef enum
{
	LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
	LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
	LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
	LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
	LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
	LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
	LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} lsm303MagGain ;

// OUTPUT LSB VALUES ACCORDING TO GAIN
#define MAGGAIN_1_3_LSB_XY						1100
#define MAGGAIN_1_3_LSB_Z						 980
#define MAGGAIN_1_9_LSB_XY						 855
#define MAGGAIN_1_9_LSB_Z						 760

#define MAGGAIN_2_5_LSB_XY						 670
#define MAGGAIN_2_5_LSB_Z						 600
#define MAGGAIN_4_0_LSB_XY						 450
#define MAGGAIN_4_0_LSB_Z						 400
#define MAGGAIN_4_7_LSB_XY						 400
#define MAGGAIN_4_7_LSB_Z						 355
#define MAGGAIN_5_6_LSB_XY						 330
#define MAGGAIN_5_6_LSB_Z						 295
#define MAGGAIN_8_1_LSB_XY						 230
#define MAGGAIN_8_1_LSB_Z						 205

//=========================================================================
//    MAGNETOMETER UPDATE RATE SETTINGS
typedef enum
{
	LSM303_MAGRATE_0_7                        = 0x00,  // 0.75 Hz
	LSM303_MAGRATE_1_5                        = 0x01,  // 1.5 Hz
	LSM303_MAGRATE_3_0                        = 0x62,  // 3.0 Hz
	LSM303_MAGRATE_7_5                        = 0x03,  // 7.5 Hz
	LSM303_MAGRATE_15                         = 0x04,  // 15 Hz
	LSM303_MAGRATE_30                         = 0x05,  // 30 Hz
	LSM303_MAGRATE_75                         = 0x06,  // 75 Hz
	LSM303_MAGRATE_220                        = 0x07   // 200 Hz
} lsm303MagRate ;

#define MR_REG_M_CONTINUOUS_CONVERSION_MODE		0x00

#define CRA_REG_M_MAGRATE_MASK					0x07
#define CRA_REG_M_MAGRATE_POS					2

#define CRA_REG_M_TEMPERATURE_ENABLE			0x80
#define CRA_REG_M_TEMPERATURE_DISABLE			0x00

#define LSM303_MAG_STATUS_REG_DATA_READY		0x01
#endif /* CODE_APP_SENSORS_LSM303_H_ */
