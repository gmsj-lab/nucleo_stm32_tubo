/*
 * L3GD20.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_SENSORS_L3GD20_H_
#define CODE_APP_SENSORS_L3GD20_H_


/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
#define L3GD20_ADDRESS           (0x6B << 1)   // 1101011
#define L3GD20_POLL_TIMEOUT      (100)         // Maximum number of read attempts
#define L3GD20_ID                0xD4
#define L3GD20H_ID               0xD7
#define GYRO_SENSITIVITY_250DPS  (0.00875)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS  (0.0175)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS (0.070)      // Roughly 18/256
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
typedef enum
{                                             // DEFAULT    TYPE
	GYRO_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
	GYRO_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
	GYRO_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
	GYRO_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
	GYRO_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
	GYRO_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
	GYRO_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
	GYRO_REGISTER_OUT_TEMP            = 0x26,   //            r
	GYRO_REGISTER_STATUS_REG          = 0x27,   //            r
	GYRO_REGISTER_OUT_X_L             = 0x28,   //            r
	GYRO_REGISTER_OUT_X_H             = 0x29,   //            r
	GYRO_REGISTER_OUT_Y_L             = 0x2A,   //            r
	GYRO_REGISTER_OUT_Y_H             = 0x2B,   //            r
	GYRO_REGISTER_OUT_Z_L             = 0x2C,   //            r
	GYRO_REGISTER_OUT_Z_H             = 0x2D,   //            r
	GYRO_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
	GYRO_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
	GYRO_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
	GYRO_REGISTER_INT1_SRC            = 0x31,   //            r
	GYRO_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
	GYRO_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
	GYRO_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
	GYRO_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
	GYRO_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
	GYRO_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
	GYRO_REGISTER_INT1_DURATION       = 0x38,   // 00000000   rw
	GYRO_REGISTER_LOW_ODR       	  = 0x39    // 00000000   rw
} gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
typedef enum
{
	GYRO_RANGE_250DPS  = 250,
	GYRO_RANGE_500DPS  = 500,
	GYRO_RANGE_2000DPS = 2000
} gyroRange_t;
/*=========================================================================*/


#define GYRO_REGISTER_STATUS_ZYX_DATA_OVERRUN		0x80
#define GYRO_REGISTER_STATUS_ZYX_DATA_READY			0x08

#define CTRL_REG1_ODR_200_HZ						0x5F
#define CTRL_REG1_ODR_200_HZ_CUT_12_5				0x4F
#define CTRL_REG1_ODR_200_HZ_CUT_70					0x7F
#define CTRL_REG1_ODR_400_HZ_CUT_20					0x8F
#define CTRL_REG1_ODR_400_HZ_CUT_110				0xBF
#define CTRL_REG1_ODR_800_HZ_CUT_100				0xF7

#endif /* CODE_APP_SENSORS_L3GD20_H_ */
