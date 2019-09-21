/*
 * sensor.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include "sensor.h"

void Sensor::init ( I2cDriver & i2c ) {
	this->i2c = & i2c ;
	i2c.begin () ;
}
uint8_t	Sensor::readSensor  ( int reg ) {
	return i2c->read ( i2cAdress , (uint8_t) reg ) ;
}
void Sensor::readSensor ( int reg , uint8_t * data , uint8_t length ) {
	return i2c->read ( i2cAdress , (uint8_t) reg , data , length ) ;
}
void Sensor::writeSensor ( uint8_t reg, uint8_t value ) {
	i2c->write ( i2cAdress , reg , value ) ;
}

