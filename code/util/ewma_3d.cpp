/*
 * ewma_3d.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#if 0
#include <ewma_3d.h>

Ewma3D::Ewma3D( uint8_t size ) {
	decay  = 2.0 / ( (size / 2.0) + 1.0 ) ;
	output.x = 0.0 ;
	output.y = 0.0 ;
	output.z = 0.0 ;
}

void Ewma3D::init ( double_3D_t & sample ) {
	output = sample ;
}

void Ewma3D::set ( double_3D_t & sample ) {
	output.x = (sample.x * decay) + (output.x * (1.0 - decay) ) ;
	output.y = (sample.y * decay) + (output.y * (1.0 - decay) ) ;
	output.z = (sample.z * decay) + (output.z * (1.0 - decay) ) ;
}

double_3D_t & Ewma3D::get ( void ) {
	return output ;
}
#endif
