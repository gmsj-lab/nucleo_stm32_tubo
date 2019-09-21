/*
 * timers.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include <timers.h>

Timers::Timers ( TIM_HandleTypeDef & htim ) {

	tickCount		= 0 ;
	   _1msCallback = NULL ;
	   _2msCallback = NULL ;
	_2_5_msCallback = NULL ;
	   _5msCallback = NULL ;
	  _20msCallback = NULL ;
	  _40msCallback = NULL ;
	 _100msCallback = NULL ;
	_1000msCallback = NULL ;

	this->htim = & htim ;
}

void Timers::init ( void ) {
	HAL_TIM_Base_Start_IT ( htim ) ;
}

void Timers::stop ( void ) {
	HAL_TIM_Base_Stop_IT ( htim ) ;
}

void Timers::setCallback (  TimersFrequency frequency, Callback callback ) {
	switch ( frequency ) {
	case _1000_HZ_TIMER :
		_1msCallback 	= callback ;
		break ;
	case _500_HZ_TIMER :
		_2msCallback 	= callback ;
		break ;
	case _400_HZ_TIMER :
		_2_5_msCallback = callback ;
		break ;
	case _200_HZ_TIMER :
		_5msCallback 	= callback ;
		break ;
	case _50_HZ_TIMER :
		_20msCallback 	= callback ;
		break ;
	case _25_HZ_TIMER :
		_40msCallback 	= callback ;
		break ;
	case _10_HZ_TIMER :
		_100msCallback 	= callback ;
		break ;
	case _1_HZ_TIMER :
		_1000msCallback = callback ;
		break ;
	}
}

#if 0
// Called every 1 ms (1000 Hz)
void Timers::timersCallback () {

	tickCount ++ ;
	if ( _1msCallback )      	  _1msCallback () ;

	if ( tickCount % 2 == 0 ) {
		if ( _2msCallback ) 	  _2msCallback () ;
	} else return ;

	if ( tickCount % 5 == 0 ) {
		if ( _5msCallback ) 	  _5msCallback () ;
	} else return ;

	if ( tickCount % 20 == 0 ) {
		if ( _20msCallback )     _20msCallback () ;
	} else return ;

	if ( tickCount % 40 == 0 ) {
		if ( _40msCallback )     _40msCallback () ;
	}

	if ( tickCount % 100 == 0 ) {
		if ( _100msCallback )   _100msCallback () ;
	} else return ;

	if ( tickCount % 1000 == 0 ) {
		if ( _1000msCallback ) _1000msCallback () ;
	}
}
#endif
#if 0
// Called every 5 ms (200 Hz)
void Timers::timersCallback () {
	// A tick every 5 ms..
	tickCount ++ ;

	if ( _5msCallback )       	  _5msCallback () ;

	if ( tickCount % 4 == 0 ) {
		if ( _20msCallback )     _20msCallback () ;
	} else return ;

	if ( tickCount % 8 == 0 ) {
		if ( _40msCallback )     _40msCallback () ;
	}

	if ( tickCount % 20 == 0 ) {
		if ( _100msCallback )   _100msCallback () ;
	} else return ;

	if ( tickCount % 200 == 0 ) {
		if ( _1000msCallback ) _1000msCallback () ;
	}
}
#else
// Called every 2,5 ms (400 Hz)
void Timers::timersCallback () {

	tickCount ++ ;
	if ( _2_5_msCallback )     _2_5_msCallback () ;

	if ( tickCount % 2 == 0 ) {
		if ( _5msCallback ) 	  _5msCallback () ;
	} else return ;

	if ( tickCount % 8 == 0 ) {
		if ( _20msCallback )     _20msCallback () ;
	}

	if ( tickCount % 16 == 0 ) {
		if ( _40msCallback )     _40msCallback () ;
	}

	if ( tickCount % 40 == 0 ) {
		if ( _100msCallback )   _100msCallback () ;
	} else return ;

	if ( tickCount % 400 == 0 ) {
		if ( _1000msCallback ) _1000msCallback () ;
	}
}
#endif
