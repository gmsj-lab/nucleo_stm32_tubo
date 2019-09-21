/*
 * cpu_load.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */

#ifndef CODE_UTIL_CPU_LOAD_H_
#define CODE_UTIL_CPU_LOAD_H_

#define TIME_PERIOD	1000000		// measurement iterval in us

#include "utils.h"
#include "target_attribute.h"

class CpuLoad {

private:
	uint32_t	idleReferenceLap ;
	uint32_t	lastLapTime ;
	uint32_t	maxLapTime ;
	uint32_t	idleTime ;
	uint32_t	busyTime ;

	TT_bool		onOff ;
	TT_uint32	cpuload ;
	TT_uint32	lapMax ;

public:
	CpuLoad ( const char * name = "CPU LOAD" ) :
		onOff 	( "OnOff"			, name ) ,
		cpuload	( "Cpuload"			, name ) ,
		lapMax	( "MaxLoopTime"  	, name )
	{
		idleReferenceLap = TIME_PERIOD ;
		lastLapTime		 = 0 ;
		maxLapTime		 = 0 ;
		idleTime  		 = 0 ;
		busyTime 		 = 0 ;

		cpuload .setReadOnly ( true ) ;
		lapMax	.setReadOnly ( true ) ;
	}
	~CpuLoad () {} ;

	inline void lap () {
		if ( onOff.get () ) {
			uint32_t lap = micros () - lastLapTime ;
			if ( lap < idleReferenceLap ) {
				idleReferenceLap = lap ;
			}
			if ( lap > maxLapTime ) {
				maxLapTime = lap ;
			}
			idleTime += idleReferenceLap ;
			busyTime += lap - idleReferenceLap ;
			uint32_t totalTime = idleTime + busyTime ;

			if ( totalTime > TIME_PERIOD ) {

				cpuload.set ( ( busyTime / (double)totalTime ) * 100.0 ) ;
				if (maxLapTime > lapMax.get () ) lapMax.set ( maxLapTime ) ;

				idleTime 	= 0 ;
				busyTime 	= 0 ;
				maxLapTime 	= 0 ;
			}
		}
	}
} ;

#endif /* CODE_UTIL_CPU_LOAD_H_ */
