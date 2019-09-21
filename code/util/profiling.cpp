/*
 * profiling.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#include <profiling.h>
#if USE_PROFILING == true

Profiling::Profiling (  const char * group ,
						const char * name1 ,
						const char * name2 ,
						const char * name3 ,
						const char * name4 ,
						const char * name5 ) :

		on 	    	( "OnOff" 	, group ) ,
		defaultLevel( "other" 	, group ) ,
		level1  	( name1 	, group ) ,
		level2  	( name2 	, group ) ,
		level3  	( name3 	, group ) ,
		level4  	( name4 	, group ) ,
		level5  	( name5 	, group )
	{
		lastTime 	  = 0 ;
		currentLevel  = 0 ;
		stack.push ( currentLevel ) ;

		defaultLevel.setReadOnly ( true ) ;
		level1		.setReadOnly ( true ) ;
		level2		.setReadOnly ( true ) ;
		level3		.setReadOnly ( true ) ;
		level4		.setReadOnly ( true ) ;
		level5		.setReadOnly ( true ) ;

		clear () ;
	}
#endif


