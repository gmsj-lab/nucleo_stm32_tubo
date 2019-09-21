/*
 * complementary_filter.h
 *
 *  Created on: 8 sept. 2019
 *      Author: gilles
 */

#ifndef CODE_APP_COMPLEMENTARY_FILTER_H_
#define CODE_APP_COMPLEMENTARY_FILTER_H_


#define SHORT_TERM_RATIO	0.98
#define LONG_TERM_RATIO		(1-SHORT_TERM_RATIO)

class ComplementaryFilter {

private:
	double filteredValue ;
	double period ;

public:
	ComplementaryFilter () {
		filteredValue = 0 ;
		period		  = 0 ;
	}
	virtual ~ComplementaryFilter () {} ;

	void  init	( double frequence , double initValue ) {
		this->filteredValue = initValue ;
		this->period 		= 1000 / frequence ;
	}

	double & update	( double value , double derivative ) {

		filteredValue = SHORT_TERM_RATIO * ( filteredValue + derivative * period ) + LONG_TERM_RATIO * value ;
		return filteredValue ;
	}

	double & get ( void ) {
		return filteredValue ;
	}
} ;

#endif /* CODE_APP_COMPLEMENTARY_FILTER_H_ */
