/*
 * quick_sort.cpp
 *
 *  Created on: 8 sept. 2019
 *      Author: gmsj
 */
#if 0
#include "quick_sort.h"

void quickSort ( int array [] , int right , int left )
{
	int i = left, j = right ;
	int tmp ;
	int pivot = array[ ( left + right ) / 2 ] ;

	// PARTITION
	while ( i <= j )
	{
		// left->right pass : set i to the first element bigger than the pivot
		while ( array [ i ] < pivot ) i++ ;
		// right->left pass : set j to the first element smaller than the pivot
		while ( array [ j ] > pivot ) j-- ;

	   // switch element if bigger is before smaller
		if ( i <= j )
		{
			  tmp = array [ i ] ;
			  array [ i ] = array [ j ] ;
			  array [ j ] = tmp ;
			  i++ ;
			  j-- ;
		}
	}

	// RECURSION
	if ( j >= left )
	{
		// do it on the lower part of array
		quickSort ( array , j , left ) ;
	}
	if ( i < right )
	{
		// do it on the upper part of array
		quickSort ( array , right , i ) ;
	}
}
#endif


