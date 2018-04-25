/*********************************************************************/
/* Utility functions                      													 */
/* Name:  Carson Clarke-Magrab, Fayez Mehdad                         */
/* Date:  4/16/2018                                                  */
/* Class:  CMPE 250                                                  */
/* Section:  Section 01, Tuesday 2PM - 4PM                           */
/*-------------------------------------------------------------------*/

#include <stdlib.h>

void shuffle(char array[], int size) {
/***************************************************************/
/* Shuffles an array of size n                                 */
/***************************************************************/
	if (size > 1) {
		int j; 
		char tmp;
		for (int i = size - 1; i > 0; i--) {
			j = rand() % (i + 1);
			tmp = array[j];
			array[j] = array[i];
			array[i] = tmp;
		}
	}
}



