/*********************************************************************/
/* LCD display functions													                   */
/* Name:	Carson Clarke-Magrab, Fayez Mehdad												 */
/* Date:	4/15/2018																									 */
/* Class:	CMPE 250																									 */
/* Section:	Section 01, Tuesday 2PM - 4PM													   */
/*-------------------------------------------------------------------*/

#include "LCD.h"

void LCD_PutDec(int num) {
	unsigned short hex;
	int dig;
	for (int i = 0; i < 4; i++) {
		dig = (num % 10) << (i * 4);
		hex = hex | dig;
		num /= 10;
	}
	LCD_PutHex(hex);
}
