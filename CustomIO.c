/*********************************************************************/
/* Custom IO functions                                               */
/* Name:	Carson Clarke-Magrab, Fayez Mehdad                         */
/* Date:	4/15/2018                                                  */
/* Class:	CMPE 250                                                   */
/* Section:	Section 01, Tuesday 2PM - 4PM                            */
/*-------------------------------------------------------------------*/

#include "CustomIO.h"

/* Output characteristics */
#define MAX_WORD_DECIMAL_DIGITS (10)

void PutNumU (unsigned int num) {
/***************************************************************/
/* Prints text representation of unsigned word (32-bit) in a   */
/* minimum number of characters.                               */
/* number.                                                     */
/* Uses:  PutString                                            */
/***************************************************************/
  /* String for number digits up to 4 billion */
  char String[MAX_WORD_DECIMAL_DIGITS + 1];
  char *StringPtr;

  StringPtr = &(String[MAX_WORD_DECIMAL_DIGITS]);
  *StringPtr = 0;
  
  do {
    /* next least significant digit is remainder of division by 10 */
    *(--StringPtr) = ((char) (num % 10u)) + '0';
    /* rest of number to print */
    num /= 10u; 
  } while (num > 0);
  /* print text digits of number */
  PutStringSB (StringPtr, (MAX_WORD_DECIMAL_DIGITS + 1));
}

void PutNumUH (unsigned short num) {
	PutNumU ((unsigned int) num);
}
