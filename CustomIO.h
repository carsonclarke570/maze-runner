/*********************************************************************/
/* Custom IO functions                                               */
/* Name:	Carson Clarke-Magrab, Fayez Mehdad                         */
/* Date:	4/15/2018                                                  */
/* Class:	CMPE 250                                                   */
/* Section:	Section 01, Tuesday 2PM - 4PM                            */
/*-------------------------------------------------------------------*/

/* I/O Operations */
char GetChar (void);
void PutChar (char Character);
void PutStringSB (char* string, unsigned int capacity);
void PutNumU (unsigned int num);
void PutNumUH (unsigned short num);

/* Initializiations */
void Init_UART0_IRQ(void);
