/*********************************************************************/
/* Manages high score board											   									 */
/* Name:	Carson Clarke-Magrab, Fayez Mehdad												 */
/* Date:	4/15/2018																									 */
/* Class:	CMPE 250																									 */
/* Section:	Section 01, Tuesday 2PM - 4PM													   */
/*-------------------------------------------------------------------*/

#include "ScoreBoard.h"
#include "CustomIO.h"

#define NUM_PLACES 	(10)
#define MAX_VAL			(65535)

#define TRUE	(1)
#define FALSE (0)

#define ANSI_COLOR_RED     	"\x1b[31m"
#define ANSI_COLOR_GREEN   	"\x1b[32m"
#define ANSI_COLOR_YELLOW  	"\x1b[33m"
#define ANSI_COLOR_BLUE    	"\x1b[34m"
#define ANSI_COLOR_MAGENTA 	"\x1b[35m"
#define ANSI_COLOR_CYAN    	"\x1b[36m"
#define ANSI_COLOR_RESET   	"\x1b[0m"

#define SCORE_HEADER	"PLACE\t\t\tSCORE"
#define SCORE_LINE 		")\t\t\t"
#define NO_SCORE			"-----"
#define PRESS_ANY			"Press any key to continue"

unsigned short ScoreBoard[NUM_PLACES];

void ClearScores(void) {
/***************************************************************/
/* Clears the high score board          						           */
/***************************************************************/
	for (int i = 0; i < NUM_PLACES; i++) {
		ScoreBoard[i] = MAX_VAL;
	}
}

int AddScore(unsigned short score) {
/***************************************************************/
/* Adds a score to the high score board							           */
/* Returns: 1 if the score is a  high score, 0 if not					 */
/***************************************************************/
	int isHigh = score < ScoreBoard[NUM_PLACES - 1]; 
	int i = NUM_PLACES - 1;
	while (i > 0 && score < ScoreBoard[i - 1]) {
		ScoreBoard[i] = ScoreBoard[i - 1];
		i--;
	}
	ScoreBoard[i] = score;
	return isHigh;
}
void DisplayScores(void) {
/***************************************************************/
/* Clears the high score board          						           */
/***************************************************************/
	PutStringSB(ANSI_COLOR_RED, sizeof(ANSI_COLOR_RED));
	PutStringSB(SCORE_HEADER, sizeof(SCORE_HEADER));
	PutChar('\r');
	PutChar('\n');
	for(int i = 0; i < NUM_PLACES; i++) {
		if (i < 3) {
			PutStringSB(ANSI_COLOR_CYAN, sizeof(ANSI_COLOR_CYAN));
		} else {
			PutStringSB(ANSI_COLOR_YELLOW, sizeof(ANSI_COLOR_YELLOW));
		}
		PutNumU(i + 1);
		PutStringSB(SCORE_LINE, sizeof(SCORE_LINE));
		PutStringSB(ANSI_COLOR_GREEN, sizeof(ANSI_COLOR_GREEN));
		if (ScoreBoard[i] == MAX_VAL) {
			PutStringSB(ANSI_COLOR_RED, sizeof(ANSI_COLOR_RED));
			PutStringSB(NO_SCORE, sizeof(NO_SCORE));
		} else {
			PutNumUH(ScoreBoard[i]);
		}
		PutChar('\r');
		PutChar('\n');
	}
	PutStringSB(ANSI_COLOR_RESET, sizeof(ANSI_COLOR_RESET));
	PutStringSB(PRESS_ANY, sizeof(PRESS_ANY));
}
