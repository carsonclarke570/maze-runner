#include <MKL46Z4.h>

#include "LCD.h"
#include "LED.h"
#include "CustomIO.h"
#include "Timer.h"


#include "Maze.h"
#include "ScoreBoard.h"

/* Boolean Constants */
#define FALSE			(0)
#define TRUE			(1)

/* Clear score board flag */
#define SETUP_FLAG		(1)

int main() {

	__ASM ("CPSID I");
	Init_UART0_IRQ();
	Init_PIT_IRQ();
	InitLCD();
	Init_LED();
	__ASM ("CPSIE I");
	
	char c;
	
	ClearScores();
	
	while(TRUE) {
		InitMaze();
		Count = 0u;
		RunStopWatch = TRUE;
		TurnOnR();
		ClearScreen();
		DisplayMaze();
		while (!GameOver()) {
			c = GetChar();
			if ( c == 'w' || c == 'a' || c == 's' || c == 'd' ) {
				UpdateMaze(c);
			}
		}
		TurnOffR();
		TurnOnG();
		RunStopWatch = FALSE;
		ClearScreen();
		LCD_PutHex(Count / 100);
		AddScore(Count / 100);
		DisplayScores();
		GetChar();
		TurnOffG();
	}
}
