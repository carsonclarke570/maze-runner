    TTL Assembly Backbone Functions
;****************************************************************
;A series of I/O, drivers and queue functions.
;Name:  Carson Clarke-Magrab
;Date:  4/10/2018
;Class:  CMPE-250
;Section:  Section 01, Tuesday 2PM-4PM
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;Characters
CR          		EQU  	0x0D
LF          		EQU  	0x0A
BS					EQU		0x08
TAB					EQU		0x09
NULL        		EQU		0x00

;Queue Offsets
IN_PTR				EQU		0
OUT_PTR				EQU		4
BUF_STRT			EQU		8
BUF_PAST			EQU		12
BUF_SIZE			EQU		16
NUM_ENQD			EQU		17

;Queue Stuff
Q_BUF_SZ			EQU		4
IRQ_BUF_SZ			EQU		80
	
;Output Characteristics
MAX_WORD_DEC_DIG	EQU		10
MAX_STR_LEN			EQU		80

;Servo
SERVO_POSITIONS     EQU     5
DAC0_STEPS          EQU     4095

TPM_CnV_PWM_DUTY_2ms   EQU     6500 
TPM_CnV_PWM_DUTY_1ms   EQU     3000 

PWM_2ms             EQU     TPM_CnV_PWM_DUTY_2ms 
PWM_1ms             EQU     TPM_CnV_PWM_DUTY_1ms 

PTD5_MUX_GPIO  EQU  (1 << PORT_PCR_MUX_SHIFT)
SET_PTD5_GPIO  EQU  (PORT_PCR_ISF_MASK :OR: PTD5_MUX_GPIO)
PTE29_MUX_GPIO  EQU  (1 << PORT_PCR_MUX_SHIFT)
SET_PTE29_GPIO  EQU  (PORT_PCR_ISF_MASK :OR: PTE29_MUX_GPIO)

POS_RED         EQU  29
POS_GREEN       EQU  5
LED_RED_MASK    EQU  (1 << POS_RED)
LED_GREEN_MASK  EQU  (1 << POS_GREEN)
LED_PORTD_MASK  EQU  LED_GREEN_MASK
LED_PORTE_MASK  EQU  LED_RED_MASK
    
;KL46 SLCD Module
LCD_AR_NORMAL_NO_BLINK               EQU  0x00
;LCD_BPEN0:
;  pins 18 and 19
;    3         2         1
;  120987654321098765432109876543210
;2_000000000000011000000000000000000
;0x   0   0    0   C   0   0   0   0
LCD_BPEN0_19_18                      EQU  0x000C0000
;LCD_BPEN1:
;  pins 52 and 40
;      6         5         4       3
;  321209876543210987654321098765432
;2_000000000000100000000000100000000
;0x   0   0    1   0   0   1   0   0
LCD_BPEN1_52_40                      EQU  0x00100100
LCD_FDCR_NO_FAULT_DETECTION          EQU  0x0000
;LCD_PEN0:
;  pins 7, 8, 10, 11, 17, 18, and 19
;    3         2         1
;  120987654321098765432109876543210
;2_000000000000011100000110110000000
;0x   0   0    0   E   0   D   8   0
LCD_PEN0_19_18_17_11_10_8_7          EQU  0x000E0D80
;LCD_PEN1:
;  pins 37, 38, 40, 52, and 53
;      6         5         4       3
;  321209876543210987654321098765432
;2_000000000001100000000000101100000
;0x   0   0    3   0   0   1   6   0
LCD_PEN1_53_52_40_38_37              EQU  0x00300160
;---------------------------------------------------------------
;FRDM-KL46Z LUMEX LCD S401M16KR
;SLCD pin connections
;  Backplane
;    COM0 pin 1:   PTD0 Alt0=LCD_P40
LCD_PIN_1  EQU  40
;    COM1 pin 2:   PTE4 Alt0=LCD_P52
LCD_PIN_2  EQU  52
;    COM2 pin 3:   PTB23 Alt0=LCD_P19
LCD_PIN_3  EQU  19
;    COM3 pin 4:   PTB22 Alt0=LCD_P18
LCD_PIN_4  EQU  18
;  Frontplane
;    DIG1 pin 5:   PTC17 Alt0=LCD_P37
LCD_DIG1_PIN1  EQU  37
LCD_PIN_5      EQU  37
;    DIG1 pin 6:   PTB21 Alt0=LCD_P17
LCD_DIG1_PIN2  EQU  17
LCD_PIN_6      EQU  17
;    DIG2 pin 7:   PTB7 Alt0=LCD_P7
LCD_DIG2_PIN1  EQU  7
LCD_PIN_7      EQU  7
;    DIG2 pin 8:   PTB8 Alt0=LCD_P8
LCD_DIG2_PIN2  EQU  8
LCD_PIN_8      EQU  8
;    DIG3 pin 9:   PTE5 Alt0=LCD_P53
LCD_DIG3_PIN1  EQU  53
LCD_PIN_9      EQU  53
;    DIG3 pin 10:  PTC18 Alt0=LCD_P38
LCD_DIG3_PIN2  EQU  38
LCD_PIN_10     EQU  38
;    DIG4 pin 11:  PTB10 Alt0=LCD_P10
LCD_DIG4_PIN1  EQU  10
LCD_PIN_11     EQU  10
;    DIG4 pin 12:  PTB11 Alt0=LCD_P11
LCD_DIG4_PIN2  EQU  11
LCD_PIN_12     EQU  11
;All digit segments:  DIG1-DIG4
;  A
;F   B
;  G
;E   C
;  D
;  First register phases
LCD_SEG_D  EQU  0x11
LCD_SEG_E  EQU  0x22
LCD_SEG_G  EQU  0x44
LCD_SEG_F  EQU  0x88
;  Second register phases
LCD_SEG_C  EQU  0x22
LCD_SEG_B  EQU  0x44
LCD_SEG_A  EQU  0x88
;DIG1-DIG3 decimal point to right
;  Second register
LCD_SEG_DECIMAL  EQU  0x11
;"DIG4" colon between DIG2 and DIG3
;  DIG4 Second register
LCD_SEG_COLON  EQU  0x11
LCD_CLEAR  EQU  0x00
 ;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x00
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x0F
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
;>>>>> begin subroutine code <<<<<

GetChar		PROC	{R1 - R14}	
; Gets a character from UART0 via RxQueue. If RxQueue is empty, it waits for a
; character. Dequeue is a critical code section shared with UART0_ISR. 
; Parameters
;	Input:
;		NONE
;	Output:
;		R0:		Character from RxQueue 
;	Modify:
;		R0, APSR

			PUSH	{R1,LR}
			
			LDR		R1, =RxQRecord
GetCharLoop							; do {
			CPSID	I				; 	__asm("CPSID   I") 
			BL		Dequeue			; 	R0 = Dequeue(&RxQRecord)	
			CPSIE	I				; 	__asm("CPSIE   I")
			BCS		GetCharLoop		; } while (APSR[C] == 1)
							
			POP		{R1,PC}
			ENDP
				
PutChar		PROC	{R0 - R14}
; Puts Character to UART0 via TxQueue. If TxQueue is full, it waits for space 
; in the queue. Enqueue is a critical code section shared with UART0_ISR.   
; Parameters
;	Input:
;		R0:		Character to enqueue
;	Output:
;		NONE
;	Modify:
;		APSR

			PUSH	{LR}
			PUSH	{R0 - R1}
			
			LDR		R1, =TxQRecord
PutCharLoop										; do {
			CPSID	I							; 	__asm("CPSID   I")
			BL		Enqueue						; 	Enqueue(c, &TxQRecord) 	
			CPSIE	I							; 	__asm("CPSIE   I")
			BCS		PutCharLoop					; } while (APSR[C] == 1)
			LDR		R0, =UART0_BASE				; UART0->C2 = UART0_C2_TI_RI 
			MOVS	R1, #UART0_C2_TI_RI			;
			STRB	R1, [R0, #UART0_C2_OFFSET]	;
			POP		{R0 - R1}
			POP		{PC}
			ENDP
             
GetStringSB			PROC	{R0 - R14}
; Reads a string from the terminal keyboard and stores it in 
; memory starting at the address were R0 points
; Parameters
;	Input:
;		R0:		Pointer to destination string (StrPtr)
;		R1:		Buffer capacity (BufCap)
;	Output:
;		NONE
;	Modify:
;		ASPR
			PUSH	{LR}
			PUSH	{R0 - R5}
			
			MOVS	R2, R0			; R2 = StrPtr
			MOVS	R3, R1			; R3 = BufCap
			MOVS	R1, #0			; R1 = i = 0
			MOVS	R4, #1			; R4 = NotDone = 1
GetStringLoop						; while (NotDone) {
			CMP		R4, #0			;
			BEQ		GetStringNotBS	;
			BL		GetChar			;	R0 = in = GetChar()
			CMP		R0, #' '		;	if ((in >= ' ') && (in < 0x7F)) {
			BLT		GetStringNotStd	;
			CMP		R0, #0x7F		;
			BGE		GetStringNotStd	;
			SUBS	R5, R3, #1		; 		if (i < (BufCap - 1)) {
			CMP		R1, R5			;
			BGE		GetStringFull	;
			BL		PutChar			; 			PutChar (in);
			STRB	R0, [R2, R1]	; 			*(StrPtr + i) = in
			ADDS	R1, R1, #1		; 			i++;
GetStringFull						;		} 
			B		GetStringNotBS	;
GetStringNotStd						; 	} else if (in == '\r') {
			CMP		R0, #CR			;
			BNE		GetStringNotCR	;
			MOVS	R4, #0			; 		NotDone = FALSE
			B		GetStringDone	;
GetStringNotCR						;	} else if (Character == '\b') {
			CMP		R0, #BS			;
			BNE		GetStringNotBS	;
			CMP		R1, #0			; 		if (i != 0) {
			BEQ		GetStringNotBS	;	
			BL		PutChar			;			PutStringSB ("\b \b", 4)
			MOVS	R5, R0			;
			MOVS	R0, #' '		;
			BL		PutChar			;
			MOVS	R0, R5			;
			BL		PutChar			;
			SUBS	R1, R1, #1		; 			i = i - 1
GetStringNotBS	 					; 	} 
			B 		GetStringLoop	; } 
GetStringDone						;
			MOVS	R4, #0			; StrPtr[i] = NUL
			STRB	R4, [R2, R1]	;
			
			POP		{R0 - R5}
			POP		{PC}
			ENDP
                
PutStringSB			PROC	{R0 - R14}
; Preventing overrun of the buffer capacity, this subroutine
; displays a null-terminated string to the terminal screen
; Parameters
;	Inputs:
;		R0:		Pointer to string (&Str)
;		R1:		Buffer cpacity (BufCap)
;	Outputs:
;		NONE
;	Modify:
;		ASPR
			PUSH	{LR}
			PUSH	{R0 - R5}

			MOVS	R2, R0			; R2 = &Str
			MOVS	R3, R1			; R3 = BufCap
			MOVS	R4, #0			; R4 = i = 0
			LDRB	R1, [R2, R4]	; R1 = c = Str[0]
PutStringLoop						; while ( c != 0 && i < BufCap - 1 )
			CMP		R1, #0			;
			BEQ		PutStringDone	;
			SUBS	R5, R3, #1		;
			CMP		R4, R5			;
			BGE		PutStringDone	;
			MOVS	R0, R1			;	PutChar(c)
			BL		PutChar			;
			ADDS	R4, R4, #1		;	i++
			LDRB	R1, [R2, R4]	;	c = Str[i]
			B		PutStringLoop	; }
PutStringDone

			POP		{R0 - R5}
			POP		{PC}
			ENDP

LCD_PutHex  PROC  {R0-R14}
;****************************************************************
;Displays 4-digit hex value of least halfword in R0 on LCD
;Input:  R0:  Halfword value to display
;Output:  None
;Modifies:  PSR
;****************************************************************
;R0:value to display
;R1:nibble mask
;R2:1)bit shift amount
;   2)working copy of value to display
;R3:loop counter variable
;R4:base address of LCD digit segment setting array
;R5:digit setment settings
;R6:pointer to array of WF addresses for LCD pins
;R7:WF address for current LCD pin
            PUSH  {R1-R7}
            MOVS  R3,#4       ;Digits = 4
            LDR   R6,=LCD_WF_FRONTPLANE_PINS
            LDR   R4,=LCD_DIGITS
            MOVS  R2,#12      ;Bit offset of most significant nibble
            MOVS  R1,#0xF     ;Least nibble mask
LCD_PutHex_Repeat             ;repeat {
            RORS  R0,R0,R2    ;  NextMSN moved to LSN
            MOV   R2,R0       ;  Working copy of value
            ANDS  R2,R2,R1    ;  Nibble value
            LSLS  R2,R2,#1    ;  Halfword array index --> Byte array index
            LDRB  R5,[R4,R2]  ;  Digit pin 1 segment value
            LDM   R6!,{R7}    ;  Address of digit pin1's WF register
            ADDS  R2,R2,#1    ;  Next byte index
            STRB  R5,[R7,#0]  ;  Write to LCD_WF[LCD_DIGn_PIN1]
            LDM   R6!,{R7}    ;  Address of digit pin2's WF register
            LDRB  R5,[R4,R2]  ;  Digit pin 2 segment value
            STRB  R5,[R7,#0]  ;  Write to LCD_WF[LCD_DIGn_PIN2]
            MOVS  R2,#28      ;  Bit offset of next most significant nibble
            SUBS  R3,R3,#1    ;  Digits--
                              ;} until (Digits == 0)
            BNE   LCD_PutHex_Repeat
            POP   {R1-R7}
            BX    LR
            ENDP
				
InitQueue			PROC	{R0 - R14}
; Initializes the queue reocrd structire at the address 
; in R1 for the enpty queue buffer at the address in R0
; of size, (i.e., character capacity), given in R2.
; Parameters
;	Inputs:
;		R0:		empty queue buffer address (&Q_BUF)
;		R1:		queue record structure address (&Q_REC)
;		R2:		character capacity (Q_BUF_SZ)
;	Output:
;		NONE
;	Modify:
;		ASPR
			PUSH	{R0}

			STR		R0, [R1, #IN_PTR]	; Q_REC[IN_PTR] = &Q_BUF
			STR		R0, [R1, #OUT_PTR]	; Q_REC[OUT_PTR] = &Q_BUF
			STR		R0, [R1, #BUF_STRT]	; Q_REC[BUF_STRT] = &Q_BUF
			ADDS	R0, R0, R2			; Q_REC[BUF_PAST] = &Q_BUF + Q_BUF_SZ
			STR		R0, [R1, #BUF_PAST]	;
			STRB	R2, [R1, #BUF_SIZE] ; Q_REC[BUF_SIZE] = Q_BUF_SZ
			MOVS	R0, #0				; Q_REC[NUM_ENQD] = 0
			STRB	R0, [R1, #NUM_ENQD]	;
			
			POP		{R0}
			BX		LR
			ENDP
				
Dequeue				PROC	{R1 - R14}
; Attempts to get a character from the queue whose record 
; structure’s address is in R1:  if the queue is not empty,
; dequeues a single character from the queue to R0, and returns 
; with the PSRC bit cleared, (i.e., 0), to report dequeue 
; success; otherwise, returns with the PSRC bit set, (i.e., 1) 
; to report dequeue failure.
; Parameters
;	Input:
;		R1:		queue record structure address (&Q_REC))
;	Output:
;		R0:		dequeued character 
;	Modify:
;		ASPR
			PUSH	{R1 - R4}
			
			LDRB	R2, [R1, #NUM_ENQD]	; if (Q_REC[NUM_ENQD] > 0) {
			CMP		R2, #0				;
			BEQ		DQEmpty				;
			LDR		R4, [R1, #OUT_PTR]  ;	c = Q_REC[OUT_PTR]*
			LDRB	R0, [R4, #0]		;
			SUBS	R2, R2, #1			;	Q_REC[NUM_ENQD]--
			STRB	R2, [R1, #NUM_ENQD]	;
			ADDS	R4, R4, #1			;	Q_REC[OUT_PTR]++
			STR		R4, [R1, #OUT_PTR]	;
			LDR		R3, [R1, #BUF_PAST]	;	if (Q_REC[OUT_PTR] >= Q_REC[BUF_PAST]) {
			CMP		R4, R3				;
			BLO		DQThen				;
			LDR		R2, [R1, #BUF_STRT]	;		Q_REC[OUT_PTR] = Q_REC[BUF_STRT]
			STR		R2, [R1, #OUT_PTR]	;
DQThen									;	}
			MOVS	R3, #0x20			;	APSR[C] = 0
			LSLS	R3, R3, #24			;
			BICS	R2, R2, R3			;
            MSR     APSR, R2            ;
			B		DQDone				;
DQEmpty									; } else {
			MOVS	R3, #0x20			;	ASPR[C] = 1
			LSLS	R3, R3, #24			;
			ORRS	R2, R2, R3			;
            MSR     APSR, R2            ;
DQDone									; }
			POP		{R1 - R4}
			BX		LR
			ENDP
				
Enqueue				PROC	{R0 - R14}
; Attempts to put a character in the queue whose queue record structure’s
; address is in R1—if the queue is not full, enqueues the single character 
; from R0 to the queue, and returns with the PSRC cleared to report enqueue 
; success; otherwise, returns with the PSRC bit set to report enqueue failure.
; Parameters
;	Input:
;		R0:		character to enqueue (c)
;		R1:		queue record structure (Q_REC)
;	Output:
;		NONE
;	Modify:
;		ASPR
			PUSH	{R0 - R3}
			LDRB	R2, [R1, #NUM_ENQD]	; if (Q_REC[NUM_ENQD] < Q_REC[BUF_SIZE]) {
			LDRB	R3, [R1, #BUF_SIZE]	;
			CMP		R2, R3				;
			BGE		NQFull				;
			LDR		R3,	[R1, #IN_PTR]	;	Q_REC[IN_PTR]* = c
			STRB	R0,	[R3, #0]		;
			ADDS	R2, R2, #1			; 	Q_REC[NUM_ENQD]++
			STRB	R2, [R1, #NUM_ENQD]	;
			ADDS	R3, R3, #1			; 	Q_REC[IN_PTR]++
			STR		R3, [R1, #IN_PTR]	;
			LDR		R2, [R1, #BUF_PAST]	; 	if (Q_REC[IN_PTR] >= Q_REC[BUF_PAST]) {
			CMP		R3, R2				;
			BLO		NQThen				;
			LDR		R2, [R1, #BUF_STRT]	;		Q_REC[IN_PTR] = Q_REC[BUF_STRT]
			STR		R2, [R1, #IN_PTR]	;
NQThen									;	}
			MOVS	R3, #0x20			;	APSR[C] = 0
			LSLS	R3, R3, #24			;
			BICS	R2, R2, R3			;
            MSR     APSR, R2            ;
			B		NQDone				;
NQFull									; } else {
			MOVS	R3, #0x20			;	ASPR[C] = 1
			LSLS	R3, R3, #24			;
			ORRS	R2, R2, R3			;
            MSR     APSR, R2            ;
NQDone									; }

			POP		{R0 - R3}
            BX      LR
			ENDP    

TurnOffR	PROC	{R0 - R14}
			
			PUSH	{R0 - R1}

			LDR  	R0, =FGPIOE_BASE
			LDR  	R1, =LED_RED_MASK
			STR  	R1, [R0,#GPIO_PSOR_OFFSET]

			POP		{R0 - R1}
			BX		LR
			ENDP
			
TurnOffG	PROC	{R0 - R14}
			
			PUSH	{R0 - R1}

			LDR  	R0, =FGPIOD_BASE
			LDR  	R1, =LED_GREEN_MASK
			STR  	R1, [R0,#GPIO_PSOR_OFFSET]

			POP		{R0 - R1}
			BX		LR
			ENDP
			
TurnOnR		PROC	{R0 - R14}
			
			PUSH	{R0 - R1}

			LDR  	R0, =FGPIOE_BASE
			LDR  	R1, =LED_RED_MASK
			STR  	R1, [R0,#GPIO_PCOR_OFFSET]

			POP		{R0 - R1}
			BX		LR
			ENDP
			
TurnOnG		PROC	{R0 - R14}
			
			PUSH	{R0 - R1}

			LDR  	R0, =FGPIOD_BASE
			LDR  	R1, =LED_GREEN_MASK
			STR  	R1, [R0,#GPIO_PCOR_OFFSET]

			POP		{R0 - R1}
			BX		LR
			ENDP
				
UART0_IRQHandler
UART0_ISR	PROC	{R0 - R14}
; Handles RxIRQ (RDRF) and TxIRQ (TDRE). If TxIRQ but TxQueue is empty, TxIRQ is 
; disabled. If TxQueue is full, it waits for space in the queue. Enqueue is a critical
; code section shared with UART0_ISR.
; Parameters
;	Input:
;		NONE
;	Output:
;		NONE
;	Modify:
;		APSR
			CPSID	I							; __asm("CPSID   I");
			PUSH	{LR}
			
			LDR		R0, =UART0_BASE				; if (UART0->C2 & UART0_C2_TIE_MASK) {
			MOVS	R1, #UART0_C2_TIE_MASK		;
			LDRB	R2, [R0, #UART0_C2_OFFSET]	;
			TST		R1, R2						;
			BEQ		ISR_Outer					;
			MOVS	R1, #UART0_S1_TDRE_MASK		; 	if (UART0->S1 & UART0_S1_TDRE_MASK) {
			LDRB	R2, [R0, #UART0_S1_OFFSET]	;
			TST 	R1, R2					;
			BEQ		ISR_Outer					;
			LDR		R1, =TxQRecord				; 		R2 = Dequeue(&TxQRecord)
			BL		Dequeue						;
			BCC		ISR_Inner					;		if (APSR[C] == 1) {
			LDR		R0, =UART0_BASE				; 			UART0->C2 = UART0_C2_T_RI
			MOVS	R1, #UART0_C2_T_RI			;
			STRB	R1, [R0, #UART0_C2_OFFSET]	;
			B		ISR_Outer
ISR_Inner										;		}
			MOVS	R2, R0						;
			LDR		R0, =UART0_BASE				;		UART0->D = R2
			STRB	R2, [R0, #UART0_D_OFFSET]	;
ISR_Outer										; }
			LDR		R0, =UART0_BASE				; if (UART0->S1 & UART0_S1_RDRF_MASK) {
			MOVS	R1, #UART0_S1_RDRF_MASK		;
			LDRB	R2, [R0, #UART0_S1_OFFSET]	;
			ANDS	R2, R2, R1					;
			BEQ		ISR_Other					;
			LDRB	R0, [R0, #UART0_D_OFFSET]	;	Enqueue (UART0->D, &RxQRecord)
			LDR		R1, =RxQRecord				;
			BL		Enqueue						;
ISR_Other										; }
			CPSIE	I							; __asm("CPSIE   I");  /* unmask interrupts */
		
			POP		{PC}
			ENDP

PIT_IRQHandler
PIT_ISR			PROC	{R0 - R14}
; PIT Interrupt Service Routine          
; Handles PIT Timer 0 interrupt. Global variable RunStopWatch is IRQ count
; (timing) enable.
; Parameters
;	Input:
;		NONE
;	Output:
;		NONE
;	Modify:
;		NONE

			CPSID	I							; __asm("CPSID   I");  /* mask interrupts */
			LDR		R0, =RunStopWatch			; if (RunStopWatch) {
			LDRB	R0, [R0, #0]				;
			CMP		R0, #0						;
			BEQ		PIT_ISRElse					;
			LDR		R0, =Count					; 	Count++;
			LDR		R1, [R0, #0]				;
			ADDS	R1, R1, #1					;
			STR		R1, [R0, #0]				;
PIT_ISRElse										; }
			; clear PIT timer 0 interrupt flag 
			LDR		R0, =PIT_CH0_BASE			; PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK
			LDR		R1, =PIT_TFLG_TIF_MASK		;
			STR		R1, [R0, #PIT_TFLG_OFFSET]	;
			CPSIE	I							; __asm("CPSIE   I");  /* unmask interrupts */
			BX		LR
			ENDP
				
Init_UART0_IRQ	PROC	{R0 - R14}
; Initializes UART0 for 9600 baud and 8N1 format. Initializes circular FIFO queues for
; Rx and Tx
; Parameters
;	Input:
;		NONE
;	Output:
;		NONE
;	Modify:
;		APSR

			PUSH	{LR}
			PUSH	{R0 - R3}

			LDR		R0, =RxQBuffer							; Init_Queue(&RxQBuffer, &RxQRecord, X_BUF_SZ)
			LDR		R1, =RxQRecord							;		 
			MOVS	R2, #IRQ_BUF_SZ							;
			BL		InitQueue								;
			LDR		R0, =TxQBuffer							; Init_Queue(&TxQBuffer, &TxQRecord, X_BUF_SZ)
			LDR		R1, =TxQRecord							; 
			MOVS	R2, #IRQ_BUF_SZ							;
			BL		InitQueue								;

			; Select MCGPLLCLK / 2 as UART0 clock source 
			LDR		R0, =SIM_SOPT2							; SIM->SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK
			LDR		R1, =SIM_SOPT2_UART0SRC_MASK			;
			LDR		R2, [R0, #0]							;
			BICS	R2, R2, R1								;
			LDR		R1, =SIM_SOPT2_UART0_MCGPLLCLK_DIV2		; SIM->SOPT2 |= SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS	R2, R2, R1								;
			STR		R2, [R0, #0]							;
			
			; Set UART0 for external connection 
			LDR		R0, =SIM_SOPT5							; SIM->SOPT5 &= ~SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR		R1, =SIM_SOPT5_UART0_EXTERN_MASK_CLEAR	;
			LDR		R2, [R0, #0]							;
			BICS	R2, R2, R1								;
			STR		R2, [R0, #0]							;
 
			; Enable UART0 module clock 
			LDR		R0, =SIM_SCGC4							; SIM->SCGC4 |= SIM_SCGC4_UART0_MASK
			LDR		R1, =SIM_SCGC4_UART0_MASK				;
			LDR		R2, [R0, #0]							;
			ORRS	R2, R2, R1								;
			STR		R2, [R0, #0]							;
			
			; Enable PORT A module clock 
			LDR		R0, =SIM_SCGC5							; SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK
			LDR		R1, =SIM_SCGC5_PORTA_MASK				;
			LDR		R2, [R0, #0]							;
			ORRS	R2, R2, R1								;
			STR		R2, [R0, #0]							;
  
			; Some OpenSDA applications provide a virtual serial port 
			; through the OpenSDA USB connection using PTA1 and PTA2  
			; Select PORT A Pin 1 (J1 Pin 02) for UART0 RX 
			LDR		R0, =PORTA_PCR1							; PORTA->PCR[1] = PORT_PCR_SET_PTA1_UART0_RX
			LDR		R1, =PORT_PCR_SET_PTA1_UART0_RX			;
			STR		R1, [R0, #0]							;
  
			; Select PORT A Pin 2 (J1 Pin 04) for UART0 TX 
			LDR		R0, =PORTA_PCR2							; PORTA->PCR[2] = PORT_PCR_SET_PTA2_UART0_TX
			LDR		R1, =PORT_PCR_SET_PTA2_UART0_TX			;
			STR		R1, [R0, #0]							;
			
			; Disable UART0 
			LDR		R0, =UART0_BASE							; UART0->C2 &= ~UART0_C2_T_R;
			MOVS	R1, #UART0_C2_T_R						;
			LDRB	R2, [R0, #UART0_C2_OFFSET]				;
			BICS	R2, R2, R1								;
			STRB	R2, [R0, #UART0_C2_OFFSET]				;

			; Set UART0 interrupt priority 
			LDR		R0, =UART0_IPR							; NVIC->IP[UART0_IPR_REGISTER] |= NVIC_IPR_UART0_MASK;
			;LDR		R1, =NVIC_IPR_UART0_MASK			;
			LDR		R2, =NVIC_IPR_UART0_PRI_3				;
			LDR		R3, [R0, #0]							;
			;BICS	R3, R3, R1								;
			ORRS	R3, R3, R2								;
			STR		R3, [R0, #0]							;
			
			; Clear any pending UART0 interrupts 
			LDR		R0, =NVIC_ICPR							; NVIC->ICPR[0] = NVIC_ICPR_UART0_MASK
			LDR		R1, =NVIC_ICPR_UART0_MASK				;
			STR		R1, [R0, #0]							;
			
			; Unmask UART0 interrupts 
			LDR		R0, =NVIC_ISER							; NVIC->ISER[0] = NVIC_ISER_UART0_MASK
			LDR		R1, =NVIC_ISER_UART0_MASK				;
			STR		R1, [R0, #0]							;
			
			; Set for 9600 baud from 96MHz PLL clock 
			LDR		R0, =UART0_BASE							; UART0->BDH = UART0_BDH_9600
			MOVS	R1, #UART0_BDH_9600						;
			STRB	R1, [R0, #UART0_BDH_OFFSET]				;
			MOVS	R1, #UART0_BDL_9600						; UART0->BDL = UART0_BDL_9600
			STRB	R1, [R0, #UART0_BDL_OFFSET]				;
			MOVS	R1, #UART0_C1_8N1						; UART0->C1 = UART0_C1_8N1
			STRB	R1, [R0, #UART0_C1_OFFSET]				;
			MOVS	R1, #UART0_C3_NO_TXINV					; UART0->C3 = UART0_C3_NO_TXINV
			STRB	R1, [R0, #UART0_C3_OFFSET]				;
			MOVS	R1, #UART0_C4_NO_MATCH_OSR_16			; UART0->C4 = UART0_C4_NO_MATCH_OSR_16
			STRB	R1, [R0, #UART0_C4_OFFSET]				;
			MOVS	R1, #UART0_C5_NO_DMA_SSR_SYNC			; UART0->C5 = UART0_C5_NO_DMA_SSR_SYNC
			STRB	R1, [R0, #UART0_C5_OFFSET]				;
			MOVS	R1, #UART0_S1_CLEAR_FLAGS				; UART0->S1 = UART0_S1_CLEAR_FLAGS
			STRB	R1, [R0, #UART0_S1_OFFSET]				;
			MOVS	R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS	; UART0->S2 = UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB	R1, [R0, #UART0_S2_OFFSET]				;
			MOVS	R1, #UART0_C2_T_RI						; UART0->C2 = UART0_C2_T_RI  /* enable UART0 */		
			STRB	R1, [R0, #UART0_C2_OFFSET]				;
			
			POP		{R0 - R3}
			POP		{PC}
			ENDP
				
InitLCD     PROC  {R0-R14}
;****************************************************************
;Enables segment LCD (SLCD) display using the following ports:
;  COM0 pin 1:   PTD0 Alt0=LCD_P40
;  COM1 pin 2:   PTE4 Alt0=LCD_P52
;  COM2 pin 3:   PTB23 Alt0=LCD_P19
;  COM3 pin 4:   PTB22 Alt0=LCD_P18
;  DIG1 pin 5:   PTC17 Alt0=LCD_P37
;  DIG1 pin 6:   PTB21 Alt0=LCD_P17
;  DIG2 pin 7:   PTB7 Alt0=LCD_P7
;  DIG2 pin 8:   PTB8 Alt0=LCD_P8
;  DIG3 pin 9:   PTE5 Alt0=LCD_P53
;  DIG3 pin 10:  PTC18 Alt0=LCD_P38
;  DIG4 pin 11:  PTB10 Alt0=LCD_P10
;  DIG4 pin 12:  PTB11 Alt0=LCD_P11
;Input:  None
;Output:  None
;Modifies:  PSR
;****************************************************************
            ;Preserve registers used
            PUSH    {R0-R7}
            ;Select 32-kHz clock for SLCD
            LDR     R0,=SIM_SOPT1
            LDR     R1,=SIM_SOPT1_OSC32KSEL_MASK
            LDR     R2,[R0,#0]
            BICS    R2,R2,R1
            STR     R2,[R0,#0]
            ;Enable SLCD and PORTs B, C, D, and E module clocks
            LDR     R0,=SIM_SCGC5
            LDR     R1,=(SIM_SCGC5_SLCD_MASK :OR: \
                         SIM_SCGC5_PORTB_MASK :OR: \
                         SIM_SCGC5_PORTC_MASK :OR: \
                         SIM_SCGC5_PORTD_MASK :OR: \
                         SIM_SCGC5_PORTE_MASK)
            LDR     R2,[R0,#0]
            ORRS    R2,R2,R1
            STR     R2,[R0,#0]
            ;Select PORT B Pin 7 for SLCD pin 7 (MUX select 0)
            LDR     R0,=PORTB_BASE
            LDR     R1,=(PORT_PCR_ISF_MASK :OR: \
                         PORT_PCR_MUX_SELECT_0_MASK)
            STR     R1,[R0,#PORTB_PCR7_OFFSET]
            ;Select PORT B Pin 8 for SLCD pin 8 (MUX select 0)
            STR     R1,[R0,#PORTB_PCR8_OFFSET]
            ;Select PORT B Pin 10 for SLCD pin 11 (MUX select 0)
            STR     R1,[R0,#PORTB_PCR10_OFFSET]
            ;Select PORT B Pin 11 for SLCD pin 12 (MUX select 0)
            STR     R1,[R0,#PORTB_PCR11_OFFSET]
            ;Select PORT B Pin 21 for SLCD pin 6 (MUX select 0)
            STR     R1,[R0,#PORTB_PCR21_OFFSET]
            ;Select PORT B Pin 22 for SLCD pin 4 (MUX select 0)
            STR     R1,[R0,#PORTB_PCR22_OFFSET]
            ;Select PORT B Pin 23 for SLCD pin 3 (MUX select 0)
            STR     R1,[R0,#PORTB_PCR23_OFFSET]
            ;Select PORT C Pin 17 for SLCD pin 5 (MUX select 0)
            LDR     R0,=PORTC_BASE
            STR     R1,[R0,#PORTC_PCR17_OFFSET]
            ;Select PORT C Pin 18 for SLCD pin 10 (MUX select 0)
            STR     R1,[R0,#PORTC_PCR18_OFFSET]
            ;Select PORT D Pin 0 for SLCD pin 1 (MUX select 0)
            LDR     R0,=PORTD_BASE
            STR     R1,[R0,#PORTD_PCR0_OFFSET]
            ;Select PORT E Pin 4 for SLCD pin 2 (MUX select 0)
            LDR     R0,=PORTE_BASE
            STR     R1,[R0,#PORTE_PCR4_OFFSET]
            ;Select PORT E Pin 5 for SLCD pin 9 (MUX select 0)
            STR     R1,[R0,#PORTE_PCR4_OFFSET]
            ;Disable SLCD
            LDR     R0,=LCD_BASE
            MOVS    R1,#LCD_GCR_LCDEN_MASK
            LDR     R2,[R0,#LCD_GCR_OFFSET]
            BICS    R2,R2,R1
            STR     R2,[R0,#LCD_GCR_OFFSET]
            ;Set SLCD for charge pump, high capacitance,
            ;  pad safe, 32-kHz internal clock (MCGIRCLK),
            ;  and 32-Hz frame frequency
            ;LCD_GCR:  CPSEL=1; LADJ=3; PADSAFE=1; FFR=1; SOURCE=1;
            ;          LCLK=4;DUTY=3
            LDR     R1,=(LCD_GCR_CPSEL_MASK :OR: \
                         (3 << LCD_GCR_LADJ_SHIFT) :OR: \
                         LCD_GCR_PADSAFE_MASK :OR: \
                         LCD_GCR_FFR_MASK :OR: \
                         LCD_GCR_SOURCE_MASK :OR: \
                         (4 << LCD_GCR_LCLK_SHIFT) :OR \
                         (3 << LCD_GCR_DUTY_SHIFT))
            STR     R1,[R0,#LCD_GCR_OFFSET]
            ;Set SLCD for no blink
            MOVS    R1,#LCD_AR_NORMAL_NO_BLINK
            STR     R1,[R0,#LCD_AR_OFFSET]
            ;Set SLCD for no fault detection
            LDR     R1,=LCD_FDCR_NO_FAULT_DETECTION 
            STR     R1,[R0,#LCD_FDCR_OFFSET]
            ;Enable pins 7, 8, 10, 11, 17, 18, and 19
            ;Enable pins 37, 38, 40, 52, and 53
            LDR     R1,=LCD_PEN0_19_18_17_11_10_8_7
            LDR     R2,=LCD_PEN1_53_52_40_38_37
            STR     R1,[R0,#LCD_PENL_OFFSET]
            STR     R2,[R0,#LCD_PENH_OFFSET]
            ;Enable backplane for COM0-COM3 
            ;  pins 18, 19, 40, and 52
            LDR     R1,=LCD_BPEN0_19_18
            LDR     R2,=LCD_BPEN1_52_40
            STR     R1,[R0,#LCD_BPENL_OFFSET]
            STR     R2,[R0,#LCD_BPENH_OFFSET]
            ;Configure waveform registers (64 bytes = 16 words = 4 quadwords)
            LDR     R1,=LCD_WF            ;LCD_WF_Quadword_Ptr
            LDR     R2,=LCD_WF_Config     ;LCD_WF_Config_QuadwordPtr
            MOVS    R3,#4                 ;LCV = 4
InitLCD_WF_Repeat                         ;repeat {
            LDM     R2!,{R4-R7}           ;  *LCD_WF_Quadword_Ptr++ =
            STM     R1!,{R4-R7}           ;    *LCD_WF_Config_Quadword_Ptr++
            SUBS    R3,#1                 ;  LCV--
            BNE     InitLCD_WF_Repeat     ;} until (LCV == 0)
            ;Disable SCLD padsafe and enable SLCD
            MOVS    R1,#LCD_GCR_LCDEN_MASK
            LDR     R2,[R0,#LCD_GCR_OFFSET]
            LDR     R3,=LCD_GCR_PADSAFE_MASK
            ORRS    R2,R2,R1
            BICS    R2,R2,R3
            STR     R2,[R0,#LCD_GCR_OFFSET]
            ;Restore registers used
            POP     {R0-R7}
            BX      LR
            ENDP
				
Init_PIT_IRQ	PROC	{R0 - R14}
; Initializes PIT for interrupt every 0.01 s. Initializes RunStopWatch to 0 to
; disable timing. Initializes Count to 0 for number of PIT IRQs counted.
; Parameters
;	Input:
;		NONE
;	Output:
;		NONE
;	Modify:
;		APSR
			PUSH 	{R0 - R3}
		
			; Initialize PIT timing variables
			MOVS	R0, #0						; RunStopWatch = (uint8_t) 0u; 
			LDR		R1, =RunStopWatch			;
			STRB	R0, [R1, #0]				;
			LDR		R1, =Count					; Count = (uint32_t) 0u; /* No IRQ periods measured */
			STR		R0, [R1, #0]				;
  
			; Enable PIT module clock
			LDR		R0, =SIM_SCGC6				; SIM->SCGC6 |= SIM_SCGC6_PIT_MASK
			LDR		R1, =SIM_SCGC6_PIT_MASK		; 
			LDR		R2, [R0, #0]				;
			ORRS	R2, R2, R1					;
			STR		R2, [R0, #0]				;
			
			; Disable PIT Timer 0
			LDR		R0, =PIT_CH0_BASE			; PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK
			LDR		R1, =PIT_TCTRL_TEN_MASK		;
			LDR		R2, [R0, #PIT_TCTRL_OFFSET]	;
			BICS	R2, R2, R1					;
			STR		R2, [R0, #PIT_TCTRL_OFFSET]	;
			
			; Set PIT interrupt priority to 0 (highest) 
			LDR		R0, =PIT_IPR				;	NVIC->IP[PIT_IPR_REGISTER] &= NVIC_IPR_PIT_MASK
			LDR		R1, =NVIC_IPR_PIT_MASK		;
			;LDR		R2, =NVIC_IPR_PIT_PRI_0		;
			LDR		R3, [R0, #0]				;
			BICS	R3, R3, R1					;
			;ORRS	R3, R3, R2					;
			STR		R3, [R0, #0]				;
			
			; Clear any pending PIT interrupts 
			LDR		R0, =NVIC_ICPR				; NVIC->ICPR[0] = NVIC_ICPR_PIT_MASK
			LDR		R1, =NVIC_ICPR_PIT_MASK		;
			STR		R1, [R0, #0]				;
			
			; Unmask UART0 interrupts 
			LDR		R0, =NVIC_ISER				; NVIC->ISER[0] = NVIC_ISER_PIT_MASK
			LDR		R1, =NVIC_ISER_PIT_MASK		;
			STR		R1, [R0, #0]				;
			
			; Enable PIT timer module and set to stop in debug mode
			LDR		R0, =PIT_BASE				; PIT->MCR = PIT_MCR_EN_FRZ;
			LDR		R1, =PIT_MCR_EN_FRZ			;
			STR		R1, [R0, #PIT_MCR_OFFSET]	;
			
			; Set PIT Timer 0 period for 0.01 s 
			LDR		R0, =PIT_CH0_BASE			; PIT->CHANNEL[0].LDVAL = PIT_LDVAL_10ms;
			LDR		R1, =PIT_LDVAL_10ms			;
			STR		R1, [R0, #PIT_LDVAL_OFFSET]	;
  
			; Enable PIT Timer 0 and interrupt 
			LDR		R1, =PIT_TCTRL_CH_IE		; PIT->CHANNEL[0].TCTRL = PIT_TCTRL_CH_IE;
			STR		R1, [R0, #PIT_TCTRL_OFFSET]	;

			POP 	{R0 - R3}
			BX		LR
			ENDP
			
Init_LED	PROC	{R0 - R14}

			PUSH	{R0 - R2}
			; enable clock for PORT D and E modules
			LDR     R0, =SIM_SCGC5
			LDR     R1, =(SIM_SCGC5_PORTD_MASK :OR: SIM_SCGC5_PORTE_MASK)
			LDR     R2, [R0, #0]
			ORRS    R2, R2, R1
			STR     R2, [R0, #0]
			
			;Select PORT E Pin 29 for GPIO to red LED
			LDR     R0, =PORTE_BASE
			LDR     R1, =SET_PTE29_GPIO
			STR     R1, [R0, #PORTE_PCR29_OFFSET]
			;Select PORT D Pin 5 for GPIO to green LED
			LDR     R0, =PORTD_BASE
			LDR     R1, =SET_PTD5_GPIO
			STR     R1, [R0, #PORTD_PCR5_OFFSET]
			
			LDR  	R0, =FGPIOD_BASE
			LDR  	R1, =LED_PORTD_MASK
			STR  	R1, [R0,#GPIO_PDDR_OFFSET]
			LDR  	R0, =FGPIOE_BASE
			LDR  	R1, =LED_PORTE_MASK
			STR  	R1, [R0,#GPIO_PDDR_OFFSET]
			
			POP		{R0 - R2}
			BX		LR
			ENDP
				
;>>>>>   end subroutine code <<<<<
            ALIGN
            EXPORT  GetChar
			EXPORT	PutChar
			EXPORT	PutStringSB
            EXPORT  Init_UART0_IRQ
			EXPORT	Init_PIT_IRQ
            EXPORT  UART0_IRQHandler
			EXPORT	PIT_IRQHandler
			EXPORT	InitLCD
			EXPORT	LCD_PutHex
			EXPORT	TurnOffR
			EXPORT	TurnOffG
			EXPORT	TurnOnR
			EXPORT	TurnOnG
			EXPORT	Init_LED
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<   
LCD_WF_Config
;Each pin:  2_HGFEDCBA phase enable
            DCB  0x00  ;Pin0
            DCB  0x00  ;Pin1
            DCB  0x00  ;Pin2
            DCB  0x00  ;Pin3
            DCB  0x00  ;Pin4
            DCB  0x00  ;Pin5
            DCB  0x00  ;Pin6
            DCB  0x00  ;Pin7
            DCB  0x00  ;Pin8
            DCB  0x00  ;Pin9
            DCB  0x00  ;Pin10
            DCB  0x00  ;Pin11
            DCB  0x00  ;Pin12
            DCB  0x00  ;Pin13
            DCB  0x00  ;Pin14
            DCB  0x00  ;Pin15
            DCB  0x00  ;Pin16
            DCB  0x00  ;Pin17
            DCB  (LCD_WF_D_MASK :OR: LCD_WF_H_MASK)  ;Pin18=COM3:D,H
            DCB  (LCD_WF_C_MASK :OR: LCD_WF_G_MASK)  ;Pin19=COM2:C,G
            DCB  0x00  ;Pin20
            DCB  0x00  ;Pin21
            DCB  0x00  ;Pin22
            DCB  0x00  ;Pin23
            DCB  0x00  ;Pin24
            DCB  0x00  ;Pin25
            DCB  0x00  ;Pin26
            DCB  0x00  ;Pin27
            DCB  0x00  ;Pin28
            DCB  0x00  ;Pin29
            DCB  0x00  ;Pin30
            DCB  0x00  ;Pin31
            DCB  0x00  ;Pin32
            DCB  0x00  ;Pin33
            DCB  0x00  ;Pin34
            DCB  0x00  ;Pin35
            DCB  0x00  ;Pin36
            DCB  0x00  ;Pin37
            DCB  0x00  ;Pin38
            DCB  0x00  ;Pin39
            DCB  (LCD_WF_A_MASK :OR: LCD_WF_E_MASK)  ;Pin40=COM0:A,E
            DCB  0x00  ;Pin41
            DCB  0x00  ;Pin42
            DCB  0x00  ;Pin43
            DCB  0x00  ;Pin44
            DCB  0x00  ;Pin45
            DCB  0x00  ;Pin46
            DCB  0x00  ;Pin47
            DCB  0x00  ;Pin48
            DCB  0x00  ;Pin49
            DCB  0x00  ;Pin50
            DCB  0x00  ;Pin51
            DCB  (LCD_WF_B_MASK :OR: LCD_WF_F_MASK)  ;Pin52=COM1:B,F
            DCB  0x00  ;Pin53
            DCB  0x00  ;Pin54
            DCB  0x00  ;Pin55
            DCB  0x00  ;Pin56
            DCB  0x00  ;Pin57
            DCB  0x00  ;Pin58
            DCB  0x00  ;Pin59
            DCB  0x00  ;Pin60
            DCB  0x00  ;Pin61
            DCB  0x00  ;Pin62
            DCB  0x00  ;Pin63
            ALIGN
				
;SLCD pin connections
LCD_WF_Pins
LCD_WF_BACKPLANE_PINS
           DCD  (LCD_WF + LCD_PIN_1)
           DCD  (LCD_WF + LCD_PIN_2)
           DCD  (LCD_WF + LCD_PIN_3)
           DCD  (LCD_WF + LCD_PIN_4)
LCD_WF_FRONTPLANE_PINS
LCD_WF_FRONTPLANE_PINS_DIG1
           DCD  (LCD_WF + LCD_PIN_5)
           DCD  (LCD_WF + LCD_PIN_6)
LCD_WF_FRONTPLANE_PINS_DIG2
           DCD  (LCD_WF + LCD_PIN_7)
           DCD  (LCD_WF + LCD_PIN_8)
LCD_WF_FRONTPLANE_PINS_DIG3
           DCD  (LCD_WF + LCD_PIN_9)
           DCD  (LCD_WF + LCD_PIN_10)
LCD_WF_FRONTPLANE_PINS_DIG4
           DCD  (LCD_WF + LCD_PIN_11)
           DCD  (LCD_WF + LCD_PIN_12)
;LCD segments
;  A
;F   B
;  G
;E   C
;  D
LCD_DIGITS
LCD_0
LCD_0_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_F)
LCD_0_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B :OR: LCD_SEG_C)
LCD_1
LCD_1_PIN1   DCB  LCD_CLEAR
LCD_1_PIN2   DCB  (LCD_SEG_B :OR: LCD_SEG_C)
LCD_2
LCD_2_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_G)
LCD_2_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B)
LCD_3
LCD_3_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_G)
LCD_3_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B :OR: LCD_SEG_C)
LCD_4
LCD_4_PIN1   DCB  (LCD_SEG_F :OR: LCD_SEG_G)
LCD_4_PIN2   DCB  (LCD_SEG_B :OR: LCD_SEG_C)
LCD_5
LCD_5_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_5_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_C)
LCD_6
LCD_6_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_6_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_C)
LCD_7
LCD_7_PIN1   DCB  LCD_CLEAR
LCD_7_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B :OR: LCD_SEG_C)
LCD_8
LCD_8_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_8_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B :OR: LCD_SEG_C)
LCD_9
LCD_9_PIN1   DCB  (LCD_SEG_F :OR: LCD_SEG_G)
LCD_9_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B :OR: LCD_SEG_C)
LCD_A
LCD_A_PIN1   DCB  (LCD_SEG_E :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_A_PIN2   DCB  (LCD_SEG_A :OR: LCD_SEG_B :OR: LCD_SEG_C)
LCD_B
LCD_B_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_B_PIN2   DCB  LCD_SEG_C
LCD_C
LCD_C_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_F)
LCD_C_PIN2   DCB  LCD_SEG_A
LCD_D
LCD_D_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_G)
LCD_D_PIN2   DCB  (LCD_SEG_B :OR: LCD_SEG_C)
LCD_E
LCD_E_PIN1   DCB  (LCD_SEG_D :OR: LCD_SEG_E :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_E_PIN2   DCB  LCD_SEG_A
LCD_F
LCD_F_PIN1   DCB  (LCD_SEG_E :OR: LCD_SEG_F :OR: LCD_SEG_G)
LCD_F_PIN2   DCB  LCD_SEG_A
;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<

STR_BUF			SPACE	MAX_STR_LEN
				ALIGN

;Queue Records	
TxQRecord		SPACE	18
				ALIGN
RxQRecord		SPACE	18
				ALIGN
;Queue Buffer
RxQBuffer		SPACE	IRQ_BUF_SZ
				ALIGN
TxQBuffer		SPACE	IRQ_BUF_SZ
				ALIGN

;Timer variables
				EXPORT	Count
Count			SPACE	4
				EXPORT	RunStopWatch
RunStopWatch	SPACE	1
				ALIGN
    
;>>>>>   end variables here <<<<<
            END