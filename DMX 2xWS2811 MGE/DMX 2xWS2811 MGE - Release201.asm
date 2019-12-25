/*
 * DMX 2xWS2811 MGE.asm
 *
 *  Created: 12/12/2019
 *  Modified by Matthew Edwards
 *  High Address link added
 *  LPD6803 section removed
 *  Gamma table removed
 *  PIXEL PROTOCOL SELECTION removed
 *  Compiled with AVR Studio 7.0
 */ 
/*
 *  Created: 2/26/2014 6:24:09 PM
 *  Modified by Mac Hosehead
 *  Modified original source to work with WS2811 pixels
 *	16MHz crystal must replace 20MHz crystal
 *	Must use !Q output of 7475 for pixel data
 *	LPD6803 section is disabled
 *  No longer using Gamma table
 *  Compiled with AVR Studio 5.1
 */ 


;*******************************************************************************
;
; This file contains the firmware source code for a 512 channel 
; DMX to PIXEL 4 port bridge, compatible with WS2801 and LPD6803 RGB nodes.
;
;*******************************************************************************
;
; Title			: DMX to RGB Pixel Bridge - 4 Port
; Version		: v1.02
; Last updated	: 04/03/2011
;
; Written for the Atmel Atmega88/168/328 running at 20Mhz
;
; Copyright (C) 2011 Robert P. Martin, rcflier@hotmail.com
;
; DMX receive and LED routines based on code written
; by Hendrik Hölscher, www.hoelscher-hi.de
;
; Code developed with Atmel AVR Studio 4.18
;
;*******************************************************************************
;
; This program is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; This program is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with this program.  If not, see http://www.gnu.org/licenses/
;
;*******************************************************************************
;
; ACTIVITY LED DEFINITIONS:
;
; RED (LED1):
; SOLID ON	= Unit powered on and waiting for DMX signal.
; 1 BLINK	= Error, No DMX signal present.
; 2 BLINKS	= Error, DMX signal bad or polarity reversed.
;
; GREEN (LED2):
; OFF		= No DMX data recieved
; BLINKING	= Valid DMX data recieved.
;
;*******************************************************************************
;
; FUSES:
;
; LOW     =0xFF
; EXTENDED=0xDD
; HIGH    =0xDF
;
; Brown-out detection at VCC=2.7V
; Ext. Crystal Osc. 8.0- MHz; Start-up time PWRDWN/RESET: 16K CK/14 CK + 65 ms
;
;*****************************************************************************


; ***** Port Pin definitions 

#define	LED1		PB0				; Red Power/Error LED
#define	LED2		PB1				; Green DMX activity LED
#define	SS			PB2				; Slave Select Out
#define MOSI		PB3				; Master Out Slave In
#define MISO		PB4				; Master In Slave Out
#define SCK			PB5				; Serial Clock Out
#define BANK1		PC0				; Bank1 output select for channels 1-126
#define BANK2		PC1				; Bank2 output select for channels 127-252
#define	BLANK		PD4				; DMX output blanking option 
#define	DMX_PRG		PD7				; DMX Start Address Program jumper

; ***** Constants

.equ DMX_BUFFER		=0x0100			; DMX recieve buffer storage at start of SRAM 
.equ BANK_CHANNELS	=126			; Number of DMX channels per bank (126 for 42 RGB nodes/bank MAX 255)
.equ F_OSC			=16000000		; Clock Frequency = 16mhz

; ***** Status Flags

#define VALID_DMX	 	1			; Status bit for DMX valid register
#define DMX_RX 			2			; Status bit for DMX received register
#define RX_STATUS 		3			; Status bit for DMX changed register

; ***** Global register variables

#define	SREGbuf			R1			; Temporary SREG register
#define Blink 			R2			; ACTIVITY LED Blink status register
#define BlinkPos		R3			; ACTIVITY LED Blink position register
#define LEDdelay 		R4			; ACTIVITY LED Blink prescaler register
#define	Data			R5			; SPI output temp register
#define Ws2811_low		R6
#define Ws2811_high		R7
#define	tempL			R16			; Temporary LOW register
#define	tempH			R17			; Temporary HIGH register 
#define	Flags 			R18			; DMX status flag register
#define DMXstate		R19			; Temporary register 	
#define	step_count		R20			; Temporary register
#define	RED				R21			; LED Node RED bit temp register
#define	GREEN			R22			; LED Node GREEN bit temp register
#define	BLUE			R23			; LED Node BLUE bit temp register
#define	Interlace_Count	R24			; Counter to interlace 6803 pixels
#define	Data_Tmp		R25			; Temp holder for Data in Get_data_6803
;#define DMXaddL			R26			; DMX Address Low Register
;#define DMXaddH			R27			; DMX Address High Register

; ***** Interrupt Service Routine vectors

.org 0x0000		rjmp	reset		; Reset Vector 
.org OVF0addr	rjmp	LED_flash	; LED Flash ISR
.org URXCaddr	rjmp	RX_byte		; USART DMX Receive ISR 

; ***** Reset start vector

reset:

	cli

; ***** Setup stackpointer 

	ldi		tempL,low(RAMEND)
	ldi		tempH,high(RAMEND)
	out		SPL,tempL
	out		SPH,tempH

; ***** Setup Watchdog Timer timeout for 2 seconds

	wdr								; Watchdog Timer Reset 
	ldi		tempL,(1<<WDE)|(1<<WDCE)
	sts		WDTCSR, tempL
	ldi		tempL,(1<<WDE)|(1<<WDCE)|(0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0)
	sts		WDTCSR, tempL
	ldi		tempL,(0<<WDCE)
	sts		WDTCSR,tempL
	wdr								; Watchdog Timer Reset 

; ***** Setup Ports 

;  Initialize PortB 

	ldi 	tempL,0b10101111
	out		DDRB, tempL				; Setup PortB for 4,6 as inputs, 0,1,2,3,5,7 as outputs
	ldi 	tempL,0b01010010
	out		PortB,tempL 			; Clear PortB Outputs with pullups on input ports 4,6 

;  Initialize PortC 

	ldi		tempL,0b00001111		; Setup ports 0-3 as output, ports 4-7 as inputs
	out 	DDRC, tempL				; Setup PortC
	ldi		tempL,0b11110000
	out 	PortC,tempL  			; Setup inputs with pull-ups, set outputs low

;  Initialize PortD 

	ldi 	tempL,0b00000010		; Setup Ports 0,2,3,4,5,6,7 as inputs - Port 1 as an output 
	out 	DDRD, tempL
	ldi 	tempL,0b11111101
	out 	PortD,tempL 			; Setup ports 0,2,3,4,5,6,7 Pull-ups, Clear port 1 output

;  Initialize SPI interface

	ldi		tempL,(0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0)
	out		SPCR,tempL				; Enable SPI as Master, set SCK clock rate @ F_OSC/4 (4 Mhz)
	ldi		tempL,(0<<SPI2X)		; 
	out		SPSR,tempL

; ***** Initialize timer 0 for Activity LED
 
	clr		tempL
	out		TCCR0A,tempL
	ldi 	tempL,0b00000101		; Set timer0 to clk/1024 
	out 	TCCR0B,tempL
	ldi		tempL,(1<<TOIE0)
;	sts 	TIMSK0,tempL			; Enable timer0 overflow interrupts
	
; ***** Initialize USART for DMX receive

   	ldi 	tempL,((F_OSC/4000000)-1) ; Set USART to 250k baud rate
   	sts 	UBRR0L,tempL
   	clr 	tempL
   	sts 	UBRR0H,tempL
	ldi		tempL,(1<<RXEN0)
	sts 	UCSR0B, tempL
   	ldi 	tempL,(1<<USBS0)|(3<<UCSZ00)
   	sts 	UCSR0C,tempL
	lds		tempL,UDR0
	clr		tempL
	sts		UCSR0A,tempL
	lds		tempL,UCSR0B
	sbr		tempL,(1<<RXCIE0)|(1<<RXEN0)
	sts		UCSR0B,tempL

; ***** Define macro routines

.macro latch_delay					; 1mS delay macro for latch setup time
	ldi		tempH,1
latch_delay:
	ldi		tempL,252
latch_delay1:
	dec		tempL
	brne	latch_delay1
	dec		tempH
	brne	latch_delay
.endmacro

.macro PushREG						; Save global status MACRO
	in		SREGbuf,SREG			; Push registers to stack 
	push	SREGbuf
	push 	tempH
	push	tempL
.endmacro

.macro PopREG						; Restore global status and Exit ISR MACRO
	pop		tempL					; Pop registers from stack
	pop		tempH
	pop		SREGbuf
	out     SREG,SREGbuf
	reti							; Exit ISR	
.endmacro

; ***** Initialize variables 

	ldi		tempL,0b01111111
	mov		Ws2811_low,tempL		; the inverse of SPI data is being used so this is an inverse too
	ldi		tempL,0b00001111
	mov		Ws2811_high,tempL		; this is the inverse for high
	clr 	Flags					; Clear status flags register
	clr		step_count				; Clear step counter
	ldi		tempL,8					; Setup LED startup values
	mov		BlinkPos, tempL
	ldi   	tempL,6
	mov   	LEDdelay,tempL
	wdr								; Watchdog Timer Reset
;	ldi		DMXaddH,0				; Set DMX start address to 1
;	ldi		DMXaddL,1


; ***** Clear DMX buffer SRAM memory

	rcall	clear_buffer			; Call buffer clear subroutine
	sei								; Turn on global interrupts

; ***** Start main program loop - ISR DMX RX & ACTIVITY LED, POLLED PIXEL OUTPUT

main:
	rcall	Channel_start
	rjmp	main

; ***** Channel output routine 

Channel_start:

	ldi		YH,high(DMX_BUFFER)		; Reset pointers to start of DMX channel SRAM buffer
	ldi		YL,low (DMX_BUFFER)
;	inc		Interlace_Count			; Start interlace counter
;	inc		Interlace_Count			; increment again for 4 steps
;	andi	Interlace_Count,$06		; don't go past 6


	sbic	PinD, DMX_PRG			; Check if jumper is set DISPLAY  DMX start address PRG mode
	rjmp	display_banks			; If so, then set DMX start address

; ***** DUMP CHANNEL BANK 1
	ldi		step_count,BANK_CHANNELS; Reset channel step counter 
	rcall	start_2811				; Output channel data for 2811 Pixels
	rcall	poll_timer

; ***** DUMP CHANNEL BANK 2
	ldi		step_count,BANK_CHANNELS; Reset channel step counter 
	rcall	start_2811				; Output channel data for 2811 Pixels
	rcall	poll_timer


display_banks:

; ***** OUTPUT CHANNEL BANK 1 or 3
	ldi		step_count,BANK_CHANNELS; Reset channel step counter 
	sbi 	PortC,BANK1				; Activate channel bank1
	rcall	start_2811				; Output channel data for 2811 Pixels
	cbi 	PortC,BANK1				; Deactivate channel bank1
	rcall	poll_timer

; ***** OUTPUT CHANNEL BANK 2 or 4
	ldi		step_count,BANK_CHANNELS; Reset channel step counter 
	sbi 	PortC,BANK2				; Activate channel bank2
	rcall	start_2811				; Output channel data for 2811 Pixels
	cbi 	PortC,BANK2				; Deactivate channel bank2
	rcall	poll_timer

	latch_delay						; Latch delay macro
	ret

; ********************* 2811 PIXEL OUTPUT ROUTINE

start_2811:							; Get channel data from Gamma table

	ld		Data,Y+					; Get DMX buffer data and store in temp buffer
	cli								; Stop interrupts
	sbrc		Data,7				; Send MSB
	out		SPDR,Ws2811_high
	sbrs		Data,7
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,6
	out		SPDR,Ws2811_high
	sbrs		Data,6
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,5
	out		SPDR,Ws2811_high
	sbrs		Data,5
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,4
	out		SPDR,Ws2811_high
	sbrs		Data,4
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,3
	out		SPDR,Ws2811_high
	sbrs		Data,3
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,2
	out		SPDR,Ws2811_high
	sbrs		Data,2
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,1
	out		SPDR,Ws2811_high
	sbrs		Data,1
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit
	cli
	sbrc		Data,0
	out		SPDR,Ws2811_high
	sbrs		Data,0
	out		SPDR,Ws2811_low
	sei
	rcall	Wait_Transmit			; Wait for SPI data to finish transferring

; ***** Check step counter

	dec		step_count				; Decrement step counter and check if all steps are done 
	breq	end_channels_2811		; All steps done now exit, else continue
	rjmp	start_2811				; Start outputs over

end_channels_2811:

	ret								; Return to calling routine

; ********************* POLL LED TIMER OVERFLOW

poll_timer:

	sbic	TIFR0,0
	rcall	LED_flash
	ret


Wait_Transmit:						; SUBROUTINE - Wait for SPI transmission to complete
	in		tempL,SPSR
	sbrs	tempL,SPIF				; Check if SPI is done with transmission
	rjmp	Wait_Transmit			; If not done, then loop back and recheck
	ret								; Return to calling routine

; ***** LED Indicators flash ISR

LED_flash:

	wdr								; Watchdog Timer Reset
	sbi		TIFR0,0
;	PushREG							; Save global status MACRO

	dec	  	LEDdelay				; LED flash rate register	
	brne  	LED_end
	ldi   	tempL,6
	mov   	LEDdelay,tempL

	sbrc	Flags,RX_STATUS			; Check to see if Green LED should flash 
	rjmp	LED2_flash				; If so, turn Green LED on 
	sbi		PortB,LED2				; else turn Green LED off
	rjmp 	LED1_flash

LED2_flash:

	cbr		Flags,(1<<RX_STATUS)	; Blink Green LED
	sbic 	PortB,LED2
	rjmp 	LED2_off
	sbi		PortB,LED2				; Turn Green LED on
	rjmp 	LED1_flash

LED2_off:

	cbi 	PortB,LED2				; Turn Green LED off

LED1_flash:

	lsr   	Blink					; Check to see if Red LED should flash 
	sbrc  	Blink,0					; If 1st bit is high 
	rjmp  	LED1_off				; then turn Red LED off
	cbi   	PortB,LED1				; else turn Red LED on
	rjmp  	LED_test

LED1_off:

	sbi  	PortB,LED1				; Turn Red LED off
	sbrc	Flags,DMX_RX			; Check RX status before blanking outputs
	rjmp  	LED_test

; ***** Clear outputs on DMX receive error

blanking:

	rcall	clear_buffer			; Call buffer clear subroutine

LED_test:

    mov		tempL,BlinkPos			; Should activity LED blink? 
	dec	  	tempL
	breq	LED_toggle				; If so, then blink
	mov		BlinkPos,tempL			; else, decrement counter and exit
	rjmp	LED_end

LED_toggle:

    ldi		tempL,8					; Toggle activity LED blink pattern (blink = 0) 
	mov		BlinkPos,tempL
	clr		tempL
	sbrs 	Flags,VALID_DMX
	ldi	 	tempL,0b11110110
	sbrs 	Flags,DMX_RX
	ldi	 	tempL,0b11111110
	mov  	Blink,tempL
	cbr		Flags,(1<<VALID_DMX)|(1<<DMX_RX)

LED_end:

;	PopREG							; Restore global status and Exit ISR MACRO
	ret

; ***** Clear SRAM DMX buffer memory subroutine

clear_buffer:

	ldi		XH,high(DMX_BUFFER)		; Reset pointers for DMX channel data SRAM buffer
	ldi		XL,low (DMX_BUFFER)
	clr		tempL
	ldi 	tempH,255

clear_l:							; Now clear DMX channel buffer SRAM memory
	st 		X+,tempL 				; Clear SRAM - low bank (channels 1-255)
	dec		tempH
	brne	clear_l
clear_h:
	st 		X+,tempL				; Clear SRAM - high bank (channels 256-512)
	dec		tempH
	brne	clear_h
	ret								; Return to calling routine

; ***** DMX receive ISR 

RX_byte:

	PushREG							; Save global status MACRO
	sbr		Flags,(1<<DMX_RX)		; Indicator flag for USART triggered
	lds		tempL,UCSR0A			; Get status register before data register
	lds		tempH,UDR0
	sbrc	tempL,DOR0				; Check for overrun. Wait for reset
	rjmp	overrun
	sbrc	tempL,FE0				; Check for frame error. If so, then reset
	rjmp	frame_error
	cpi		DMXstate,1   			; Check for start byte
	breq	start_byte
	cpi 	DMXstate,2				; Check for start adress
	breq    start_addr
  	cpi 	DMXstate,3				; Now handle byte
	brsh	handle_byte

finish_byte:

	PopREG							; Restore global status and Exit ISR MACRO		

start_byte:

	tst 	tempH	 				; If null then byte overrun condition exist
	brne 	overrun					; Jump to handle overrun
	inc  	DMXstate				; Else, set status for start address and continue
;	mov		XH, DMXaddH				; DMX start address
;	mov		XL, DMXaddL
	ldi		XH,0					; Set DMX start address to channel 1
	ldi		XL,1
	rjmp	finish_byte

start_addr:

	sbiw	XH:XL,1
	brne	finish_byte
	ldi 	XH,high(DMX_BUFFER)		; Reset pointers for DMX channel data SRAM buffer
	ldi		XL,low (DMX_BUFFER)
	inc  	DMXstate 				; Now get data

handle_byte:

	ld		tempL,X					; Get old value
	cp		tempL,tempH
	breq	no_change				; If it matches current SRAM contents then jump
	st		X,tempH					; Else, store byte in SRAM
	sbr		Flags,(1<<RX_STATUS)	; Set indicator flag for channel data changed

no_change:

	adiw	XH:XL,1
	cpi		XL,low (DMX_BUFFER+512)	; Check for end of DMX channels
	brne	finish_byte				; If not done, exit
	cpi		XH,high(DMX_BUFFER+512)
	brne	finish_byte
	sbr		Flags,(1<<VALID_DMX)	; Else, set indicator flag for all channels received
	clr		DMXstate				; and continue
	rjmp	finish_byte

frame_error:						; Frame error detected as break

	ldi 	DMXstate,1				; Set status bit
	lds		tempH,UDR0				; Read RX buffer to clear error
	rjmp	finish_byte

overrun:							; Wait for frame error

 	clr 	DMXstate				; Clear status bit
	lds		tempH,UDR0				; Read RX buffer twice to clear error
	lds		tempH,UDR0
	rjmp	finish_byte

