/*
 * A Microchip PIC-based Ukulele Tuner
 * Son Phan
 * http://github.com/sonphanusa
 *
 * This is the software for a standard 4-string soprano ukulele tuner that tunes the 
 * following 4 notes at their respective frequencies: G (392 Hz), C (262 Hz), E (330 Hz), A (440 Hz). 
 * The electronic tuner includes a microphone to listen to sound inputs (ideally from a ukulele string), 
 * a LED segment display to show one of four notes (G, C, E, A), and a set of red/green LEDs to guide 
 * the pitch if it is low, high, or correct.
 * 
 * This program is developed and built in MPLAB IDE 8.92 with HITECH-C compiler.
 *
 * Reference:
 * Microchip Based Automatic Guitar Tuner - Ryan Monteleone
 * http://www.csuohio.edu/academic/success_in_math/posters/CSU/CSU_2012/Monteleon.pdf
 */

#include <htc.h> 	//HI_TECH C header file
#include "delay.h"	//Microchip delay library

__CONFIG (CP_OFF & CPD_OFF & LVP_OFF & WDTE_OFF & BOREN_OFF & PWRTE_OFF & WRT_OFF & FOSC_XT);

/* DEFINE CONSTANTS */
#define C	261.6	// Notes' frequencies
#define E	329.6	// +68 Hz
#define G	392.0	// +62.4
#define A	440.0 	// +48

#define OSC_CLK		3686400.0 	// external/oscillator clock
#define INT_CLK		(OSC_CLK/4)	// instruction clock

#define	TMR1_PRESC	4	// Timer1 prescale 1:4
#define	TMR1_EDGES	1	// Timer1 capture event (every rising edge)

#define FREQ_TOLE	3.5		// Frequency tolerance
#define LOWER_BOUND	100.0	// Set freq. lower bound to 100Hz
#define UPPER_BOUND 600.0	// Set freq. upper bound to 600Hz

#define DELAY_TIME	20		// Delay timer in ms

/* FUNCTIONS PROTOTYPE */
// Init registers and ports values
void core_init(void);
// Light up LEDs according to pitch
// uses the frequency and the note reference 
// as passed in parameters.
void light_pitch_led(float freq, float note);

/* FUNCTIONS IMPLEMENT */
void core_init(void)
{
	INTCON =	0b11000000;		//Enable Global (GIE) and Peripheral (PEIE) interrupts

	TRISB =		0b11000000; 	//Set PORTB pins to outputs for pitch LEDs
	TRISC =		0b00000100; 	//Set RC2 to input for sound signal
	TRISD =		0b00000000;		//Set PORTD pins to outputs for 7-segment LED display
	T1CON =		0b00100001; 	//Set TMR1 prescale (TMR1_PRESC)

	CCP1CON	=	0b00000101; 	//Set CCP1 to capture mode at rising TMR1_EDGES
	TMR1H =		0b00000000;		//Clear Timer1 register
	TMR1L =		0b00000000; 	//
	CCPR1H =	0b00000000;		//Clear CCPR1 register
	CCPR1L =	0b00000000; 	//	

	CCP1IE =	0;	// Disable CCP1 interrupt
	CCP1IF =	0;	// Clear CCP1 interrupt flag
	TMR0IE =	0;	// Disable interrupt on TMR0 rollover

	PORTD =	0b11111111;	// Turn off all 7 segments of the LED display

	RB0=0;	// Turn off all pitch LEDs
	RB1=0;	//
	RB2=0;	//
}

// Main
void main(void)
{
	unsigned int capold, capnew, cap;	//Timer1 capture values
	float FREQ;	//Frequency calculated value

	//Initialize registers and ports
	core_init();

	//Calculation and display UI
	while(1)
	{
		if (CCP1IF) //TMR1 capture occured (interrupt flag set)
		{
			capold = capnew; //Store old capture value
			capnew = 256*CCPR1H + CCPR1L; //Get new 16-bit CCPR1 capture value (CCPR1H:CCPR1L)
			CCP1IF = 0;	//Clear the flag

			if (capnew > capold) //Verify both captures are independent
			{
				//Because TMR1 is set to run free without resetting to 0,
				//we capture its new value at every rising TMR1_EDGES
				//and subtract the old value to get the actual new value (the difference)
				cap = capnew-capold;
				//Calculate the frequency based on Timer1, and number of rising TMR1_EDGES
				//TMR1 Frequency	= INT_CLK / TMR1_PRESC;
				//Capture Period	= cap / TMR1_EDGES;
				FREQ 				= (INT_CLK/TMR1_PRESC) / (cap/TMR1_EDGES);

				//Tuning and setting User Interface
				//in the order of G,C,E,A
				if( ((G+E)/2)<FREQ && FREQ<((A+G)/2) )
				{
					PORTD=0b10000001;	//G note: 7-segment LED display
					light_pitch_led(FREQ, G);
				}

				if( LOWER_BOUND<FREQ && FREQ<((E+C)/2) )
				{
					PORTD=0b10100011;	//C note
					light_pitch_led(FREQ, C);
				}

				if( ((E+C)/2)<FREQ && FREQ<((G+E)/2) )
				{
					PORTD=0b10000011;	//E note
					light_pitch_led(FREQ, E);
				}

				if( ((A+G)/2)<FREQ && FREQ<UPPER_BOUND )
				{
					PORTD=0b00000101;	//A note
					light_pitch_led(FREQ, A);
				}

				//Delay to 'debounce' harmonic
				DelayMs(DELAY_TIME);
			}
		}
	} //end while(1)
}

void light_pitch_led(float freq, float note)
{
	if( freq<(note-FREQ_TOLE) )
	{
		RB2=1;	//low pitch
		RB1=0;
		RB0=0;
		return;
	}
	if( (note-FREQ_TOLE)<freq && freq<(note+FREQ_TOLE) )
	{
		RB2=0;	//correct pitch
		RB1=1;
		RB0=0;
		return;
	}
	if( freq>(note+FREQ_TOLE) )
	{
		RB2=0;	//high pitch
		RB1=0;
		RB0=1;
		return;
	}
}

