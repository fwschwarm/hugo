//! Fe's PHD hat application
//!
//! Using Atmega8 microcontroller

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
 
#define F_CPU           3686400		// Taktfrequenz

#define SERVO_DDR       DDRB
#define SERVO_PORT      PORTB
#define SERVO_PIN       PINB
#define SERVO_LEFT      1
#define SERVO_RIGHT     2
 
#define INPUT_DDR       DDRC
#define INPUT_PORT      PORTC
#define INPUT_PIN       PINC
#define INPUT_GROUND    2


//ISR(TIMER0_OVF_vect)
//{
//}
 
int init(void)
{
	int i;

	// Enable ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
//	ADMUX = 0x00;
	ADMUX = (1<<REFS0);

	SERVO_PORT = 0x00;
	SERVO_DDR = 0xFF;                     

	ICR1 = 20000;
	TCCR1A  = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B =  (1<<WGM13)|(1<<WGM12)|(1<<CS10);

	// Configure debouncing routines
	INPUT_DDR &= ~0xFF;	// configure key port for input
	INPUT_PORT |= 0xFF;	// and turn on pull up resistors
 
	TCCR0 = (1<<CS02)|(1<<CS00);         // divide by 1024
//	TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
//	TIMSK |= 1<<TOIE0;                   // enable timer interrupt

	sei();

	OCR1A = -1;
	OCR1B = -1;
 
}
 
int main( void )
{
	init();

	while(1){
//		ADCSRA |= (1<<ADSC);
//		while (ADCSRA & (1<<ADSC));
//		g_speed = ADCW;

		if (INPUT_PIN &= 1<<INPUT_GROUND) {
			OCR1A = -1;
			OCR1B = -1;
		} else {
			OCR1A = 900;
	//		OCR1B = 10;
		}

	}
}

