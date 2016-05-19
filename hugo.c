//! Fe's PHD hat application
//!
//! Using Atmega8 microcontroller

#define F_CPU           1000000

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SERVO_DDR       DDRB
#define SERVO_PORT      PORTB
#define SERVO_PIN       PINB
#define SERVO_LEFT      1
#define SERVO_RIGHT     0
 
#define INPUT_DDR       DDRC
#define INPUT_PORT      PORTC
#define INPUT_PIN       PINC
#define INPUT_GROUND    2


//ISR(TIMER0_OVF_vect)
//{
//}

//! Move from -90 to 90 degree
//! Offset goes from -1 to 1.0
void move(int id, double offset)
{
	switch (id) {
	case SERVO_LEFT:
		if (offset == 0.0) {
			OCR1A = -1;
		} else {
			OCR1A = ICR1 * (1.5 + 0.5 * offset) / 20;
		}
		break;
	case SERVO_RIGHT:
		if (offset == 0.0) {
			OCR1B = -1;
		} else {
			OCR1B = ICR1 * (1.5 - 0.5 * offset) / 20;
		}
		break;
	}
}
 
int init(void)
{
	int i;

	// Enable ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
//	ADMUX = 0x00;
	ADMUX = (1<<REFS0);

	SERVO_PORT = 0x00;
	SERVO_DDR = 0xFF;                     

	ICR1 = 19999;
	TCCR1A  = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B =  (1<<WGM13)|(1<<WGM12)|(1<<CS10);

	// Configure debouncing routines
	INPUT_DDR &= ~0xFF;	// configure key port for input
	INPUT_PORT |= 0xFF;	// and turn on pull up resistors
 
	TCCR0 = (1<<CS02)|(1<<CS00);         // divide by 1024
//	TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
//	TIMSK |= 1<<TOIE0;                   // enable timer interrupt

	sei();

	move(SERVO_LEFT, 0.0);
	move(SERVO_RIGHT, 0.0);
}

int main( void )
{
	int i;

	init();

	while(1){
//		ADCSRA |= (1<<ADSC);
//		while (ADCSRA & (1<<ADSC));
//		g_speed = ADCW;

		if (INPUT_PIN &= 1<<INPUT_GROUND) {
			move(SERVO_LEFT, 0.0);
			move(SERVO_RIGHT, 0.0);
		} else {
			move(SERVO_LEFT, -0.1);
			move(SERVO_RIGHT, -0.1);
		}

	}
}

