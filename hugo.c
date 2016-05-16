//! Fe's PHD hat application
//!
//! Using Atmega8 microcontroller

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//#include <inttypes.h>
#include <util/delay.h>
 
#define F_CPU           3686400		// Taktfrequenz

#ifndef F_CPU
#define F_CPU           1000000		// processor clock frequency
#warning kein F_CPU definiert
#endif

#define SERVO_DDR       DDRB
#define SERVO_PORT      PORTB
#define SERVO_PIN       PINB
#define SERVO_LEFT      1
#define SERVO_RIGHT     2
 
#define KEY_DDR         DDRD
#define KEY_PORT        PORTD
#define KEY_PIN         PIND
#define KEY0            0
#define KEY1            1
#define KEY2            2
#define KEY3            3
#define KEY4            4
#define KEY5            5
#define ALL_KEYS        (1<<KEY0 | 1<<KEY1 | 1<<KEY2 | 1<<KEY3 | 1<<KEY4 | 1<<KEY5)
 
#define REPEAT_MASK     (1<<KEY1 | 1<<KEY2)       // repeat: key1, key2
#define REPEAT_START    50                        // after 500ms
#define REPEAT_NEXT     20                        // every 200ms
 
#define LED_DDR         DDRC
#define LED_PORT        PORTC
#define LED0            0
#define LED1            1
#define LED2            2
#define LED3            3
#define LED4            4
#define LED5            5
 
volatile uint8_t key_state;		// debounced and inverted key state:
					// bit = 1: key pressed
volatile uint8_t key_press;		// key press detect
 
volatile uint8_t key_rpt;		// key long press and repeat
 
#define N_STATES 6
#define SPEED    0.25

volatile int8_t g_state[N_STATES];
volatile uint8_t g_count[N_STATES];
volatile uint8_t g_max[N_STATES];
volatile uint8_t g_speed;

ISR( TIMER0_OVF_vect )			// every 10ms
{
	static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt;
	uint8_t i;
	
	TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
	
	i = key_state ^ ~KEY_PIN;                       // key changed ?
	ct0 = ~( ct0 & i );                             // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
	i &= ct0 & ct1;                                 // count until roll over ?
	key_state ^= i;                                 // then toggle debounced state
	key_press |= key_state & i;                     // 0->1: key press detect
	
	if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
		rpt = REPEAT_START;                     // start delay
	if( --rpt == 0 ){
		rpt = REPEAT_NEXT;                      // repeat delay
		key_rpt |= key_state & REPEAT_MASK;
	}

	for (i = 0; i < N_STATES ; i++) {

		if (g_state[i]) {

			g_count[i]++;

			if (g_count[i] > SPEED * g_max[i]) {

				switch (i) {
				case 0:
					LED_PORT ^= 1<<LED0;
					break;
				case 1:
					LED_PORT ^= 1<<LED1;
					break;
				case 2:
					LED_PORT ^= 1<<LED2;
					break;
				case 3:
					LED_PORT ^= 1<<LED3;
					break;
				case 4:
					LED_PORT ^= 1<<LED4;
					break;
				case 5:
					LED_PORT ^= 1<<LED5;
					break;
				}

				LED_PORT ^= 1<<LED5;

				g_count[i] = 0;
			}
		}
	}
}
 
//! @brief Check if a key has been pressed.
//!
//! Each pressed key is reported only once.
uint8_t get_key_press( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_press;                          // read key(s)
	key_press ^= key_mask;                          // clear key(s)
	sei();

	return key_mask;
}
 
//! @brief check if a key has been pressed long enough
//!
//! such that the
//! key repeat functionality kicks in. After a small setup delay
//! the key is reported being pressed in subsequent calls
//! to this function. This simulates the user repeatedly
//! pressing and releasing the key.
uint8_t get_key_rpt( uint8_t key_mask )
{
	cli();                                          // read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	sei();

	return key_mask;
}
 
//! check if a key is pressed right now
uint8_t get_key_state( uint8_t key_mask )
{
	key_mask &= key_state;

	return key_mask;
}
 
uint8_t get_key_short( uint8_t key_mask )
{
	cli();                                          // read key state and key press atomic !

	return get_key_press( ~key_state & key_mask );
}
 
uint8_t get_key_long( uint8_t key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}

int init(void)
{
	int i;

	g_max[0] = 0;
	g_max[1] = 16;
	g_max[2] = 8;
	g_max[3] = 4;
	g_max[4] = 2;
	g_max[5] = 2;

	for (i = 0; i < N_STATES ; i++) {
		g_state[i] = 0;
		g_count[i] = 0;
	}

	// Enable ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
//	ADMUX = 0x00;
	ADMUX = (1<<REFS0);

	LED_PORT = 0x00;
	LED_DDR = 0xFF;                     

	SERVO_PORT = 0x00;
	SERVO_DDR = 0xFF;                     

	ICR1 = 20000;
	TCCR1A  = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B =  (1<<WGM13)|(1<<WGM12)|(1<<CS10);

	// Configure debouncing routines
	KEY_DDR &= ~ALL_KEYS;                // configure key port for input
	KEY_PORT |= ALL_KEYS;                // and turn on pull up resistors
 
	TCCR0 = (1<<CS02)|(1<<CS00);         // divide by 1024
	TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
	TIMSK |= 1<<TOIE0;                   // enable timer interrupt
 
	sei();
}
 
int main( void )
{
	init();

	while(1){
		ADCSRA |= (1<<ADSC);

		while (ADCSRA & (1<<ADSC));

		g_speed = ADCW;

		OCR1A = 700;  // servo1
		OCR1B = 700;  // servo2
                _delay_ms(1500);

		OCR1A = 2200; //180-deg
		OCR1B = 2200;
		_delay_ms(1500);

		if( get_key_short( 1<<KEY0 )) {

			g_state[0] = !g_state[0];

			if (!g_state[0])
				LED_PORT &= ~(1<<LED0);
		}
 
		if( get_key_short( 1<<KEY1 )) {

			g_state[1] = !g_state[1];

			if (!g_state[1])
				LED_PORT &= ~(1<<LED1);
		}

		if( get_key_short( 1<<KEY2 )) {

			g_state[2] = !g_state[2];

			if (!g_state[2])
				LED_PORT &= ~(1<<LED2);
		}

		if( get_key_short( 1<<KEY3 )) {

			g_state[3] = !g_state[3];

			if (!g_state[3])
				LED_PORT &= ~(1<<LED3);
		}

		if( get_key_short( 1<<KEY4 )) {

			g_state[4] = !g_state[4];

			if (!g_state[4])
				LED_PORT &= ~(1<<LED4);
		}

		if( get_key_short( 1<<KEY5 )) {

			g_state[5] = !g_state[5];

			if (!g_state[5])
				LED_PORT &= ~(1<<LED5);
		}
	}
}

