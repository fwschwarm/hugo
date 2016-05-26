//! Fe's PHD hat application
//!
//! Using Atmega8 microcontroller

#define F_CPU           1000000

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SPEED_SLOW	0.1
#define SPEED_FAST	0.9

#define SPEED_BASE	0.15

#define MODE_MOTH	0
#define MODE_BUG	1

#define SERVO_DDR       DDRB
#define SERVO_PORT      PORTB
#define SERVO_PIN       PINB
#define SERVO_LEFT      1
#define SERVO_RIGHT     0
 
#define SENSOR_DDR	DDRC
#define SENSOR_PORT	PORTC
#define SENSOR_PIN	PINC
#define SENSOR_LEFT	1
#define SENSOR_RIGHT	0
#define SENSOR_GROUND	2
#define SENSOR_CRIGHT	4
#define SENSOR_CLEFT	5

int g_mode = MODE_MOTH;

//ISR(TIMER0_OVF_vect)
//{
//}

//! Read value from ADC
uint16_t measure(uint8_t ch)
{
	//! Select ADC Channel ch must be 0-7
	ch = ch&0b00000111;
	ADMUX &= ch;
	ADMUX |= ch;

	//! Start Single conversion
	ADCSRA |= (1<<ADSC);

	//! Wait for conversion to complete
	while (!(ADCSRA & (1<<ADIF)));

	//! Clear ADIF
	ADCSRA |= (1<<ADIF);

	return(ADC);
}

//! Move from -90 to 90 degree
//! Offset goes from -1 to 1.0
void move(int id, double offset)
{
	switch (id) {
	case SERVO_LEFT:
//		if (offset == 0.0) {
//			OCR1A = -1;
//		} else {
			OCR1A = ICR1 * (1.5 - 0.5 * offset) / 20;
//		}
		break;
	case SERVO_RIGHT:
//		if (offset == 0.0) {
//			OCR1B = -1;
//		} else {
			OCR1B = ICR1 * (1.5 + 0.5 * offset) / 20;
//		}
		break;
	}
}
 
int init(void)
{
	int i;

	SERVO_PORT = 0x00;
	SERVO_DDR = 0xFF;                     

	//! Activate the servo hardware timers
	ICR1 = 19999;
	TCCR1A  = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B =  (1<<WGM13)|(1<<WGM12)|(1<<CS10);

	//! Configure sensor port
	SENSOR_DDR &= ~0xFF;
	//! Turn on internal pull ups
	SENSOR_PORT |= 0xFF;

	//! Enable the ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);
	ADMUX = (1<<REFS0);
//ADMUX=(1<<REFS0);                         // For Aref=AVcc;
//ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Rrescalar div factor =128

	//! Activate the 8-bit timer
	TCCR0 = (1<<CS02)|(1<<CS00);         // divide by 1024
//	TCNT0 = (uint8_t)(int16_t)-(F_CPU / 1024 * 10e-3 + 0.5);  // preload for 10ms
//	TIMSK |= 1<<TOIE0;                   // enable timer interrupt

	//! Enable interupts
	sei();

	//! Stop motors
	move(SERVO_LEFT, 0.0);
	move(SERVO_RIGHT, 0.0);
}

int crazy(void)
{
	return 0;
}

int work(void)
{
	static uint32_t evasive_count = 0;
	static int evasive = 0;
	int bump_left = 0, bump_right = 0;
	uint16_t photo_left, photo_right, photo_ave;
	uint32_t photo_tot;
	double speed_left = SPEED_BASE, speed_right = SPEED_BASE;
	double speed_factor = 1.0;

	//! Check for collision
	if (SENSOR_PIN &= 1<<SENSOR_CLEFT) {
		bump_left = 0;
	} else {
		move(SERVO_LEFT, 0.0);
		bump_left = 1;
		evasive = 1;
	}

	if (SENSOR_PIN &= 1<<SENSOR_CRIGHT) {
		bump_right = 0;
	} else {
		move(SERVO_RIGHT, 0.0);
		bump_right = 1;
		evasive = 2;
	}

	if (bump_left || bump_right)
		evasive_count = 3E-4 * F_CPU;

	if (bump_left && bump_right)
		evasive = 3;

	//! Check for evasive maneuver
	if (evasive_count > 0) {

		evasive_count--;

		switch (evasive) {
		case 1:
			speed_left = SPEED_SLOW;
			speed_right = SPEED_FAST;
			break;
		case 2:
			speed_left = SPEED_FAST;
			speed_right = SPEED_SLOW;
			break;
		case 3:
			speed_left = 1.01*SPEED_FAST;
			speed_right = 0.09*SPEED_FAST;
			break;
		default:
			return crazy();
		}
	} else {
		//! Measure brightness
		photo_left = measure(SENSOR_LEFT);
		photo_right = measure(SENSOR_RIGHT);
	
		photo_ave = 0.5 * photo_left + 0.5 * photo_right;
		photo_tot = photo_left + photo_right;
	
		switch (g_mode) {
	
		case MODE_MOTH:
			speed_factor = -1.0;
			break;
	
		case MODE_BUG:
			speed_factor = 1.0;
			break;
		}
	
		//! Correct direction
		speed_left += speed_factor * ((double)photo_left - (double)photo_ave) / (double)photo_ave;
		speed_right += speed_factor * ((double)photo_right - (double)photo_ave) / (double)photo_ave;
	}

	move(SERVO_LEFT, speed_left);
	move(SERVO_RIGHT, speed_right);
}

int sleep(void)
{
	move(SERVO_LEFT, 0.0);
	move(SERVO_RIGHT, 0.0);

	_delay_ms(10);
}

int main(void)
{
	int i;

	init();

	while(1){
		if (SENSOR_PIN &= 1<<SENSOR_GROUND)
			sleep();
		else
			work();
	}
}

