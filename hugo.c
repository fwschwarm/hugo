//! Fe's PHD hat application
//!
//! Using Atmega8 microcontroller

#define F_CPU           1000000	///< CPU frequency

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define GROUND_COUNT	1.5E-3 * F_CPU ///< Number of work function loops before giving up if ground contact is lost
#define EVASIVE_COUNT	1.5E-3 * F_CPU ///< Number of work function loops for evasive manouvers

#define SPEED_STOP	0.0	///< Stop motor
#define SPEED_SLOW	0.1	///< Slow speed
#define SPEED_BASE	0.15	///< Normal speed
#define SPEED_FAST	0.9	///< Fast speed

#define MODE_MOTH	0	///< Search for light places
#define MODE_BUG	1	///< Search for dark places

#define SERVO_DDR       DDRB	///< Data direction of servo ports
#define SERVO_PORT      PORTB	///< Servo port
#define SERVO_PIN       PINB	///< Servo pin
#define SERVO_LEFT      1	///< PWM pin left
#define SERVO_RIGHT     0	///< PWM pin right
//#define SERVO_LEFT_OFFSET   0.053	///< Speed offset left (0.023 -- 0.083 -> 0.053)
//#define SERVO_RIGHT_OFFSET -0.191	///< Speed offset right (-0.222 -- -0.16 -> -0.191)
#define SERVO_LEFT_OFFSET   0.095	///< Speed offset left (0.023 -- 0.083 -> 0.053)
#define SERVO_RIGHT_OFFSET -0.21	///< Speed offset right (-0.222 -- -0.16 -> -0.191)
#define SERVO_LEFT_FACTOR   0.76	///< Speed calibration left ()
#define SERVO_RIGHT_FACTOR  0.76	///< Speed calibration right (0.52,)

#define LED_DDR		SERVO_DDR	///< LED port is same as servo port
#define LED_PORT	SERVO_PORT	///< LED port is same as servo port
#define LED_PIN		SERVO_PIN	///< LED port is same as servo port
#define LED_FRONT	4	///< Front LED
 
#define SENSOR_DDR	DDRC	///< Data direction of servo ports
#define SENSOR_PORT	PORTC	///< Servo port
#define SENSOR_PIN	PINC	///< Servo pin
#define SENSOR_LEFT	1	///< Left photo resistor pin
#define SENSOR_RIGHT	0	///< Right photo resistor pin
#define SENSOR_GROUND	2	///< Ground switch pin (active low)
#define SENSOR_CRIGHT	4	///< Left collison switch pin (active low)
#define SENSOR_CLEFT	5	///< Right collison switch pin (active low)

int g_mode = MODE_MOTH;		///< Default mode

//! Turn on LED
void led_on()
{
	LED_PORT &= 1 << LED_FRONT;
}

//! Turn off LED
void led_off()
{
	LED_PORT |= ~(1 << LED_FRONT);
}

//! Turn off LED
void led_toggle()
{
	LED_PORT ^= 1 << LED_FRONT;
}

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

//! Set motor speed
//!
//! @param id    Motor [SERVO_LEFT|SERVO_RIGHT]
//! @param speed Speed [-1.0:1.0]
void move(int id, double speed)
{
	static double speed_left = -10.0, speed_right = -10.0;
	switch (id) {
	case SERVO_LEFT:
//		if (speed == SPEED_STOP) {
//			OCR1A = -1;
//		} else {
			if (speed != speed_left) {
				OCR1A = ICR1 * (1.5 - 0.5 * (SERVO_LEFT_FACTOR * speed - SERVO_LEFT_OFFSET)) / 20.0;
				speed_left = speed;
			}
//		}
		break;
	case SERVO_RIGHT:
//		if (speed == SPEED_STOP) {
//			OCR1B = -1;
//		} else {
			if (speed != speed_right) {
				OCR1B = ICR1 * (1.5 + 0.5 * (SERVO_RIGHT_FACTOR * speed - SERVO_RIGHT_OFFSET)) / 20.0;
				speed_right = speed;
			}
//		}
		break;
	}
}
 
//! Initialize registers
int init(void)
{
	int i;

	//! Motor PWM ports
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

	//! Activate the 8-bit timer
	TCCR0 = (1<<CS02)|(1<<CS00);         ///< divide by 1024

	//! Enable interupts
	sei();

	//! Turn off LED
	led_off();

	//! Stop motors
	move(SERVO_LEFT, SPEED_STOP);
	move(SERVO_RIGHT, SPEED_STOP);
}

//! Undefined behaviour
int crazy(void)
{
	return 0;
}

//! Stop motors and sleep for a moment
int sleep(void)
{
	move(SERVO_LEFT, SPEED_STOP);
	move(SERVO_RIGHT, SPEED_STOP);

	_delay_ms(10);
}

//! Main work function
int work(void)
{
	static uint32_t evasive_count = 0, ground_count = 0, lost_count = 0;
	static int evasive = 0, ground = 1, lost = 0;
	int bump_left = 0, bump_right = 0;
	uint16_t photo_left, photo_right, photo_ave;
	uint32_t photo_tot;
	double speed_left = SPEED_STOP, speed_right = SPEED_STOP;
	double speed_factor = 1.0;

	//! Check for ground contact
	if (SENSOR_PIN &= 1<<SENSOR_GROUND) {

		if (ground) {
			//! Lost ground contact
			lost_count = 30;
			lost = 1;
		}

		ground = 0;

		if (lost_count > 0) {
			lost_count--;
		} else if (lost) {
			evasive = 3;
			evasive_count = GROUND_COUNT;
			lost = 0;
		}
//			lost_count--;
//			evasive = 0;
//			ground = 0;
//		} else {
//			evasive = 3;
//			evasive_count = GROUND_COUNT;
//			ground = 0; 
//			led_off();
//		}

	} else {
		ground = 1;

		led_on();
	}

	//! Check for collision
	if (SENSOR_PIN &= 1<<SENSOR_CLEFT) {
		bump_left = 0;
	} else {
		move(SERVO_LEFT, SPEED_STOP);
		bump_left = 1;
		evasive = 1;
	}
	
	if (SENSOR_PIN &= 1<<SENSOR_CRIGHT) {
		bump_right = 0;
	} else {
		move(SERVO_RIGHT, SPEED_STOP);
		bump_right = 1;
		evasive = 2;
	}
	
	if (bump_left || bump_right)
		evasive_count = EVASIVE_COUNT;
	
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
			speed_left = 1.01 * SPEED_FAST;
			speed_right = 0.09 * SPEED_FAST;
			break;
		default:
			return crazy();
		}

	} else if (ground) {
		//! Measure brightness
		photo_left = measure(SENSOR_LEFT);
		photo_right = measure(SENSOR_RIGHT);
	
		photo_ave = 0.5 * photo_left + 0.5 * photo_right;
		photo_tot = photo_left + photo_right;
	
		switch (g_mode) {
	
		case MODE_MOTH:
			speed_factor = 1.0;
			break;
	
		case MODE_BUG:
			speed_factor = -1.0;
			break;
		}

		//! Set base speed
		speed_left = -SPEED_BASE;
		speed_right = -SPEED_BASE;

		//! Correct direction
		speed_left += speed_factor * ((double)photo_left - (double)photo_ave) / (double)photo_ave;
		speed_right += speed_factor * ((double)photo_right - (double)photo_ave) / (double)photo_ave;
	} else {
		sleep();
	}

	move(SERVO_LEFT, speed_left);
	move(SERVO_RIGHT, speed_right);

	return 0;
}

int main(void)
{
	int i;

	init();

	while(1)
		work();
//		sleep();
}

