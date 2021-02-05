/*
 * seven_segment.c
 *
 *  Created on: Feb 2, 2021
 *      Author: Sarah Hejazy
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
long long X = F_CPU;
unsigned char seconds = 0;
unsigned char minutes = 0;
unsigned char hours = 0;

void Timer_CRC_Init(void){
	TCCR1A = (1<<FOC1A) | (1<<FOC1B); // Non PWM
	TCCR1B = (1<<CS12) | (1<<CS10) | (1<<WGM12); //N = 1024 & enable compare mode
	TCNT1 = 0;//initial value
	OCR1A = 975; //Top value
	TIMSK = (1<<OCIE1A);//enable module interrupt
}

ISR(TIMER1_COMPA_vect){
	seconds++;
	if(seconds == 60){
		seconds = 0;
		minutes++;
	}
	if(minutes == 60){
		minutes = 0;
		hours++;
	}
	if(hours == 24){
		hours = 0;
	}
}

//reset the timer
void INT0_Init(void){
	DDRD &= ~(1<<PD2);//set as i/p pin
	PORTD |= (1<<PD2);//activate the internal pull up resistor
	GICR |= (1<<INT0);//enable interrupt 0
	MCUCR |= (1<<ISC01);//interrupt with the falling edge
}

ISR(INT0_vect){
	seconds = 0;
	minutes = 0;
	hours = 0;
	TCNT1 = 0;
	TCCR1B |= (1<<CS12)| (1<<CS10);//N = 1024
}

//pause the stop watch
void INT1_Init(void){
	DDRD &= ~(1<<PD3);//set as i/p pin
	GICR |= (1<<INT1);//enable interrupt 1
	MCUCR |= (1<<ISC11) | (1<<ISC10);//interrupt with the raising edge
}

ISR(INT1_vect){
	TCCR1B &= 0xf8;//set the first 3-bits
}

//resume the stop watch
void INT2_Init(void){
	DDRB &= ~(1<<PB2);//set as i/p pin
	PORTB |= (1<<PB2);//activate the internal pull up resistor
	GICR |= (1<<INT2);//enable interrupt 2
	MCUCR &= ~(1<<ISC2);//interrupt with the falling edge
}

ISR(INT2_vect){
	TCCR1B |= (1<<CS12) | (1<<CS10);//N = 1024
}


int main(void){
	INT0_Init();
	INT1_Init();
	INT2_Init();
	SREG = (1<<7);// enable I-bit
	Timer_CRC_Init();

	DDRC = 0x0f; //first 4-bits as output
	PORTC = 0; //start 7-segment is off

	DDRA = 0x3f;//first 6-bits as output
	PORTA = 0xff;//enable high

	while(1){
		PORTA = 0x01;
		PORTC = seconds%10;
		_delay_ms(3);

		PORTA = (PORTA<<1);
		PORTC =(seconds)/10;
		_delay_ms(3);

		PORTA = (PORTA<<1);
		PORTC = minutes%10;
		_delay_ms(3);

		PORTA = (PORTA<<1);
		PORTC =(minutes)/10;
		_delay_ms(3);

		PORTA = (PORTA<<1);
		PORTC = hours%10;
		_delay_ms(3);

		PORTA = (PORTA<<1);
		PORTC =(hours)/10;
		_delay_ms(3);
	}
}

