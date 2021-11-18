#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/**************************************************************************************/
/************************ GLOBAL VARIABLE DEFINITIONS *********************************/
/**************************************************************************************/

/**************************************************************************************/

//ADC global variables
volatile unsigned int ADC_result_flag;
volatile uint16_t ADC_result = 0b1111111111;

//DC motor global variables
volatile unsigned char kill = 0x01;

//RL sensor global variables
volatile unsigned int read_RL = 0x00;
volatile uint16_t peak = 0b1111111111;

/**************************************************************************************/
/**************************** FUNCTION DEFINITIONS ************************************/
/**************************************************************************************/

/**************************************************************************************/

//mTimer: delays the program by 1 * count miliseconds
void mTimer(int count);
void mTimer(int count){
	int i = 0;
	
	TCCR1B|=_BV(CS10); //sets prescaler to 1
	TCCR1B |=_BV(WGM12); //sets mode to CTC and compare TCNT1 to OCR1A
	OCR1A = 0x03e8; //sets OCR1A register (16bit) to 1000 in hexadecimal. OCR1A is always comapred to TCNT1, when they match an interupt is triggered
	TCNT1 = 0x0000; //sets TCNT1 register (16bit) to 0 in hexadecimal. This register counts clock ticks
	//TIMSK1 = TIMSK1|0b00000010; //commented out if sei() and cli() used in main, to prevent infinite loop forming
	TIFR1 |=_BV(OCF1A); //clears inturupt flag
	while(i<count){
		if((TIFR1 & 0x02)==0x02){
			TIFR1|=_BV(OCF1A); //if statement will execute everytime one 1 milisecond is counted and clears inturpt flag
			i++; //adds 1 to the i varible indicating 1milisecond has passed
		}
	}
	return;
}

//This function initializes the Pulse Width Modulation signal, the PWM signal is outputed to PORT B PIN 7
void initPWM();
void initPWM(){
	//step 1. Fast pwm mode with TOP equal to the max value, output compare register updated at top, tome overflow should be max
	TCCR0A|=_BV(WGM00);
	TCCR0A|=_BV(WGM01);
	
	//step 2. Enable output compare interrupt
	//TIMSK0|=_BV(OCIE0A); //commented out if sei() and cli() used in main, to prevent infinite loop forming
	
	//step 3. Set compare match output mode to clear on compare and set out
	TCCR0A|=_BV(COM0A1);
	
	//step 4. Set prescaller to 1/8
	TCCR0B|=_BV(CS01);

	//step 5. Set duty cycle
	OCR0A = 0b01000000; //comment out if using POT output as duty cycle
	
	//Step 6. OC0A output is on PORT B PIN 7
	
	//Step 7. 
	//cli(); //comment out if sei() and cli() are in main

}

void initINT(){
	EIMSK|=(_BV(INT0)); //enable external interrupt 2, located on PORT D PIN 0 this is pause switch for DC motor
	EICRA|=(_BV(ISC01)|_BV(ISC00)); //detect a rising edge on INT0

    EIMSK|=(_BV(INT1)); //enable external interrupt 2, located on PORT D PIN 1 this is OR sensor
	EICRA|=(_BV(ISC10)); //detect any edge on INT0
}

void init_RL_ADC();
void init_RL_ADC()
{
	ADCSRA|=_BV(ADEN); //enabling the ADC
	ADCSRA|=_BV(ADIE); //enable ADC interrupt flag 
	//ADMUX|=_BV(ADLAR); //ADC left adjust, commented out since we are using 10 bit ADC
	ADMUX|=_BV(REFS0); //Using aVcc as reference voltage
	ADMUX|=_BV(MUX0); //commented out bit write, ADC takes input from PORT F PIN 1
}

//compares the current peak value to the ADC_result value and sets peak to the next lowest value.
void findPeak();
void findPeak()
{
	if(peak > ADC_result){
		peak = ADC_result;
	}
}

//outputs the peak value to the led when there is nothing 
void display();
void display(){
		PORTC = peak; //Output ADC output to LED bank in PORT C
   		PORTD = (peak & 0b1100000000)>>3; //Output ADC output to LED bank in PORT D
}

//read_ADC reads the 10 bit binary number after the ADC is complete
void read_ADC();
void read_ADC(){
	if(ADC_result_flag)
		{
			ADC_result_flag = 0x0; //Clear ADC results
			findPeak();
			display();
			ADCSRA|=_BV(ADSC);
		}
}

/**************************************************************************************/
/********************************** MAIN **********************************************/
/**************************************************************************************/

/**************************************************************************************/

int main()
{
	DDRB = 0xFF; //PORT B has outputs needed for DC motor
	DDRC = 0xFF; //PORT C controls the LED bank to observe the value of the ADC
	DDRD = 0b11110000; //PORT D has LEDs used to indicate the status of the DC Motor and also the pins that trigger interrupt 0 and 2
	DDRA = 0xFF;
    DDRF = 0x00;

	cli(); //disable global interrupts
	initPWM(); //start PWM signal on PORT B PIN 7
    initINT(); //initiate inturpts used for this test
    init_RL_ADC(); //initiate 10 bit ADC for RL sensor
	sei(); // enable global interrupts

	
	while(1)
	{
		if(read_RL == 1){
				read_ADC();
		}
	} 
}

/**************************************************************************************/
/******************** SUBROUTINES FOR INTERRUPTS ************************************/
/**************************************************************************************/

/**************************************************************************************/

ISR(ADC_vect){
    ADC_result =  ADCL | ADCH << 8 ; //Output the results to ADC_result, this will clear ADCH
	ADC_result_flag = 0x1; //sets ADC_result_flag to 1
}

//This subroutine will trigger when interrupt 2 is fired.
//This interrupt will kill all power to the DC motor
ISR(INT0_vect)
{
    if(kill == 0x00){
	    kill = 0x01; //kill all power to the motor driver
		PORTB = 0b0000;
    }
    else{
        kill = 0x00;
		PORTB = 0b1000;
    }
	mTimer(20); //debounce
}

ISR(INT1_vect){
    if(read_RL == 0x00){
	    read_RL = 0x01; //kill all power to the motor driver
        peak = 0b1111111111;
		ADCSRA|=_BV(ADSC);
    }
    else{
        read_RL = 0x00;
    }
}




