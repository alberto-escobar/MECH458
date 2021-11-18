/**************************************************************************************
MECH 458 Final Project Code

Alberto Esobar V00822248
Houman Haghnazari V00846206

Demo Day: December 14, 2019
At station 5

Notes:
PWM set to 0b01010111
using austin's method for stepper acceleration
***************************************************************************************/

/**************************************************************************************/
/******************************* LIBRARIES ********************************************/
/**************************************************************************************/

/**************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "LinkedQueue.h"

/**************************************************************************************/
/************************ GLOBAL VARIABLE DEFINITIONS *********************************/
/**************************************************************************************/

/**************************************************************************************/
//Flags
volatile unsigned char calibrationFlag;
volatile unsigned char ADC_result_flag;
volatile unsigned char readFlag = 0x00;
volatile unsigned char sortFlag = 0x00;

volatile unsigned char rampDownFlag = 0x00;
volatile unsigned char pauseFlag = 0x01;

//Global timer
volatile unsigned int globalmTimer = 0;
volatile unsigned int globalsTimer = 0;

//RL sensor and ADC global variables
volatile uint16_t ADC_result = 0b1111111111;
volatile uint16_t peak = 0b1111111111;

//stepper motor global variables
volatile int step = 1;
volatile unsigned char bucketPosition;

//temporary variable used for sorting alorithm
volatile unsigned char object = 0;

//Linked Queue variables
link *head, *tail;
link *newLink, *rtnLink;

//varibles for determining number of parts identified
volatile int sortedBlack = 0;
volatile int sortedSteel = 0;
volatile int sortedWhite = 0;
volatile int sortedAluminium = 0;
volatile int onBelt = 0;

//Calibration standards
int black = 960;
int steel = 608;
int white = 914;
int aluminium = 104;

//Stepper delay acceleration profiles
int delay50[50] =
					{20,20,19,18,16,15,14,13,12,11,
					10,9,8,7,7,6,6,5,5,5,
				    5,5,5,5,5,5,5,5,5,5,
					5,5,5,6,6,7,7,8,9,10,
					11,12,13,14,15,16,18,19,20,20};

int delay100[100] =
					{20,20,19,18,16,15,14,13,12,11,
					10,9,8,7,7,6,6,5,5,5,
				   	5,5,5,5,5,5,5,5,5,5,
				   	5,5,5,5,5,5,5,5,5,5,
				   	5,5,5,5,5,5,5,5,5,5,
				   	5,5,5,5,5,5,5,5,5,5,
				   	5,5,5,5,5,5,5,5,5,5,
				   	5,5,5,5,5,5,5,5,5,5,
					5,5,5,6,6,7,7,8,9,10,
					11,12,13,14,15,16,18,19,20,20};
/**************************************************************************************/
/**************************** Acceleration Profiles ************************************/
/**************************************************************************************/

/**************************************************************************************/
/*

//slow linear, slop of 0.5
					{20,20,19,19,18,18,17,17,16,16,
					15,14,13,12,11,10,9,8,7,6,
					5,5,5,5,5,5,5,5,5,5,
					6,7,8,9,10,11,12,13,14,15,
					16,16,17,17,18,18,19,19,20,20};

//normal linear, slop of 1
					{20,20,19,18,17,16,15,14,13,12,
					11,10,9,8,7,6,5,5,5,5,
					5,5,5,5,5,5,5,5,5,5,
					5,5,5,5,6,7,8,9,10,11,
					12,13,14,15,16,17,18,19,20,20};

//austin's method	
					{20,20,18,18,16,16,14,14,12,12,
					10,10,8,8,6,6,5,5,5,5,
					5,5,5,5,5,5,5,5,5,5,
					5,5,5,5,6,6,8,8,10,10,
					12,12,14,14,16,16,18,18,20,20};

//S curve start from 20
					{20,20,19,18,16,15,14,13,12,11,
					10,9,8,7,7,6,6,5,5,5,
					5,5,5,5,5,5,5,5,5,5,
					5,5,5,6,6,7,7,8,9,10,
					11,12,13,14,15,16,18,19,20,20};

//S curve start from 30
					{30,27,24,22,19,17,14,12,10,9,
					7,6,6,5,5,5,5,5,5,5,
					5,5,5,5,5,5,5,5,5,5,
					5,5,5,5,5,5,5,6,6,7,
					9,10,12,14,17,19,22,24,27,30};

*/
/**************************************************************************************/
/**************************** FUNCTION DEFINITIONS ************************************/
/**************************************************************************************/

/**************************************************************************************/

//mTimer: Delays the program by 1 * count miliseconds.
void mTimer(int count);
void mTimer(int count){
	int i = 0;
	
	TCCR1B|=_BV(CS10); //sets prescaler to 1
	TCCR1B |=_BV(WGM12); //sets mode to CTC and compare TCNT1 to OCR1A
	OCR1A = 0x03e8; //sets OCR1A register (16bit) to 1000 in hexadecimal. OCR1A is always comapred to TCNT1, when they match an interupt is triggered
	TCNT1 = 0x0000; //sets TCNT1 register (16bit) to 0 in hexadecimal. This register counts clock ticks
	//TIMSK1 = TIMSK1|0b00000010; //Enable interrupt. commented out if sei() and cli() used in main, to prevent infinite loop forming
	TIFR1 |=_BV(OCF1A); //clears inturupt flag
	while(i<count){
		if((TIFR1 & 0x02)==0x02){
			TIFR1|=_BV(OCF1A); //if statement will execute everytime one 1 milisecond is counted and clears inturpt flag
			i++; //adds 1 to the i varible indicating 1milisecond has passed
		}
	}
	return;
}

//initPWM: This function initializes the Pulse Width Modulation signal, the PWM signal is outputed to PORT B PIN 7.
void initPWM();
void initPWM(){
	TCCR0A|=_BV(WGM00); //Fast pwm mode with TOP equal to the max value, output compare register updated at top, tome overflow should be max
	TCCR0A|=_BV(WGM01);

	//TIMSK0|=_BV(OCIE0A); //Enable output compare interrupt. commented out if sei() and cli() used in main, to prevent infinite loop forming
	
	TCCR0A|=_BV(COM0A1); //Set compare match output mode to clear on compare and set out
	TCCR0B|=_BV(CS01); //Set prescaller to 1/8
	
	OCR0A = 0b01000000;  //set duty cycle. comment out if using POT output as duty cycle
	
	//OC0A (PWM signal) output is on PORT B PIN 7
}

//initINT: Initalizes the external interrupts.
void initINT(){
	EIMSK|=(_BV(INT0)); //enable external interrupt 0, located on PORT D PIN 0 this is for pause switch
	EICRA|=(_BV(ISC01)|_BV(ISC00)); //detect a rising edge on INT0

    EIMSK|=(_BV(INT1)); //enable external interrupt 1, located on PORT D PIN 1 this is for OR sensor
	EICRA|=(_BV(ISC10)); //detect any edge on INT1

	EIMSK|=(_BV(INT2)); //enable external interrupt 2, located on PORT D PIN 2 this is for HE sensor
	EICRA|=(_BV(ISC21)); //detect falling edge on INT2
	
	EIMSK|=(_BV(INT3)); //enable external interrupt 3, located on PORT D PIN 3 this is for EX sensor
	EICRA|=(_BV(ISC31)); //detect falling edge on INT3

	EIMSK|=(_BV(INT4)); //enable external interrupt 0, located on PORT D PIN 0 this is for pause switch
	EICRB|=(_BV(ISC41)|_BV(ISC40)); //detect a rising edge on INT0
}

//initRL: Initalizes the ADC to read the RL analog signal.
void initRL();
void initRL()
{
	ADCSRA|=_BV(ADEN); //enabling the ADC
	ADCSRA|=_BV(ADIE); //enable ADC interrupt flag 
	
	//ADMUX|=_BV(ADLAR); //ADC left adjust. Commented out since we are using 10 bit ADC
	
	ADMUX|=_BV(REFS0); //Using aVcc as reference voltage
	ADMUX|=_BV(MUX0); //commented out bit write, ADC takes input from PORT F PIN 1
}

//findPeak: Compares the current peak value to ADC_result= and sets peak to the lowest value between the two variables.
void findPeak();
void findPeak()
{
	if(peak > ADC_result){
		peak = ADC_result;
	}
}

//readRL: Processes ADC conversion results.
void readRL();
void readRL(){
	if(ADC_result_flag)
		{
			ADC_result_flag = 0x0; //Clear ADC results
			findPeak();
			ADCSRA|=_BV(ADSC);
	}
}

//findMin: Finds the smallest value in an array and outputs the index + 1 where the smallest value is on the array.
int findMin(int ar[]);
int findMin(int ar[]){
int min = 0;

for(int i = 1; i < 4; i++){
		if(ar[min] > ar[i]){
			min = i;
		}
	}
	return min + 1;
}

//determineObject: Identifies the object that is at the RL sensor using multiclass identification algorithm
void determineObject();
void determineObject(){
	int diffblack = abs((int)peak - black);
	int diffsteel = abs((int)peak - steel);
	int diffwhite = abs((int)peak - white);
	int diffaluminium = abs((int)peak - aluminium);
	int array[4] = {diffblack,diffsteel,diffwhite,diffaluminium};
	object = (unsigned char)findMin(array);
	if((int)peak < (white - 100) && (int)peak > (aluminium + 100)){
		object = 0b10;
	}
	//else{}

}

//moveStepper: Basic function for moving the stepper and has options for delay, number of steps,and cw or ccw. Uses linear aceleration.
void moveStepper(int ccw, int steps, int delay);
void moveStepper(int ccw, int steps, int delay){ // this function moves the stepper motor
	int count = 0;
	while(count < steps){
		switch(step){ //since there are four step configurations for the stepper motor, a switch case statement helps move the stepper
			case 1:
				PORTA = 0b110110;
				step = 2;
				count++;
				mTimer(delay);
				if(ccw == 1){ //this helps reverse the movement of the stepper if running counter clock wise
					step = 4;
				}
				break;
			case 2:
				PORTA = 0b101110;
				step = 3;
				count++;
				mTimer(delay);
				if(ccw == 1){
					step = 1;
				}
				break;
			case 3:
				PORTA = 0b101101;
				step = 4;
				count++;
				mTimer(delay);
				if(ccw == 1){
					step = 2;
				}
				break;
			case 4:
				PORTA = 0b110101;
				step = 1;
				count++;
				mTimer(delay);
				if(ccw == 1){
					step = 3;
				}
				break;
			default:
				break;
		}

		if(count > (steps - 10)){
			delay = delay + 2;
		}
		else{
			if(delay > 5){
				delay = delay - 1;
			}
		}
	}
}

//moveStepper50: moves the stepper 50 steps using the delay50 array.
void moveStepper50(int ccw, int steps);
void moveStepper50(int ccw, int steps){ // this function moves the stepper motor
	int count = 0;
	while(count < steps){
		switch(step){ //since there are four step configurations for the stepper motor, a switch case statement helps move the stepper
			case 1:
				PORTA = 0b110110;
				step = 2;
				count++;
				mTimer(delay50[count]);
				if(ccw == 1){ //this helps reverse the movement of the stepper if running counter clock wise
					step = 4;
				}
				break;
			case 2:
				PORTA = 0b101110;
				step = 3;
				count++;
				mTimer(delay50[count]);
				if(ccw == 1){
					step = 1;
				}
				break;
			case 3:
				PORTA = 0b101101;
				step = 4;
				count++;
				mTimer(delay50[count]);
				if(ccw == 1){
					step = 2;
				}
				break;
			case 4:
				PORTA = 0b110101;
				step = 1;
				count++;
				mTimer(delay50[count]);
				if(ccw == 1){
					step = 3;
				}
				break;
			default:
				break;
		}
	}
}

//moveStepper100: moves the stepper 100 steps using the delay100 array.
void moveStepper100(int ccw, int steps);
void moveStepper100(int ccw, int steps){ // this function moves the stepper motor
	int count = 0;
	while(count < steps){
		switch(step){ //since there are four step configurations for the stepper motor, a switch case statement helps move the stepper
			case 1:
				PORTA = 0b110110;
				step = 2;
				count++;
				mTimer(delay100[count]);
				if(ccw == 1){ //this helps reverse the movement of the stepper if running counter clock wise
					step = 4;
				}
				break;
			case 2:
				PORTA = 0b101110;
				step = 3;
				count++;
				mTimer(delay100[count]);
				if(ccw == 1){
					step = 1;
				}
				break;
			case 3:
				PORTA = 0b101101;
				step = 4;
				count++;
				mTimer(delay100[count]);
				if(ccw == 1){
					step = 2;
				}
				break;
			case 4:
				PORTA = 0b110101;
				step = 1;
				count++;
				mTimer(delay100[count]);
				if(ccw == 1){
					step = 3;
				}
				break;
			default:
				break;
		}
	}
}

//initStepper: this function initalizes the stepper motor and homes it
void initStepper();
void initStepper(){ //this function initalizies movement for the stepper. This function will move the stepp 90 degrees clockewise then 90 degrees counter clockwise
	moveStepper(0,200,20);
	mTimer(500);
	moveStepper(1,200,20);
    bucketPosition = 0;
	calibrationFlag = 0x01;
	while(bucketPosition != 1){
		moveStepper(0,1,20);
	}
}

//moveTray: this function is used to move the bucket to the position for sorting the object at the end of the belt.
void moveTray(char newBucketPosition);
void moveTray(char newBucketPosition)
{
	switch(bucketPosition){
		case 1:
			switch(newBucketPosition){
				case 1:
					break;
				case 2:
					moveStepper50(1,50);
					break;
				case 3:
					moveStepper100(0,100);
					break;
				case 4:
					moveStepper50(0,50);
					break;
			}
			break;
		case 2:
			switch(newBucketPosition){
				case 1:
					moveStepper50(0,50);;
					break;
				case 2:
					break;
				case 3:
					moveStepper50(1,50);
					break;
				case 4:
					moveStepper100(0,100);
					break;
			}
			break;
		case 3:
			switch(newBucketPosition){
				case 1:
					moveStepper100(0,100);
					break;
				case 2:
					moveStepper50(0,50);
					break;
				case 3:
					break;
				case 4:
					moveStepper50(1,50);
					break;
			}
			break;
		case 4:
			switch(newBucketPosition){
				case 1:
					moveStepper50(1,50);
					break;
				case 2:
					moveStepper100(0,100);
					break;
				case 3:
					moveStepper50(0,50);
					break;
				case 4:
					break;
			}
			break;
	}
	bucketPosition = newBucketPosition;
}

//countIdentified: this function tallies the identified object that passed the RL sensor
void countIdentified(char item);
void countIdentified(char item){
	if(item == 0b001){
		sortedBlack = sortedBlack + 1;
	}
	else if(item == 0b010){
		sortedSteel = sortedSteel + 1;
	}
	else if(item == 0b011){
		sortedWhite = sortedWhite + 1;
	}
	else if(item == 0b100){
		sortedAluminium = sortedAluminium + 1;
	}
	else{}
}

/**************************************************************************************/
/********************************** MAIN **********************************************/
/**************************************************************************************/

/**************************************************************************************/

int main()
{
	DDRA = 0xFF; //PORT A has outputs needed for stepper
	DDRB = 0xFF; //PORT B has outputs needed for DC motor
	DDRC = 0xFF; //PORT C controls the LED bank to observe the value of the ADC
	DDRD = 0b11110000; //PORT D has LEDs used to indicate the status and also the pins that trigger interrupt 0 - 3
    DDRF = 0x00; //PORT F is input for ADC 
	DDRE = 0x00; //PORTE is for interrupt 4
	
	setup(&head, &tail);
	
	cli(); //disable global interrupts
	initPWM(); //initate PWM
    initINT(); //initiate inturpts
    initRL(); //initiate 10 bit ADC
	sei(); // enable global interrupts
	
	//Stepper motor's dance of good luck and high accuracy
	initStepper();
	moveStepper50(1,50);
	moveStepper100(1,100);
	moveStepper50(0,50);
	moveStepper100(0,100);

	PORTD = 0b00100000; //green light to indicate ready for operation 
	
	while(1)
	{
		//Ramp Down Mode
		if(rampDownFlag == 0x01){ 
			PORTD = 0b10000000;
			globalmTimer++;			
			
			if(globalmTimer > 1000){
				globalmTimer = 0;
				globalsTimer++;
			}
			else{}

			if(globalsTimer > 50){
				
				if(size(&head,&tail) == 0){
					mTimer(10);
					PORTB = 0b0110;
					
					while(1){
						PORTC = 0b00010000 | (unsigned char)sortedBlack;
						mTimer(500);
						PORTC = 0b00100000 | (unsigned char)sortedSteel;
						mTimer(500);
						PORTC = 0b01000000 | (unsigned char)sortedWhite;
						mTimer(500);
						PORTC = 0b10000000 | (unsigned char)sortedAluminium;
						mTimer(500);
						PORTC = 0b11110000 | (unsigned char)onBelt;
						mTimer(500);		
					}
				}
				else{}

			}
			else{}
				
		}
		else{}

		//Pause Mode
		if(pauseFlag == 0x01){ 
			PORTD = 0b01000000;
			
			PORTC = 0b00010000 | (unsigned char)sortedBlack;
			mTimer(500);
			PORTC = 0b00100000 | (unsigned char)sortedSteel;
			mTimer(500);
			PORTC = 0b01000000 | (unsigned char)sortedWhite;
			mTimer(500);
			PORTC = 0b10000000 | (unsigned char)sortedAluminium;
			mTimer(500);
			PORTC = 0b11110000 | (unsigned char)onBelt;
			mTimer(500);
			
			if(pauseFlag == 0x00){
				PORTC = 0b0;
				PORTD = 0;
				PORTB = 0b1000;
			}
		}
		else{}

		//Inspection mode
		if(readFlag == 1){ 
			readRL();
			determineObject();
		}
		else{}
		
		//Sorting Mode
		if(sortFlag == 0x01){ 
			countIdentified(firstValue(&head).itemCode);
			onBelt--;
			if(firstValue(&head).itemCode != bucketPosition){
				PORTB = 0b0000;
				mTimer(5);
				PORTB = 0b0001;
    			mTimer(12);
    			PORTB = 0b0000;
				moveTray(firstValue(&head).itemCode);
    			PORTB = 0b1000;
			}
			else{}
	
			if(size(&head,&tail) == 0b1){
        		clearQueue(&head,&tail);
    		}
    		else{
        		dequeue(&head, &tail, &newLink);
    		}
			
			sortFlag = 0x00;
		}
		else{}

	} 
}


/**************************************************************************************/
/******************** SUBROUTINES FOR INTERRUPTS ************************************/
/**************************************************************************************/

/**************************************************************************************/
//ADC conversion complete subroutine: Stores 10 bit ADC value and sets ADC flag on
ISR(ADC_vect){
    ADC_result =  ADCL | ADCH << 8 ; //Output the results to ADC_result, this will clear ADCH
	ADC_result_flag = 0x1; //sets ADC_result_flag to 1
}

//Pause button subroutine: sets pause flag and kills power to motor if MCU is entering pause mode
ISR(INT0_vect)
{
    if(pauseFlag == 0x00){
	    pauseFlag = 0x01;
		PORTB = 0b0000;
    }
    else{
        pauseFlag = 0x00;
		//PORTB = 0b1000;
    }
	mTimer(20); //debounce
}

//OR sensor subroutine: If laser beam broken, initates the ADC for reading the RL sensor. If laser beam is restored then object just inspected is added to queue
ISR(INT1_vect){
    if(readFlag == 0x00){
	    readFlag = 0x01;
        peak = 0b1111111111;
        PORTD = 0b00010000;
        ADCSRA|=_BV(ADSC);
    }
    else{
        readFlag = 0x00;
        PORTD = 0b00100000;
        initLink(&newLink);
        newLink->e.itemCode = (object);
        enqueue(&head,&tail,&newLink);
		onBelt++;
    }
}

//HE sensor subroutine: If HE sensor is triggered, checks if MCU is in calibration mode for stepper and sets the bucket flag to home.
ISR(INT2_vect){
	if(calibrationFlag == 0x01){
		bucketPosition = 1;
		calibrationFlag = 0x00;
	}
}

//EX sensor subroutine: If EX sensor is triggered, sets sort flag to intiate sorting of object to bucket
ISR(INT3_vect){
	sortFlag = 0x01;
}

//Ramp down button subroutine: Sets flag for MCU to enter ramp down mode
ISR(INT4_vect){
	rampDownFlag = 0x01;
	mTimer(20); //debounce
}

//BAD ISR subsroutine: Implemented as specified by Patrick
ISR(BADISR_vect){
	PORTC = 0b10101010;
}

/**************************************************************************************/
/******************** SUBROUTINES FOR LINKED QUEUE ************************************/
/**************************************************************************************/

/**************************************************************************************
* DESC: initializes the linked queue to 'NULL' status
* INPUT: the head and tail pointers by reference
*/

void setup(link **h,link **t){
	*h = NULL;		/* Point the head to NOTHING (NULL) */
	*t = NULL;		/* Point the tail to NOTHING (NULL) */
	return;
}/*setup*/




/**************************************************************************************
* DESC: This initializes a link and returns the pointer to the new link or NULL if error 
* INPUT: the head and tail pointers by reference
*/
void initLink(link **newLink){
	//link *l;
	*newLink = malloc(sizeof(link));
	(*newLink)->next = NULL;
	return;
}/*initLink*/




/****************************************************************************************
*  DESC: Accepts as input a new link by reference, and assigns the head and tail		
*  of the queue accordingly				
*  INPUT: the head and tail pointers, and a pointer to the new link that was created 
*/
/* will put an item at the tail of the queue */
void enqueue(link **h, link **t, link **nL){

	if (*t != NULL){
		/* Not an empty queue */
		(*t)->next = *nL;
		*t = *nL; //(*t)->next;
	}/*if*/
	else{
		/* It's an empty Queue */
		//(*h)->next = *nL;
		//should be this
		*h = *nL;
		*t = *nL;
	}/* else */
	return;
}/*enqueue*/




/**************************************************************************************
* DESC : Removes the link from the head of the list and assigns it to deQueuedLink
* INPUT: The head and tail pointers, and a ptr 'deQueuedLink' 
* 		 which the removed link will be assigned to
*/
/* This will remove the link and element within the link from the head of the queue */
void dequeue(link **h, link **t, link **deQueuedLink){
	/* ENTER YOUR CODE HERE */
	*deQueuedLink = *h;	// Will set to NULL if Head points to NULL
	/* Ensure it is not an empty queue */
    if (*h != NULL){
		*h = (*h)->next; 
	}/*if*/

	return;
}/*dequeue*/




/**************************************************************************************
* DESC: Peeks at the first element in the list
* INPUT: The head pointer
* RETURNS: The element contained within the queue
*/
/* This simply allows you to peek at the head element of the queue and returns a NULL pointer if empty */
element firstValue(link **h){
	return((*h)->e);
}/*firstValue*/





/**************************************************************************************
* DESC: deallocates (frees) all the memory consumed by the Queue
* INPUT: the pointers to the head and the tail
*/
/* This clears the queue */
void clearQueue(link **h, link **t){

	link *temp;

	while (*h != NULL){
		temp = *h;
		*h=(*h)->next;
		free(temp);
	}/*while*/
	
	/* Last but not least set the tail to NULL */
	*t = NULL;		

	return;
}/*clearQueue*/





/**************************************************************************************
* DESC: Checks to see whether the queue is empty or not
* INPUT: The head pointer
* RETURNS: 1:if the queue is empty, and 0:if the queue is NOT empty
*/
/* Check to see if the queue is empty */
char isEmpty(link **h){
	/* ENTER YOUR CODE HERE */
	return(*h == NULL);
}/*isEmpty*/





/**************************************************************************************
* DESC: Obtains the number of links in the queue
* INPUT: The head and tail pointer
* RETURNS: An integer with the number of links in the queue
*/
/* returns the size of the queue*/
char size(link **h, link **t){

	link 	*temp;			/* will store the link while traversing the queue */
	char 	numElements;

	numElements = 0;

	temp = *h;			/* point to the first item in the list */

	while(temp != NULL){
		numElements = numElements + 0b1;
		temp = temp->next;
	}/*while*/
	
	return(numElements);
}/*size*/
