#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define F_CPU 16000000

//Left motor operated internally by 3 inputs. Two control inputs and one PWM signal defined below as follows.
#define L_CTRL_1 PIND2
#define L_CTRL_2 PIND4
#define PWM_L PIND5 //OC0B

//Same as the left motor
#define R_CTRL_1 PIND7
#define R_CTRL_2 PINB0
#define PWM_R PIND6 //OC0A


//Definitions that allow us to more easily select which sensor we are targeting.
#define IR_LEFT 5
#define IR_RIGHT 3
#define OPTICAL_LEFT 4
#define OPTICAL_RIGHT 2
#define OPTICAL_FRONT 1

//Function prototypes for assisting subroutines.
void initializeMotors();
void initializePWM();
void initializePWM();
void initializeADC();
void startConversion();
void selectChannel(uint8_t analogPin);
uint16_t readSensor(uint8_t analogPin);

void analogWrite(uint8_t pin, uint8_t dutyCycle);


//Motor subroutines supporting CW and CCW rotation, coasting, braking and pivoting.
void rightFwd(uint8_t spd);
void rightRev(uint8_t spd);
void leftFwd(uint8_t spd);
void leftRev(uint8_t spd);
void rightCoast();
void leftCoast();
void rightStop();
void leftStop();
void rightBrake();
void leftBrake();
void rightDrive(int speed);
void leftDrive(int speed);
void rightMotor(int speed);
void leftMotor(int speed);
void pivot(int speed);
void drive(int speed);
void brake();
void coast();
void stop();


//Array of unsigned integers used to store up to 6 different sensor values located on different analog channels.
volatile uint16_t sensorValues[6];

//Initial channel set to Infra red sensor right.
volatile uint8_t currentAnalogChannel = OPTICAL_FRONT;

//Interrupt service routine to update the sensor value array continuously using interrupts
ISR(ADC_vect){
	
	//There is a simple logic change that would allow up to 6 sensors 0-5 analog pins
	
	if(currentAnalogChannel > IR_LEFT){
		currentAnalogChannel = OPTICAL_FRONT;
		} else {}
			
	sensorValues[currentAnalogChannel] = ADCW;
	
	
	
	selectChannel(currentAnalogChannel++);
	startConversion();
}

int main(void)
{
	initializeMotors();
	initializeADC();

	while (1)
	{
		/*drive(100);
		_delay_ms(16000);
		drive(-100);
		_delay_ms(16000);*/
		readSensor(IR_LEFT);
		readSensor(IR_RIGHT);
	}
	
	return 0;
}


//Changes the contents of the relevant registers to allow for PWM on 2 separate pins that serve as input for the motor-driver module.
void initializePWM(){
	
	DDRD |= (1 << PWM_L) | (1 << PWM_R);
	
	OCR0A = 0;
	OCR0B = 0;

	// set none-inverting mode
	TCCR0A |= (1 << COM0A1);
	TCCR0A |= (1 << COM0B1);

	// set fast PWM Mode
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	
	
	// set prescaler to 8 and starts PWM
	//Temp disabled prescaler
	TCCR0B |= (1 << CS02) | (1 << CS00);
}

//initializeMotors is a convenient subroutine which can be called in order set the required pins to output and initializes PWM with the expectation
//that the motors should not work if PWM has not been initialized.
void initializeMotors(){
	initializePWM();
	
	DDRD |= (1 << L_CTRL_1) | (1 << L_CTRL_2) | (1 << R_CTRL_1);
	DDRB |= (1 << R_CTRL_2);
	
}

//Rotate the leftMotor CW
void leftFwd(uint8_t spd){
	PORTD |= (1 << L_CTRL_1);
	PORTD &= ~(1 << L_CTRL_2);
	analogWrite(PWM_L, spd);
	
}

//Rotate the leftMotor CCW
void leftRev(uint8_t spd){
	PORTD &= ~(1 << L_CTRL_1);
	PORTD |= (1 << L_CTRL_2);
	analogWrite(PWM_L, spd);
}

//Rotate right motor CW
void rightFwd(uint8_t spd){
	PORTD |= (1 << R_CTRL_1);
	PORTB &= ~(1 << R_CTRL_2);
	analogWrite(PWM_R, spd);
}

//Rotate right motor CCW
void rightRev(uint8_t spd){
	PORTD &= ~(1 << R_CTRL_1);
	PORTB |= (1 << R_CTRL_2);
	analogWrite(PWM_R, spd);
}
//Stop left motor
void leftStop(){
	PORTD &= ~(1 << L_CTRL_1);
	PORTD &= ~(1 << L_CTRL_2);
	analogWrite(PWM_L, 0);
}

//Stop right motor
void rightStop(){
	PORTD &= ~(1 << R_CTRL_1);
	PORTB &= ~(1 << R_CTRL_2);
	analogWrite(PWM_R, 0);
}

//Stop both motors
void stop(){
	leftStop();
	rightStop();
}

//Takes in a value either positive or negative that will drive both motors at the same speed. If the value is negative the car will move CCW or in reverse
void drive(int speed){
	if(speed > 0){
		leftFwd((uint8_t) abs(speed));
		rightFwd((uint8_t) abs(speed));
	} else {
		leftRev((uint8_t) abs(speed));
		rightRev((uint8_t) abs(speed));
	}
}

	
//analogWrite works in conjunction with initializePWM where we enable timer0, OCR0B and OCR0A. YOU MUST INITIALIZE PWM BEFORE ANALOGWRITE WOULD WORK.
//This sub-routine defines the dutyCycle of the motors by writing to the respective timer0 registers which will drive the motors.
void analogWrite(uint8_t pin, uint8_t dutyCycle){
	
	//TODO: Do some checking so that value is between 0-255
	
	if(pin == PWM_L){
		OCR0B = dutyCycle;
	}else if(pin == PWM_R){
		OCR0A = dutyCycle;
	}else{
		//Do something weird
	}
}

//Initializes the ADC registers to support the IR and optical sensors.
void initializeADC(){
	//Configure the ADMUX for AVCC as input (internal 5v reference) / not left adjusted results (10-bit resolution) / ADC0 as input / 0b0100:0000
	ADMUX = 0x40;
	//Configure ADCSRA for enable ADC / don't start conversion / prescaler 128 / disable auto-trigger / enable ADC interrupt 0b10001:111
	ADCSRA = 0x8F;
	
	selectChannel(OPTICAL_FRONT);
	sei();
	startConversion();
}

//Takes an analog pin input as previously defined e.g. IR_LEFT IR_RIGHT to initialize the ADMUX with the proper channel
void selectChannel(uint8_t analogPin){
	ADMUX = (ADMUX & 0xE0) | (analogPin & 0x1F);   //select channel (MUX0-4 bits)
}

//Read sensor takes in the latest value stored in the sensors array at the specified index and returns it. This value is updated via the ADC_vect interrupt. 
uint16_t readSensor(uint8_t analogPin){
	return sensorValues[analogPin];
}

//For convenience startConversion is used instead of the bit maths. 
void startConversion(){
	ADCSRA |= (1 << ADSC);
}

