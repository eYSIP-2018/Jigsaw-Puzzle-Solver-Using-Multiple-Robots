#define F_CPU 14745600
#define pi 3.1415
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
//#include <avr/lcd.h>
#include "lcd.h"

#include <math.h>
#include <stdlib.h>

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
int error_previous = 0;
int error_angle = 0;
int servo_angle = 0;
int align_angle = 0;
int forward_bit = 0;

int align = 0;
int servo_blocking = 0;
int stop_bit = 1;
volatile char data;
volatile int count = 0;
volatile int flag=0;
int number[4] = {0,0,0,0}; 
char str[200] ="AVR\0";

struct PID{
	int kp;
	int kd;
	int ki;
}pid;

struct cords{
	int x;
	int y;
}dest;

struct robot{
	struct cords head;
	struct cords tail;
}firebird;

volatile unsigned char LM = 124;
volatile unsigned char RM = 124;
void buzzer_config(void);
void buzzer_on(void);
void buzzer_off(void);


void buzzer_config(void){
	DDRC = DDRC | 0x08;
	PORTC = PORTC & 0xF7;
}

void buzzer_on(void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off(void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}


//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Initialize the ports
void port_init(void)
{
 servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
 servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to initialize all the peripherals

//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 



//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}

void forward (void)
{
  motion_set(0x06);
}

void back (void)
{
  motion_set(0x09);
}

void left (void)
{
  motion_set(0x05);
}

void right (void)
{
  motion_set(0x0A);
}

void stop (void)
{
  motion_set(0x00);
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}



//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}


void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}



//Function to initialize ports
void init_ports()
{
 motion_pin_config();
 lcd_port_config();
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config	

 
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}




void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

void init_devices(void)
{
 cli(); //disable all interrupts
 port_init();
 timer1_init();
 init_ports();
 timer5_init();
 uart0_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei(); //re-enable interrupts 
}

void linear_distance_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_distance_mm(DistanceInMM);
}
/*int get_angle(){
    float arucolength, tail_dist, head_dist;
    float m1, m2, angle, value;
	angle=0;
    arucolength = sqrt(pow(firebird.head.x - firebird.tail.x,2) + pow(firebird.head.y - firebird.tail.y,2));
    tail_dist = sqrt(pow(dest.x - firebird.tail.x,2) + pow(dest.y - firebird.tail.y,2));
    head_dist = sqrt(pow(dest.x - firebird.head.x,2) + pow(dest.y - firebird.head.y,2));

    m1 = (firebird.head.y - firebird.tail.y)/(firebird.head.x - firebird.tail.x);
    m2 = (dest.y - firebird.tail.y)/(dest.x - firebird.tail.x);
	
	value = (m2 - m1)/(1 + m1*m2 );

    if(abs(value) >=1)
		angle = atan(value) * (180/pi);
	else if (value < 0){
		value = (1 + m1*m2 )/(m2 - m1);
		angle= -90-(atan(value)*(180/pi));
	}
	else if(value>0){
		
	value = (1 + m1*m2 )/(m2 - m1);
	angle= 90-(atan(value)*(180/pi));
	}
	else if (value == 0)
	angle=0;
	

    if(head_dist > sqrt(pow(arucolength,2) + pow(tail_dist,2))){
        if(angle < 0)
            angle = 180  + angle;
        else if(angle > 0)
            angle = -180 + angle;
        else
            angle = 180;
    }

    return (int)angle;
}

int angle_optional(){
	float arucolength, tail_dist, head_dist;
    float m1, m2, angle, value;
	angle=0;
    arucolength = sqrt(pow(firebird.head.x - firebird.tail.x,2) + pow(firebird.head.y - firebird.tail.y,2));
    tail_dist = sqrt(pow(dest.x - firebird.tail.x,2) + pow(dest.y - firebird.tail.y,2));
    head_dist = sqrt(pow(dest.x - firebird.head.x,2) + pow(dest.y - firebird.head.y,2));

    m1 = (firebird.head.y - firebird.tail.y)/(firebird.head.x - firebird.tail.x);
    m2 = (dest.y - firebird.tail.y)/(dest.x - firebird.tail.x);
	
	value = (m2 - m1)/(1 + m1*m2 );
	if (value == 0)
	angle=0;
	else if (m1*m2 == -1){
		if (m2>m1)
		angle= -90;
		else if(m1>m2)
		angle= 90;
	}
	else if(abs(value)<1){
		angle=90-(atan(1/value)*(180/pi));
	}
	else
	angle=atan(value)*(180/pi);
	
if (angle < 0)
angle=angle+180;
	
return (int)angle;	
}*/


int compute(int angle){
    float P, I, D, output;
	I=0;
    P = 0.01*pid.kp * angle;
    I += 0.001*pid.ki * angle;
    D = 0.01*pid.kd * (angle - error_previous);

    output = P + I + D;

    if(output > 120){
        output = 120;
    }
    if(output < -120){
        output = -120;
    }

    error_previous = angle;

    return (int)output;
}

void resetNumber(){
    number[0] = 0;
    number[1] = 0;
    number[2] = 0;
    number[3] = 0;
}

void parsePacket(){

    int i = 0;
    int j = 0;

    while(str[i] != '\0'){

        if(str[i] == 'P'){
            i++;
            for(j=0; j<3; j++){
                while(str[i] != '|'){
                    number[j] = number[j]*10 + (str[i] - '0');
                    i++;
                }
                i++;
            }

            pid.kp = number[0];
            pid.ki = number[1];
            pid.kd = number[2];
            resetNumber();
        }
		
		if(str[i] == 'A'){
            i++;
            while(str[i] != '|'){
                number[0] = number[0]*10 + (str[i] - '0');
                i++;
            }
            i++;
            error_angle = number[0];
            
            resetNumber();
        }

         if(str[i] == 'S'){
            i++;
            while(str[i] != '|'){
                number[0] = number[0]*10 + (str[i] - '0');
                i++;
            }
            i++;
            stop_bit = number[0];
            resetNumber();
        }

		if(str[i] == 'F'){
            i++;
            while(str[i] != '|'){
                number[0] = number[0]*10 + (str[i] - '0');
                i++;
            }
            i++;
            forward_bit = number[0];
            resetNumber();
        }

        if(str[i] == 'B'){
            i++;
            for(j=0; j<2; j++){
                while(str[i] != '|'){
                    number[j] = number[j]*10 + (str[i] - '0');
                    i++;
                }
                i++;
            }

            servo_blocking = number[0];
			servo_angle = number[1];
            resetNumber();
        }

		if(str[i] == 'Z'){
            i++;
            while(str[i] != '|'){
                number[0] = number[0]*10 + (str[i] - '0');
                i++;
            }
            i++;
            align = number[0];
            resetNumber();
        }
		
        i++;
    }


}
void servo_set()
{ 
  _delay_ms(1000);
  servo_3(105);
  _delay_ms(1000);
  servo_1(0);
  _delay_ms(1000);
  servo_2(90);
  _delay_ms(1000);

  //servo_1_free();
  //servo_2_free();
  //servo_3_free();
}

void pick_block(int deg){
	unsigned char i = 0;
   	servo_set();
    _delay_ms(1000);
	if(deg == 0 )
	{   
		_delay_ms(500);
		servo_3(0);
		_delay_ms(1000);
		servo_3_free();
	 	for(i=0;i<70;i++)
		   { servo_1(i);
		     _delay_ms(30);
		   }
		_delay_ms(1000);
		  	servo_3(109);
		_delay_ms(1000);
		servo_3_free();
	}
	else if(deg == 90)
	{
   		_delay_ms(1000);
 		servo_3(0);
		_delay_ms(1000);
		servo_3_free();
    	for(i=0;i<70;i++)
		   {
		     servo_1(i);
		     _delay_ms(30);
		   }
		_delay_ms(1000);
		  	servo_3(109);
		_delay_ms(1000);
		servo_3_free();
		servo_2(180);
		_delay_ms(1000);
		servo_2_free();		
	}
	else if(deg == 270)
	{
		_delay_ms(500);
		servo_3(0);
		_delay_ms(1000);
		servo_3_free();
		for(i=0;i<70;i++)
		   { 
		   	 servo_1(i);
		     _delay_ms(30);
		   }
		_delay_ms(1000);
		servo_3(109);
		servo_2(0);
		_delay_ms(1000);
		servo_2_free();
	}
	else
	{   _delay_ms(500);
		servo_3(0);
		_delay_ms(1000);
		servo_3_free();
	 	for(i=0;i<70;i++)
		   { servo_1(i);
		     _delay_ms(30);
		   }
		_delay_ms(1000);
		servo_2(0);
		_delay_ms(1000);
		servo_2_free();
		for(i=70;i>20;i--)
		{
		  servo_1(i);
	      _delay_ms(30);
     	}
		_delay_ms(1000);
		servo_2(30);
		_delay_ms(1000);
		servo_2_free();
		servo_1(0);
		_delay_ms(1000);
		servo_2(90);
		_delay_ms(1000);
		servo_2_free();
		for(i=0;i<70;i++)
		   { servo_1(i);
		     _delay_ms(30);
		   }
		_delay_ms(1000);
		 servo_3(109);
		_delay_ms(1000);
		servo_3_free();
		servo_2(0);
		_delay_ms(1000);
		servo_2_free();

	}
    servo_blocking = 0;

}

void drop_block(int angle){
	unsigned char i = 0;
    _delay_ms(500);
	servo_2_free();
    servo_3_free();
	servo_3(10);
	_delay_ms(2000);
	servo_3_free();
	for(i=70;i>20;i--)
		{
		  servo_1(i);
	      _delay_ms(30);
     	}
	_delay_ms(2000);
	servo_2(90);
	_delay_ms(2000);
	servo_2_free();
	servo_set();
    servo_blocking = 0;
	servo_1_free();
    servo_2_free();
    servo_3_free();
	back_mm(70);
}

ISR(USART0_RX_vect){
	data = UDR0;
	if(data=='>'){
		flag = 0;
		str[count]='\0';
		count = 0;
		parsePacket();
	}
	else if(flag){
		str[count++] = data;
	}

	if(data == '<'){
		flag = 1;		
	}
}



int main()
{
    int motorspeed;
	init_devices();
	lcd_set_4bit();
	servo_set();
	lcd_init();
	motorspeed = 0;
    velocity(127, 127);
    forward();
	
	while(1){
		if(!flag){

            if(servo_blocking == 1){
				stop();
                pick_block(servo_angle);
				servo_blocking = 0;
                continue;
            }

            if(servo_blocking == 2){
				stop();
                drop_block(servo_angle);
				servo_blocking = 0;
                continue;
            }

			if(align == 1){
				velocity(140,140);
				left_degrees(10); 
				align = 0;
				continue;
			}
			else if(align == 2){
				velocity(140,140);
				right_degrees(10);
				align = 0;
				continue;
			}
			
			if(forward_bit == 1){
				velocity(130,130);
				forward_mm(50);
				forward_bit = 0;
				continue;
				}
			else if(forward_bit == 2){
				velocity(130,130);
				back_mm(50);
				forward_bit = 0;
				continue;
				}



            if(stop_bit == 1){
                stop();
            }
            else{
			
				if(error_angle-180>35){
					velocity(140,140);
					left();
					//_delay_ms(300);
					//stop();
				}
				else if(error_angle-180<-35){
					velocity(140,140);
					right();
				//	_delay_ms(300);
				//	stop();
				}
				else{
					forward();	
        			motorspeed = compute(error_angle-180);
					//velocity(110 - motorspeed, 110 + motorspeed);
        			velocity(127 - motorspeed, 124 + motorspeed);
				}
			}
            


            lcd_print(1,1, 127-motorspeed, 3);
            lcd_print(1,5, 124+motorspeed, 3);
            lcd_print(1,9,abs(error_angle-180), 3);
			lcd_print(2,1, pid.kp, 3);
			lcd_print(2,8, stop_bit, 3);
            /*lcd_print(1,13, firebird.tail.y, 3);
            
            
			lcd_print(2,10,dest.x, 3);
			lcd_print(2,13,dest.y, 3);*/
		}
	}
}
