#define F_CPU 14745600
#define pi 3.1415
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/lcd.h>
#include <math.h>
#include <stdlib.h>

int error_previous = 0;
int error_angle=0;
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

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize ports
void init_ports()
{
 motion_pin_config();
 lcd_port_config();
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

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
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
		
		if(str[i] == 'T'){
            i++;
            for(j=0; j<2; j++){
                while(str[i] != '|'){
                    number[j] = number[j]*10 + (str[i] - '0');
                    i++;
                }
                i++;
            }
            dest.x = number[0];
            dest.y = number[1];
            resetNumber();
        }

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
        
        if(str[i] == 'R'){
            i++;
            for(j=0; j<4; j++){
                while(str[i] != '|'){
                    number[j] = number[j]*10 + (str[i] - '0');
                    i++;
                }
                i++;
            }

            firebird.head.x = number[0];
            firebird.head.y = number[1];
            firebird.tail.x = number[2];
            firebird.tail.y = number[3];
            resetNumber();
        }
		
		if(str[i] == 'A'){
            i++;
            for(j=0; j<1; j++){
                while(str[i] != '|'){
                    number[j] = number[j]*10 + (str[i] - '0');
                    i++;
                }
                i++;
            }

            error_angle = number[0];
            resetNumber();
        }
		
        i++;
    }
}

ISR(USART0_RX_vect){
	data = UDR0;
	if(data=='>'){
		flag = 0;
		str[count]='\0';
		count = 0;
	}
	else if(flag){
		str[count++] = data;
	}

	if(data == '<'){
		flag = 1;
		
	}
}

void init_devices (void)
{
 cli();
 init_ports();
 timer5_init();
 uart0_init();
 sei();
}

int main()
{
    int motorspeed,head_dist;
	init_devices();
	lcd_set_4bit();
	lcd_init();
    velocity(127, 127);
    forward();
	
	while(1){
		if(!flag){
            parsePacket();
            head_dist = sqrt(pow(dest.x - firebird.head.x,2) + pow(dest.y - firebird.head.y,2));
			if(head_dist <10){
				velocity(0,0);
			}
			else{
            motorspeed = compute(error_angle-180);
            velocity(127 - motorspeed, 124 + motorspeed);
			}			
            lcd_print(1,1, 127-motorspeed, 3);
            lcd_print(1,5, 124+motorspeed, 3);
            lcd_print(1,9,abs(error_angle-180), 3);
			
            /*lcd_print(1,13, firebird.tail.y, 3);
            lcd_print(2,1, pid.kp, 3);
            lcd_print(2,4, pid.kd, 3);
            lcd_print(2,7, pid.ki, 3);
			lcd_print(2,10,dest.x, 3);
			lcd_print(2,13,dest.y, 3);*/
		}
	}
}
