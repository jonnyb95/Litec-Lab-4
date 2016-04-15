/* 
Welcome to our stellar Lab 4 Code! 
Section 4, Side A: 
-Daniel LeBourdais
-Jonathan Blumers
-Greg Lerner
*/


// LIBRARY IMPORTS
#include <c8051_SDCC.h> // include C8051 library
#include <stdio.h> // include C standard input output library
#include <stdlib.h> // include C standard library
#include <i2c.h>



// FUNCTION PROTOTYPES
void Port_Init(void); // Will initialize output ports for the motor and servo
void PCA_Init(void); // Will initialize PCA0 
void XBR0_Init(void); 
unsigned int Read_Ranger(void);
void PCA_ISR ( void ) __interrupt 9;
void i2c_start(void);
void SMB_Init(void);
unsigned int Read_Compass(void);
void Drive_Car(void);
void MotorSpeed(void);
void CorrectSteering(void);

void centered(void);
void max_left(void);
void max_right(void);
unsigned int userInputPWForWheelPosition(void); 

unsigned int Inputgain(void);
void Inputheading(void);


// GLOBAL VARIABLES
unsigned int Counts = 0; // overflow counts of PCA0 counter/timer
unsigned char Data[2];


unsigned int r_count = 0;
unsigned int range;


unsigned int h_count = 0; 
unsigned int heading;
__xdata unsigned int desired_heading = 0;
__xdata unsigned int k = 1;
signed int error;


__sbit __at 0xB6 motorSS;
__sbit __at 0xB7 servoSS;


__xdata unsigned int CEX0_CENTER = 2765; // approximate value associated with central servo position
__xdata unsigned int CEX0_MIN = 1659; // lower bound associated with rightmost servo position
__xdata unsigned int CEX0_MAX = 3871; // upper bound associated with leftmost servo position
unsigned int CEX0_PW = 2765; // pulse width being sent to CEX0, servo

__xdata unsigned int CEX2_MIN = 2027; // lower bound associated with minimum motor speed
__xdata unsigned int CEX2_MAX = 3502; // upper bound associated with maximum motor speed
__xdata unsigned int CEX2_NEUTRAL = 2765;
unsigned int CEX2_PW = 2765; // pulse width being sent to CEX2, motor 

char input; // user input characters for device calibration
unsigned int calibrating;

char headingType;
int getHeading; 
char inputHeading;
char headingArray[4];
char i;
unsigned int batteryVoltage;

//------------------------------------------------------

// MAIN

void main(void)
{   

//Call initialization routines
    Sys_Init();
    putchar(' ');
    Port_Init();
    XBR0_Init();
	PCA_Init();
	SMB_Init();
	Counts = 0; 
	while(Counts < 50); //Wait a second for the motor to warm up
	Counts = 0;
    while (h_count < 5); // Wait a long time (1s) for keypad & LCD to initialize
	lcd_clear();
	lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");

	    //Calibrate the servo
    printf("Servo Steering Calibration\n\n\r");
    CEX0_PW = CEX0_CENTER; // set steering to near center
	PCA0CP2 = 0xFFFF - CEX2_PW;
	PCA0CP0 = 0xFFFF - CEX0_PW;
	centered();     // calibration function
	max_left();     // calibration function
	max_right();    // calibration function 
	printf("--------------------- CALIBRATION COMPLETE ----------------------\n\n\n\r");
	
	Inputheading();
	Inputgain();
    
	while(1)
	{
		Drive_Car();
	}
}

//------------------------------------------------------

void Inputheading(void)
{
	lcd_clear();
	printf("Press '1' for input\r\nPress '2' for list\r\n");
	lcd_print("Press '1' for input\nPress '2' for list\n");
	headingType = read_keypad();
	while(h_count<1);
	while(headingType != -1);
	if(headingType == 1)
	{
		lcd_clear();
		printf("Input 4 character desired heading\r\n");				
		lcd_print("Input 4 character desired heading\r\n");
		for(i = 0; i<=3; i++)
		{
			while(h_count<1);
			headingArray[i] = read_keypad();
			
		}
		desired_heading = ((headingArray[0]*1000) + (headingArray[1]*100) + (headingArray[2]*10) + headingArray[3]);
	}
	if(headingType == 2)
	{
		lcd_clear();
		printf("Select from the following options: Option 1: 0 Degrees, Option 2: 90 Degrees, Option 3: 180 Degrees, Option 4: 270 Degrees\r\n");
		lcd_print("Select from the following options: Option 1: 0 Degrees, Option 2: 90 Degrees, Option 3: 180 Degrees, Option 4: 270 Degrees\r\n");
		inputHeading = read_keypad();
		while(h_count<1);
		while(inputHeading != -1);
		if(inputHeading == 1)
		{
			desired_heading = 0;
		}
		if(inputHeading == 2)
		{
			desired_heading = 900;
		}
		if(inputHeading == 3)
		{
			desired_heading = 1800;
		}
		if(inputHeading == 4)
		{
			desired_heading = 2700;
		}
	}
}

unsigned int Inputgain(void)
{
	printf("Input desired gain\r\n");
	while(h_count<1);
	while(headingType != -1);	
	k = read_keypad();
	return k;
}

void UpdateLCD(void)
{
	Inputheading();
	if(h_count >= 20)
	{
		printf("Heading: %d, Range: %d, Battery voltage: %d \r\n", heading, range, batteryVoltage);
		h_count = 0;
	}
}

void CorrectSteering(void)
{
	error = desired_heading - heading;
	printf("Desired Heading: %d\tActual Heading: %d\tError: %d\tCorrection Value: %d\n\r", desired_heading, heading, error, k*error);
	if(error>1800){error-=3600;}
	else if(error<(-1800)){error+=3600;}
	
	CEX0_PW = (int) (CEX0_CENTER + (k * error));
	
	PCA0CP0 = 0xFFFF - CEX0_PW; //Configure CEX0 to have a pulse width of PW
	//printf("CEX0 PW: %d\r\n", CEX0_PW);
}

void MotorSpeed(void)
{
	
	if(range <= 10)
	{
		CEX2_PW = CEX2_MAX; //Max forward
	}
	if(range >10 && range <40)
	{
		CEX2_PW = (int) (3502 - (24.567*(range-10))); //linearly varies PW between max forward and neutral
	}
	if(range <=50 && range >=40)
	{
		CEX2_PW = CEX2_NEUTRAL; //Neutral
	} 
	if(range >50 && range <90)
	{
		CEX2_PW = (int) (2765 - (18.45*(range-50))); //linearly varies PW between neutral and max reverse
	}
	if(range >=90)
	{
		CEX2_PW = CEX2_MIN; //Max reverse
	}
	PCA0CP2 = 0xFFFF - CEX2_PW; //Configure CEX2 to have a pulse width of PW
}

void Drive_Car(void) 
{
	if (h_count >= 2) 
	{
		Read_Compass(); 
		h_count = 0;
		if(servoSS == 1)
		{
			CorrectSteering();
		}
		else
		{
			PCA0CP0 = 0xFFFF - CEX0_CENTER;
		}
	}

	if (r_count >= 4) 
	{
		Read_Ranger(); 
		r_count = 0;
		if(motorSS == 1)
		{
			MotorSpeed();
		}
		else
		{
			PCA0CP2 = 0xFFFF - CEX2_NEUTRAL;
		}
//	UpdateLCD();
	}
}

unsigned int Read_Ranger(void)
{
	unsigned char Data[2];
	unsigned char addr = 0xE0;
	i2c_read_data(addr, 2, Data, 2); //read two bytes starting at register 2
	range = (((unsigned int)Data[0] << 8) | Data[1]);
	Data[0] = 0x51;
	i2c_write_data(0xE0, 0, Data, 1); //send out new ping
	return range;
}

unsigned int Read_Compass(void)
{
	unsigned char Data[2];
    unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2
    heading = (((unsigned int)Data[0] << 8) | Data[1]); //combine the two values
    return heading/10; // the heading returned in degrees between 0 and 3599
}

// SERVO CALIBRATION

void centered()
{
	printf("CENTER: Adjust the pulswidth to make the servo more left (a) or right (d), until the steering is centered, when centered press 'g'\n\r");
	CEX0_CENTER = userInputPWForWheelPosition();
}

void max_left()
{
	printf("LEFT: Adjust the steering until the servo is at a maximum left (a), when left press 'g'\n\r");
    CEX0_MIN = userInputPWForWheelPosition(); 
}

void max_right()
{
	printf("RIGHT: Adjust the steering until the servo is at a maximum right (d), when right press 'g'\n\r");
	CEX0_MAX = userInputPWForWheelPosition();	
}

unsigned int userInputPWForWheelPosition(void) {

	printf("CEX0 PW before calibration: %d\r\n", CEX0_PW); 

    calibrating = 1;
	while(calibrating) {
		input = getchar(); printf("\n\r");
		if (input == 'd') { 
			CEX0_PW = (CEX0_PW + 10 > CEX0_MAX) ? CEX0_MAX : CEX0_PW + 10; 
			
			PCA0CP0 = 0xFFFF - CEX0_PW; 
		} 
		else if (input == 'a') { 
			CEX0_PW = (CEX0_PW - 10 < CEX0_MIN) ? CEX0_MIN : CEX0_PW - 10; 
			
			PCA0CP0 = 0xFFFF - CEX0_PW; 
		} 
		else if (input == 'g') { calibrating = 0; }
	}

	printf("CEX0 PW after calibration: %d\r\n\n\n", CEX0_PW); 

	return CEX0_PW;
}



//  ISRs AND INITILIZATIONS

void PCA_ISR ( void ) __interrupt 9
{

	PCA0 = 0xFFFF-36864; /* MOST IMPORTANT LINE: Set start count so that the timer overflows in 20ms */
	Counts ++; // Increment the global overflow counter, Counts
	CF = 0; // Clear the PCA0 counter/timer overflow flag -- **isn't this redundant with the next line?**
	PCA0CN &= 0x40; // For PCA0, clear the overflow flag and clear each module's flag
	r_count++;
	h_count++;
}

void SMB_Init(void)
{
	SMB0CR = 0x93;
	ENSMB = 1;
}

// Set up ports for input and output
void Port_Init() 
{ 
	P1MDOUT |= 0x05; 
	P3MDOUT &= ~0xC0;
	P3 |= 0xC0;
}  // set output pins for CEX0/2 (Port 1.0/2) to push-pull mode 

// Set up the crossbar
void XBR0_Init() 
{ 
	XBR0 = 0x27; 
} // Configure crossbar

void PCA_Init(void)
{
	PCA0MD = 0x81; /* Enable PCA0 Counter/Timer Overflow interrupt when CF is set
	Also suspend PCA0 operation while the system controller is in idle mode */
	PCA0CPM0 = 0xC2; // For PCA0 module 0, enable 16-bit PWM mode and compare
	PCA0CPM2 = 0xC2;
	PCA0CN = 0x40; // Enable the PCA0 Counter/Timer itself
	EIE1 |= 0x08; // Enable interrupt requests generated by CP1RIF flag
	EA = 1; // Enable all interrupts
}