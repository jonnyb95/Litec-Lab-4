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
void ADC_Init(void); /* Set the reference voltage for the A/D converter to 2.4 V internal, enable ADC1 and give it a gain of 1 */
void XBR0_Init(void); 
void SMB_Init(void);
void PCA_ISR ( void ) __interrupt 9;
void i2c_start(void);
void GetBatteryVoltage(void); /* Set ADC1 to battery input, wait, read voltage, return to pot input, print voltage in mV */
unsigned char ReadADC(void); /* Specify an analog input pin and perform an A/D conversion at that pin */
unsigned char ReadPot(void); /* Ensure ADC1 on pot input, read voltage, return ADC1 value */
unsigned int ReadRanger(void);
unsigned int ReadCompass(void);
void DriveCar(void);
void MotorSpeed(unsigned char digitalValue);
void CorrectSteering(void);
void AvoidObsticles(void);
void Calibrate(void);
void Centered(void);
void MaxLeft(void);
void MaxRight(void);
unsigned int UserSetPW(void); 
unsigned int InputGain(void);
signed int InputHeading(void);
void PrintData(void);
void Pause20ms(void);


// GLOBAL VARIABLES
__sbit __at 0xB6 motorSS;
__sbit __at 0xB7 servoSS;

char i;  // incrementer
unsigned int Counts = 0; // 20ms count
unsigned int lcd_count = 0;
unsigned char Data[2];
unsigned int r_count = 0;
unsigned int range;
unsigned int h_count = 0; 
unsigned int heading;
__xdata unsigned int desiredHeading = 0;
__xdata unsigned int k = 1;
signed int error;
unsigned char turnDistance = 20; //in cm, given in lab description
unsigned char stopDistance = 10; //in cm, given in lab description
unsigned char hasTurned = 0; 
unsigned int batteryVoltage;
__xdata unsigned int CEX0_CENTER = 2765; // approximate value associated with central servo position
__xdata unsigned int CEX0_MIN = 1659; // lower bound associated with rightmost servo position
__xdata unsigned int CEX0_MAX = 3871; // upper bound associated with leftmost servo position
unsigned int CEX0_PW = 2765; // pulse width being sent to CEX0, servo
__xdata unsigned int CEX2_MIN = 2027; // lower bound associated with minimum motor speed
__xdata unsigned int CEX2_MAX = 3502; // upper bound associated with maximum motor speed
__xdata unsigned int CEX2_NEUTRAL = 2765;
unsigned int CEX2_PW = 2765; // pulse width being sent to CEX2, motor 
char input; // user input characters for device calibration
unsigned int calibrating;  // calibrating flag
char headingType;
int headingInput;

// MAIN FUNCTION ------------------------------------------------------

void main(void)
{   
//Call initialization routines
    Sys_Init();
    putchar(' ');
    Port_Init();
    XBR0_Init();
	PCA_Init();
	ADC_Init();
	SMB_Init();
	for(i=0; i<50; i++) {Pause20ms();} //Wait 1s for the motor & LCD/keypad
	InputHeading();
	InputGain();
	printf("\n\rdesiredHeading,heading,error,range,CEX0_PW,CEX2_PW,batteryVoltage");
	while(1)
	{
		DriveCar();
		AvoidObsticles();
		PrintData();
	}
}

// FUNCTION DEFINITIONS ------------------------------------------------------

// GENERAL
void Pause20ms(void) {
	Counts = 0;
    while (Counts < 1);
}

// DATA FUNCTIONS
unsigned int ReadRanger(void) {
	unsigned char Data[2];
	unsigned char addr = 0xE0;
	i2c_read_data(addr, 2, Data, 2); //read two bytes starting at register 2
	range = (((unsigned int)Data[0] << 8) | Data[1]);
	Data[0] = 0x51;
	i2c_write_data(0xE0, 0, Data, 1); //send out new ping
	return range;
}
unsigned int ReadCompass(void) {
	unsigned char Data[2];
    unsigned char addr = 0xC0; // the address of the sensor, 0xC0 for the compass
	i2c_read_data(addr, 2, Data, 2); // read two byte, starting at reg 2
    heading = (((unsigned int)Data[0] << 8) | Data[1]); //combine the two values
    return heading/10; // the heading returned in degrees between 0 and 3599
}
void GetBatteryVoltage(void) {
	AMX1SL = 4; // Change ADC1 input to P1.4 (battery voltage divider)
	//delay ~5ms
	batteryVoltage = ((ReadADC() / 256) * 15000);  // get A/D conversion and convert to mV
	AMX1SL = 5; // Change ADC1 input back to P1.5 (potentiometer)
	printf("Battery Voltage: %dmV", batteryVoltage);
	printf("Battery Voltage: %d.%d%d%dV", batteryVoltage/1000,(batteryVoltage%1000)/100,((batteryVoltage%1000)%100)/10,((batteryVoltage%1000)%100)%10);
}
unsigned char ReadPot(void) {
	AMX1SL = 5; // Ensure ADC1 input is on P1.5 (potentiometer)
	return ReadADC();	
}
unsigned char ReadADC(void) {
    ADC1CN = ADC1CN & ~0x20; // Clear the Conversion Completed flag
    ADC1CN = ADC1CN | 0x10; // Initiate A/D conversion
    while ((ADC1CN & 0x20) == 0x00); // Wait for conversion to complete
    return ADC1; // Return digital value in ADC1 register
}

// LCD KEYPAD FUNCTIONALITY
signed int InputHeading(void) {
	lcd_clear();
	Pause20ms();
	printf("Press '1' on keypad to input heading\r\nPress '2' on keypad for list of possible headings\r\n");
	lcd_print("Press '1' for input\nPress '2' for list\n");
	Pause20ms();
	headingType = kpd_input(1);
	Pause20ms();
	if(headingType != -1) // wait until key not pressed
	{
		printf("success!");
		if(headingType == 1)
		{
			lcd_clear();
			Pause20ms();
			printf("Input 4 character desired heading on keypad and press '#' key when finished\r\n");				
			lcd_print("Input 4 character\ndesired heading\r\n");
			desiredHeading = kpd_input(1);
		}
		if(headingType == 2)
		{
			lcd_clear();
			printf("Select from the following options: Option 1: 0 Degrees Option 2: 90 Degrees Option 3: 180 Degrees Option 4: 270 Degrees\r\n");
			lcd_print("Option 1: 0 Degrees Option 2: 90 Degrees Option 3: 180 Degrees Option 4: 270 Degrees\r\n");
			headingInput = kpd_input(1);
			Pause20ms();
			if(headingInput != -1)
			{	// wait until key is not pressed
				if(headingInput == 1){desiredHeading = 0;}
				if(headingInput == 2){desiredHeading = 900;}
				if(headingInput == 3){desiredHeading = 1800;}
				if(headingInput == 4){desiredHeading = 2700;}
			}
		}
	}
	return desiredHeading;
}
unsigned int InputGain(void) {
	printf("Input desired gain\r\n");
	lcd_clear();
	Pause20ms();
	lcd_print("Input desired gain\r\n");
	Pause20ms();
	if(headingType != -1){k = kpd_input(1);}
	lcd_clear();
	return k;
}
void PrintData(void) {
	printf("\n\r%d,%d,%d,%d,%d,%d,%d", desiredHeading,heading,error,range,CEX0_PW,CEX2_PW,batteryVoltage);
	if(lcd_count >= 20)
	{
		lcd_count = 0;
		lcd_print("\rHeading: %d\n\rRange: %d\n\rBattery voltage: %d\r\n", heading, range, batteryVoltage);	
	}
}

// POSITION CONTROLL
void AvoidObsticles(void) {
	if (r_count >= 4) 
	{
		ReadRanger(); 
		r_count = 0;
	}
	if (!hasTurned && range <= turnDistance && range > stopDistance)
	{
		desiredHeading += 900; /* Raise the desired heading by 90 degrees */ 
		hasTurned = 1; 
    }
}
void CorrectSteering(void) {
	error = desiredHeading - heading;
	printf("Desired Heading: %d\tActual Heading: %d\tError: %d\tCorrection Value: %d\n\r", desiredHeading, heading, error, k*error);
	if(error>1800){error-=3600;}
	else if(error<(-1800)){error+=3600;}
	CEX0_PW = (int)(CEX0_CENTER + (k * error));
	if(CEX0_PW > CEX0_MAX) {CEX0_PW = CEX0_MAX;}
	if(CEX0_PW < CEX0_MIN) {CEX0_PW = CEX0_MIN;}
	PCA0CP0 = 0xFFFF - CEX0_PW; //Configure CEX0 to have a pulse width of PW
}
void MotorSpeed(unsigned char digitalValue) {
    CEX2_PW = CEX2_MIN + (int)(digitalValue * (CEX2_MAX - CEX2_MIN)/(255 - 0));
    // Since the digital value is an unsigned char, it ranges from 0 to 255
	if (range < stopDistance) {CEX2_PW = CEX2_NEUTRAL;}
	// If latest range is within the stop distance, put the car in neutral
    PCA0CP2 = 0xFFFF - CEX2_PW; //Set the motor pulse width
}
void DriveCar(void) {
	if (h_count >= 2) 
	{
		ReadCompass(); 
		h_count = 0;
		if(motorSS == 1){MotorSpeed(ReadPot());}
		else{PCA0CP2 = 0xFFFF - CEX2_NEUTRAL;}
		if(servoSS == 1){CorrectSteering();}
		else{PCA0CP0 = 0xFFFF - CEX0_CENTER;}
	}
}

// SERVO CALIBRATION
void Calibrate(void) {
	printf("---------------------- SERVO CALIBRATION ----------------------\n\n\r");
	Centered();     // calibration function
	MaxLeft();     // calibration function
	MaxRight();    // calibration function 
	printf("--------------------- CALIBRATION COMPLETE ---------------------\n\n\n\r");
}
void Centered(void) {
	CEX0_PW = CEX0_CENTER; // set steering to near center
	PCA0CP2 = 0xFFFF - CEX2_PW;
	PCA0CP0 = 0xFFFF - CEX0_PW;
	printf("CENTER: Adjust the pulswidth to make the servo more left (a) or right (d), until the steering is centered, when centered press 'g'\n\r");
	CEX0_CENTER = UserSetPW();
}
void MaxLeft(void) {
	printf("LEFT: Adjust the steering until the servo is at a maximum left (a), when left press 'g'\n\r");
    CEX0_MIN = UserSetPW(); 
}
void MaxRight(void) {
	printf("RIGHT: Adjust the steering until the servo is at a maximum right (d), when right press 'g'\n\r");
	CEX0_MAX = UserSetPW();	
}
unsigned int UserSetPW(void) {
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
void PCA_ISR ( void ) __interrupt 9 {

	PCA0 = 0xFFFF-36864; /* MOST IMPORTANT LINE: Set start count so that the timer overflows in 20ms */
	Counts ++; // Increment the global overflow counter, Counts
	CF = 0; // Clear the PCA0 counter/timer overflow flag -- **isn't this redundant with the next line?**
	PCA0CN &= 0x40; // For PCA0, clear the overflow flag and clear each module's flag
	r_count++;
	h_count++;
	lcd_count++;
}
void PCA_Init(void) {
	PCA0MD = 0x81; /* Enable PCA0 Counter/Timer Overflow interrupt when CF is set
	Also suspend PCA0 operation while the system controller is in idle mode */
	PCA0CPM0 = 0xC2; // For PCA0 module 0, enable 16-bit PWM mode and compare
	PCA0CPM2 = 0xC2;
	PCA0CN = 0x40; // Enable the PCA0 Counter/Timer itself
	EIE1 |= 0x08; // Enable interrupt requests generated by CP1RIF flag
	EA = 1; // Enable all interrupts
}
void ADC_Init(void) {
    REF0CN |= 0x03; // Set Vref to use internal reference voltage (2.4 V)
	REF0CN &= ~0x08; // Set Vref to use internal reference voltage (2.4 V)
    ADC1CN = 0x80; // Enable A/D converter (ADC1)
    ADC1CF |= 0x01; // Set A/D converter gain to 1
	ADC1CF &= ~0x02; // Set A/D converter gain to 1
}
void SMB_Init(void) {
	SMB0CR = 0x93;
	ENSMB = 1;
}
void Port_Init(void) { 
	P1MDOUT |= 0x05; // set output pins for CEX0/2 (Port 1.0/2) to push-pull mode
	P3MDOUT &= ~0xC0;
	P3 |= 0xC0;
}
void XBR0_Init(void) { 
	XBR0 = 0x27; // Configure crossbar
}