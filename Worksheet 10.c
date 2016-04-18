
//LCD Model: Devantech LCD02
//LCD Documentation: http://www.robot-electronics.co.uk/htm/Lcd02tech.htm
/* Getting the Keypad and LCD to work, using PCA0. SMB is assumed to be on P0.2 & P0.3 (XBR0 = 0x05)
Okay to change XBR0 setting to 0x07 to match wiring */

#include <stdio.h>
#include <stdlib.h>
#include <c8051_SDCC.h> // Include files. This file is available online in LMS
#include <i2c.h>        // Get from LMS, THIS MUST BE INCLUDED AFTER stdio.h
#define PCA_START 28672 // 28672 for exactly 20ms
//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);   // Initialize ports for input and output
void Interrupt_Init(void);
void PCA_Init(void);
void SMB0_Init(void);
void PCA_ISR(void) __interrupt 9;
//void wait(void);
void pause(void);
char twoDigitNum(void);
void displayInput(char); 

// Global variables
unsigned int Counts, nCounts, nOverflows;

//MAIN FUNCTION **************************************************************
void main(void)
{
    char keypad;
    char tensDigit; 
    char onesDigit; 
    char numAsDecimal; 
    
    initMethods(); 
    putchar('\r');  // Dummy write to serial port
    printf("\nStart\r\n");

    Counts = 0;
    while (Counts < 1); // Wait a long time (1s) for keypad & LCD to initialize

    lcd_clear();
    lcd_print("Calibration:\nHello world!\n012_345_678:\nabc def ghij");
    
    
    while (1)
    {
        keypadInput = read_keypad(); 
        if (keypadInput != -1) { //if keypad pressed
            tensDigit = keypadInput; while(read_keypad() != -1); pause();
            
            do {
                onesDigit = read_keypad();
            } while (read_keypad() == -1)
            while(read_keypad() != -1); pause(); 
             
            numAsDecimal = twoDigitNum(tensDigit, onesDigit); 
            displayInput(numAsDecimal); 
        }
    }
}



//KEYPAD READING**************************************************************
char twoDigitNum(char tensDigit, char onesDigit) {
    return tensDigit*10 + onesDigit; 
}

void displayInput(char) {
    lcd_clear(); //function included in LCD library
    lcd_print("Your key was:\n %c = Hex %X", numAsDecimal, numAsDecimal);
    printf("\n\rYour key was: %c = Hex %X", numAsDecimal, numAsDecimal);
    if(keypad == 0) {
        printf("   **Wire Connection/XBR0 Error**   ");
    }   // A returned value of 0 (0x00) indicates wiring error*/
}

//TIMING FUNCTIONS*************************************************************

void pause(void)
{
    nCounts = 0;
    while (nCounts < 1);// 1 count -> (65536-PCA_START) x 12/22118400 = 20ms
}                       // 6 counts avoids most of the repeated hits

void wait(void)
{
    nCounts = 0;
    while (nCounts < 50);    // 50 counts -> 50 x 20ms = 1000ms
}


//INITIALIZATION FUNCTIONS*****************************************************

void initMethods(void) {
    Sys_Init();     // System Initialization - MUST BE 1st EXECUTABLE STATEMENT
    Port_Init();    // Initialize ports 2 and 3 - XBR0 set to 0x05, UART0 & SMB
    Interrupt_Init();   // You may want to change XBR0 to match your SMB wiring
    PCA_Init();
    SMB0_Init();
}

void Port_Init(void)	//0x05
{
    XBR0 = 0x05;    // NOTE: Only UART0 & SMB enabled; SMB on P0.2 & P0.3
}                   // No CEXn are used; no ports to initialize

void Interrupt_Init(void)
{
    IE |= 0x02;
    EIE1 |= 0x08;
    EA = 1;
}

void PCA_Init(void)
{
    PCA0MD = 0x81;      // SYSCLK/12, enable CF interrupts, suspend when idle
//  PCA0CPMn = 0xC2;    // 16 bit, enable compare, enable PWM; NOT USED HERE
    PCA0CN |= 0x40;     // enable PCA
}

void SMB0_Init(void)    // This was at the top, moved it here to call wait()
{
    SMB0CR = 0x93;      // Set SCL to 100KHz
    ENSMB = 1;          // Enable SMBUS0
}

void PCA_ISR(void) __interrupt 9
{
    if (CF)
    {
        CF = 0;                     // clear the interrupt flag
        nOverflows++;               // continuous overflow counter
        nCounts++;
        PCA0L = PCA_START & 0xFF;   // low byte of start count
        PCA0H = PCA_START >> 8;     // high byte of start count
        if (nCounts > 50)
        {
            nCounts = 0;
            Counts++;               // seconds counter
        }
     }
     else PCA0CN &= 0xC0;           // clear all other 9-type interrupts
}
