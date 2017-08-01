/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Fall 2016
***********************************************************************
      
 Team ID: < 7 >

 Project Name: < Self-Parking Car >

 Team Members:

   - Team/Doc Leader: < Apoorv Sharma >      Signature: ______AS________________
   
   - Software Leader: < Tanuj Rungta >      Signature: ______TR________________

   - Interface Leader: < Kunwar Digraj Singh Jain >     Signature: ______KDSJ________________

   - Peripheral Leader: < Ruchir Aggarwal >    Signature: __________RA____________                        


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

***********************************************************************

 The objective of this Mini-Project is to .... < Design a car that has parallel parking capabilities >


***********************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1. Car can parallel park within a given box

 2. Car can decide not to park if the parking distance is too small

 3. Car can park if there is no car ahead of the car it chooses to park after

 4. 

 5.

***********************************************************************

  Date code started: < 2nd December 2016 >

  Update history (add an entry every time a significant change is made):

  Date: < 5th December >  Name: < Tanuj Rungta >   Update: < Improvement in detecting edge >

  Date: < 6th December 2016 >  Name: < Tanuj Rungta >   Update: < Improvement in parking by checking allignment detection >

  Date: < 7th December 2016 >  Name: < Tanuj Rungta >   Update: < Changed code structure from sequential based approach to a state machine based
                                                                  approach>


***********************************************************************
*/


#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void rdisp(void); // RPM display
void bco(char x); // SCI buffered character output
void shiftout(char); // LCD drivers (written previously)
void lcdwait(void);
void send_byte(char);
void send_i(char);
void chgline(char);
void print_c(char);
void pmsglcd(char[]);
void getReadings();
void drive(int left, int right, int time);

void parkCar();
void detectParking();
void fbStateMachine();
void moveCar();
void detectEdge();

/* Variable declarations */           
char leftpb = 0; // left pushbutton flag
char rghtpb = 0; // right pushbutton flag
char prevpb = 0; // previous pushbutton state
char tin = 0; // SCI transmit display buffer IN pointer
char tout = 0; // SCI transmit display buffer OUT pointer
int prevpbl = 0;      //to store the previous condition of previous left push button
int prevpbr = 0;      //to store the previous condition of previous right push button
int tencnt = 0;
int i = 0;
int diff = 0; 
int changeDistance = 0;
int timeTaken = 0;
int fbState = 0; // 0:stop ; 1:forward ; 2:backward
int isForward;
int isBackward = 0;
int isLeft = 0;
int isRight = 0;
int carState = 0;
int detectParkingState = 0;
int parkCarState = 0;
int count = 0;

unsigned long int front_sense = 0 ;
unsigned int final_front_sense = 0 ;
unsigned int prev = 0 ;

unsigned long int back_sense = 0 ;
unsigned int final_back_sense = 0 ;

unsigned long int right_one_sense = 0 ;
unsigned int final_right_one_sense = 0 ;

unsigned long int right_two_sense = 0 ;
unsigned int final_right_two_sense = 0 ;

#define TSIZE 81 // transmit buffer size (80 characters)
char tbuf[TSIZE]; // SCI transmit display buffer

              
/* Special ASCII characters */
#define CR 0x0D // ASCII return 
#define LF 0x0A // ASCII new line 

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10 // RS pin mask (PTT[4])
#define RW 0x20 // R/W pin mask (PTT[5])
#define LCDCLK 0x40 // LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F // LCD initialization command
#define LCDCLR 0x01 // LCD clear display command
#define TWOLINE 0x38 // LCD 2-line enable command
#define CURMOV 0xFE // LCD cursor move instruction
#define LINE1 = 0x80 // LCD line 1 cursor position
#define LINE2 = 0xC0 // LCD line 2 cursor position

  
/*   
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */
  
  ATDDIEN = 0xC0; //For PAD7 and PAD6
  DDRAD = 0; //program port AD for input mode
  DDRM = 0xFF;
  
  //ATD Initilizations 
  DDRT = 0x7F; 
  ATDCTL2 = 0x80; 
  ATDCTL3 = 0x30; //conversion length is 6 sequence 
  ATDCTL4 = 0xE5; // 8 bit 
  
  //PWM Initilizations
  PWME = 0x0F; //enable ch 3
  PWMPOL = 0x0F; //positive polarity
  PWMCAE = 0; //left align
  MODRR = 0x0F; //PWM Ch3 is routed to PTT3
  
  PWMCLK_PCLK3 = 1; //use SB for ch 3
  PWMCLK_PCLK2 = 1;
  PWMCLK_PCLK1 = 1; //use SA for ch 3
  PWMCLK_PCLK0 = 1;
  
  PWMSCLB = 0x1D; //post scalar is 29
  PWMSCLA = 0x1D; //post scalar is 29
  PWMPRCLK = 0x44; //prescale to 16
  PWMCTL = 0; //operate in 8 bit mode
  
  PWMPER3 = 0xFF; //0 to 100 range for duty cycle
  PWMDTY3 = 0;   //motor is initlally stoppe  
  
  PWMPER2 = 0xFF; //0 to 100 range for duty cycle
  PWMDTY2 = 0;   //motor is initlally stopped
  
  PWMPER1 = 0xFF; //0 to 100 range for duty cycle
  PWMDTY1 = 0;   //motor is initlally stoppe
  
  PWMPER0 = 0xFF; //0 to 100 range for duty cycle
  PWMDTY0 = 0;   //motor is initlally stoppe
  

  //Initialize interrupts 
  CRGINT = 0x80; //enable RTI
  RTICTL = 0x1F;  //set to 2.048 ms
  
  //SPI Intilizations 
  SPIBR = 0x1; //10
  SPICR1 = 0x50;
  SPICR2 = 0x00;
  
  //TIM Initilizations
  TSCR1 = 0x80; //enable timer
  TIOS = 0x80; //ch 7 for output cmpre
  TSCR2 = 0x0C; //enable reset after OC7 and prescalar of 16
  TC7 = 1500;  //10ms interrupt rate
  TIE = 0x80; //enable for ch 7
  
  PACTL = 0x40;

/* 
   Initialize the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up     
*/ 
  PTT_PTT6 =  1; //clocks is high
  PTT_PTT5 =  0; //Write mode
  
  send_i(LCDON);
  send_i(TWOLINE);
  send_i(LCDCLR);
  
  lcdwait(); //2ms delay              
}

    
/*     
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  DisableInterrupts
  initializations();     
  EnableInterrupts;
    
  for(;;) {
 
  /* < start of your main loop > */
    
    getReadings();
      
      if ((isForward == 1) && (isLeft == 1)) {
         drive(0,230,70);
      } else if ((isForward == 1) && (isRight == 1)){
          drive(180,0,20);
      } else if ((isBackward == 1) && (isLeft == 1)){
          drive(150,-200, 20);
      } else if ((isBackward == 1) && (isRight == 1)){
         drive(-190,150, 20);
      }
    
    if (leftpb == 1) {
      leftpb = 0;
      carState = 1;
      detectParkingState = 1;
    }
    
    if (rghtpb == 1){
      rghtpb = 0;
      carState = 0;
      changeDistance = 0;
    }
    
    if (carState == 0) {
      send_i(LCDCLR);
      pmsglcd("Ready to Park");
    }
    
  } /* loop forever */ 
} /* do not leave main */

void getReadings() {
   i = 0;
   while (i < 1000) {
        ATDCTL5 = 0x10; //start conversion sequence, starting on channel 0       
        while(ATDSTAT0_SCF == 0) { ; }
        front_sense        = front_sense        + ATDDR0H; //copy converted value
        right_one_sense    = right_one_sense    + ATDDR2H; //copy converted value
        right_two_sense    = right_two_sense    + ATDDR3H; //copy converted value
        back_sense         = back_sense         + ATDDR1H; //copy converted value      
        i++ ;
   }     
   i = 0 ;      
   final_front_sense         = front_sense / 1000 ;
   final_right_one_sense     = right_one_sense / 1000; 
   final_right_two_sense     = right_two_sense / 1000;
   final_back_sense          = back_sense / 1000;    
   front_sense = 0 ;
   right_one_sense = 0;
   right_two_sense = 0;
   back_sense = 0;
   
}

void drive (int left, int right, int time){
   int outer = 1000;
   int inner = time;
    
   while (outer != 0) { 
      inner = time;
      while (inner != 0){ 
        if (right >= 0) {
          PWMDTY1 = right;
          PWMDTY0 = 0;
        } else {
          PWMDTY1 = 0;
          PWMDTY0 = -1 * right;
        }
        if (left >= 0){
          PWMDTY3 = left;
          PWMDTY2 = 0;
        } else {
          PWMDTY3 = 0;
          PWMDTY2 = -1 * left;
        }
        inner--;
      }
      outer--;
    }
}

void fbStateMachine() {
  
  if (fbState == 0){
    PWMDTY1 = 0;
    PWMDTY0 = 0;
    PWMDTY3 = 0;
    PWMDTY2 = 0;
  } else if ((fbState == 1) && (isForward == 1)){
    PWMDTY1 = 170;
    PWMDTY0 = 0;
    PWMDTY3 = 140;
    PWMDTY2 = 0;
  } else if ((fbState == 1) && (isForward == 0)){
    PWMDTY1 = 0;
    PWMDTY0 = 170;
    PWMDTY3 = 0;
    PWMDTY2 = 140;
    fbState = 0;
  } else if ((fbState == 2) && (isBackward == 1)){
    PWMDTY1 = 0;
    PWMDTY0 = 170;
    PWMDTY3 = 0;
    PWMDTY2 = 140;
  } else if ((fbState == 2) && (isBackward == 0)){
    PWMDTY1 = 170;
    PWMDTY0 = 0;
    PWMDTY3 = 140;
    PWMDTY2 = 0;
    fbState = 0;
  }

}

void moveCar(){

  if (carState == 0) {
      
    fbState = 0;
    isForward = 0;
    isBackward = 0;
    isLeft = 0;
    isRight = 0;
    detectParkingState = 0;
  
  } else if (carState == 1) {
    
    fbState = 1;
    isForward = 1;
    detectParking();  
  
  } else if (carState == 2) {
    
    parkCar();
  
  }
  
}

void parkCar() {
  
  diff = final_right_one_sense - final_right_two_sense;
  
  if (parkCarState == -1) {
     
    // Error detection
    // Move forward a certain amount
    
    if (final_back_sense < (changeDistance + 40)) {
      isForward = 1;
      fbState = 1;
    } else {
      isForward = 0;
      parkCarState = 2;
    }
      
  } else if (parkCarState == 0) {
         
    send_i(LCDCLR);
    pmsglcd("Parking Car");
    
    // Move forward a certain amount
    if (((diff < 3) && (diff >= 0)) || ((diff > -3) && (diff <= 0))) {
      fbState = 1;
      isForward = 1;
    } else {
      isForward = 0;
      parkCarState = 1;
    }
  
  } else if (parkCarState == 1) {
  
    // reverse inward till a certain amount
    
    if (diff > 50) {
      parkCarState = -1;
    }
    
    if (final_back_sense > (changeDistance + 15)) {       
      fbState = 2;
      isForward = 0;
      isBackward = 1;
      isRight = 1;    
    } else { 
      isRight = 0;
      isBackward = 0; 
      if (final_back_sense < 50) {
        parkCarState = -1;
      } else {
        parkCarState = 2;
      }  
    }
    
  } else if (parkCarState == 2) {
    
    // move backward a certain amount
    if (final_back_sense > (changeDistance + 15)) {
       fbState = 2;
       isBackward = 1;
    } else {
      isBackward = 0;
      if (final_back_sense < 20) {
        parkCarState = -1;
      } else {
        parkCarState = 3;
      }
    }
  
  } else if (parkCarState == 3) {
  
    // reverse outward till a certain amount
    
    if (final_back_sense <= 20) {

      parkCarState = 4;
      isLeft = 0;
      isBackward = 0;
    } else if ((diff < 8) && (diff >= 0) || ((diff > -8) && (diff <= 0))) {
      isLeft = 0;
    } else {
      fbState = 2;
      isBackward = 1;
      isLeft = 1;
    }
                  
  } else if (parkCarState == 4) {
    send_i(LCDCLR);
    pmsglcd("Car Parked");
  }
}

void detectEdge() {
 
 diff = final_right_one_sense - final_right_two_sense;
 if (((diff < 5) && (diff >= 0)) || ((diff > -10) && (diff <= 0))) {
    changeDistance = (final_right_two_sense + final_right_one_sense) / 2;
    isLeft = 0;
    isRight = 0;
 } else if ((diff >= 5) && (diff < 30)) {
    isLeft = 0;
    isRight = 1;  
 } else if ((diff <= -10) && (diff > -40)){
    isLeft = 1;
    isRight = 0;
 } else {
    detectParkingState++;
 }
 
}

void detectParking() {
  if (detectParkingState == 1) {
    
    count = 0;
    send_i(LCDCLR);
    pmsglcd("Detect Parking");
    
    detectEdge();
  
  } else if (detectParkingState == 2) {
    
    if (count == 0) {  
      timeTaken = 0;  // start timer
    }
    count++;
  
    diff = final_right_one_sense - final_right_two_sense;
    if ((timeTaken > 80) && (((diff < 20) && (diff >= 0)) || ((diff > -20) && (diff <= 0)))) {
      detectParkingState = 3;
    } else {
      isLeft = 0;
      isRight = 0;
      isForward = 1;
      fbState = 1;
    }
    
  } else if (detectParkingState == 3) {
    
    
    if (timeTaken > 180) {
      carState = 2;
      parkCarState = 0;
    } else {  
      detectEdge();
    }
    
  } else if (detectParkingState == 4) {
    
    isForward = 0;
    isLeft = 0;
    isRight = 0;
    
    carState = 2;
    parkCarState = 0;
    
    if (timeTaken < 140) {
      carState = 0;
    }

  }
}

/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  // clear RTI interrupt flag
  CRGFLG = CRGFLG | 0x80;
  
  if ((prevpbl == 1) && (PORTAD0_PTAD7 == 0)){
        leftpb = 1;
    } 
   
    prevpbl = PORTAD0_PTAD7;

    if ((prevpbr == 1) && (PORTAD0_PTAD6 == 0)){
        rghtpb = 1;
    }
    
    prevpbr = PORTAD0_PTAD6; 

}

/*
***********************************************************************                       
  TIM interrupt service routine

  Initialized for 10.0 ms interrupt rate

  Uses variable "tencnt" to track if one-tenth second has accumulated
     and sets "tenths" flag 
                         
  Uses variable "onecnt" to track if one second has accumulated and
     sets "onesec" flag     
;***********************************************************************/

interrupt 15 void TIM_ISR(void)
{
  // clear TIM CH 7 interrupt flag 
  TFLG1 = TFLG1 | 0x80;
  
  tencnt++;
  
  if (tencnt == 5) { // Call every 50 ms  
    fbStateMachine();
  }
  
  if(tencnt == 10) {  // Call every 100 ms      
    timeTaken++;
    fbStateMachine();
    moveCar();
    tencnt = 0;
  }
 
}

/*
***********************************************************************                       
  SCI (transmit section) interrupt service routine
                         
    - read status register to enable TDR write
    - check status of TBUF: if EMPTY, disable SCI transmit interrupts and exit; else, continue
    - access character from TBUF[TOUT]
    - output character to SCI TDR
    - increment TOUT mod TSIZE 

  NOTE: DO NOT USE OUTCHAR (except for debugging)     
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{

  if(SCISR1_TDRE == 1) { //read status
    if(tin == tout) {
      SCICR2_SCTIE = 0; //disable interrupt    
    } else {
      SCIDRL = tbuf[tout]; //access character
/**************************** NEED TO OUTPUT CHARACTER ****************/
      tout = (tout + 1) % TSIZE; //increment tout mod tsize
    }
  }
 
}


/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  It should shift MSB first.  
            MISO = PM[4]
            SCK  = PM[5]
***********************************************************************
*/
 
void shiftout(char x)

{
 
  // read the SPTEF bit, continue if bit is 1
  // write data to SPI data register
  // wait for 30 cycles for SPI data to shift out
  
  int i;
  
    while(!SPISR_SPTEF) {;}   
    SPIDR = x;
    for(i = 0; i < 30 ;i++) {
    } 

}


/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
      int i_count;
  
      for(i_count = 0; i_count < 5000; i_count++) {
        ;
      } 
 
}

/*
*********************************************************************** 
  send_byte: writes character x to the LCD
***********************************************************************
*/

void send_byte(char x)
{
     // shift out character
     // pulse LCD clock line low->high->low
     // wait 2 ms for LCD to process data
     
     shiftout(x);
     
     PTT_PTT6 = 0;
     PTT_PTT6 = 1;
     PTT_PTT6 = 0;
     
     lcdwait();
}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD  
***********************************************************************
*/

void send_i(char x)
{
        // set the register select line low (instruction data)
        // send byte
        
        PTT_PTT4 = 0;
        send_byte(x);
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
   send_i(CURMOV);
   send_i(x); //position to move the cursor to
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD            
***********************************************************************
*/
 
void print_c(char x)
{
    PTT_PTT4 = 1;
    send_byte(x);
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[])
{ 

  int i = 0;
  while(str[i] != '\0') {
    print_c(str[i]);
    i++;
  }

}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}