/* YourDuinoStarter Example: nRF24L01 Transmit Joystick values
 - WHAT IT DOES: Reads Analog values on A0, A1 and transmits
   them over a nRF24L01 Radio Link to another transceiver.
 - SEE the comments after "//" on each line below
 - CONNECTIONS: nRF24L01 Modules See:
 http://arduino-info.wikispaces.com/Nrf24L01-2.4GHz-HowTo
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 9
   4 - CSN to Arduino pin 53
   5 - SCK to Arduino pin 52
   6 - MOSI to Arduino pin 51
   7 - MISO to Arduino pin 50
   8 - UNUSED
   - 
   Analog Joystick or two 10K potentiometers:
   GND to Arduino GND
   VCC to Arduino +5V
   X Pot to Arduino A0
   Y Pot to Arduino A1
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN   9
#define CSN_PIN 53

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int joystick[4];  // 4 element array holding Joystick readings

void resetData() 
{
  joystick[0] = 0;
  joystick[1] = 127;
  joystick[2] = 127;
  joystick[3] = 127;
}//end resetData

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}//end mapJoystickValues


void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(9600);
  printf_begin();
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipe);
  radio.setAutoAck(false);
  radio.printDetails();
  resetData();
}//--(end setup )---

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  /*
  joystick[0] = mapJoystickValues( analogRead(A0), 0, 522, 1023, true );//throttle
  joystick[1] = mapJoystickValues( analogRead(A1), 0, 506, 1023, true );//yaw
  joystick[2] = mapJoystickValues( analogRead(A2), 0, 500, 1023, true );//pitch
  joystick[3] = mapJoystickValues( analogRead(A3), 0, 502, 1023, true );//roll
  */
  /*
  joystick[0] = map(analogRead(A0),0,1023,1000,2000); //throttle
  joystick[1] = map(analogRead(A1),0,1023,-512,511);  //yaw
  joystick[2] = map(analogRead(A2),0,1023,-512,511);  //pitch
  joystick[3] = map(analogRead(A3),0,1023,-512,511);  //roll
  */
  joystick[0] = analogRead(A0);
  joystick[1] = analogRead(A1);
  joystick[2] = analogRead(A2);
  joystick[3] = analogRead(A3);
  bool x=radio.write( joystick, sizeof(joystick) );
  /*
  Serial.print("x =");
  Serial.print(x);
  
  Serial.print(" ,");
  Serial.print(joystick[0]);
  Serial.print(" ,");
  Serial.print(joystick[1]);
  Serial.print(" ,");
  Serial.print(joystick[2]);
  Serial.print(" ,");
  Serial.println(joystick[3]);
  */
}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/

//NONE
//*********( THE END )***********
