/* YourDuinoStarter Example: nRF24L01 Receive Joystick values

 - WHAT IT DOES: Receives data from another transceiver with
   2 Analog values from a Joystick or 2 Potentiometers
   Displays received values on Serial Monitor
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
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"
#include <Servo.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define CE_PIN  9
#define CSN_PIN 53

// NOTE: the "LL" at the end of the constant is "LongLong" type
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe


/*-----( Declare objects )-----*/
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio
/*-----( Declare Variables )-----*/
int joystick[4];  // 4 element array holding Joystick readings
Servo myservo[4];
int servo_throttle,servo_yaw,servo_pitch,servo_roll;
int pid_roll,pid_pitch,pid_yaw;
int motor[4];
//reset
void resetData() 
{
  // 'safe' values to use when no radio input is detected
  servo_throttle  =  map(0,0, 255,544,2400);
  servo_yaw       =  map(127,0, 255,544,2400);
  servo_pitch     =  map(127,0, 255,544,2400);
  servo_roll      =  map(127,0, 255,544,2400);
}


int PIDMIX(int x,int y,int z)
{
    pid_roll=0;
    pid_pitch=0;
    pid_yaw=0;
    servo_throttle =1500;
    return servo_throttle + pid_roll *x +pid_pitch *y + pid_yaw*z;
}

void mixtable()
{
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R 後右
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R前右
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L 後左
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L前左
}

void writeMotor()//Writes the Motors values to the PWM compare register
{
   //min = 1000*16-16000 = 0 max = 2000*16-16000 = 16000
   // [1000:2000] => [8000:16000] for timer 3 & 4 for mega
   OCR3C = ( ( motor[0]<<4 ) - 16000);
   OCR3A = ( ( motor[1]<<4 ) - 16000);
   OCR4A = ( ( motor[2]<<4 ) - 16000);
   OCR3B = ( ( motor[3]<<4 ) - 16000);
}

void servoInitial()//output.cpp line 507
{
      pinMode(2,OUTPUT);
      pinMode(3,OUTPUT);
      pinMode(5,OUTPUT);
      pinMode(6,OUTPUT);
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3FFF; // TOP to 16383;      
      TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B |= (1<<WGM43);
      TCCR4B &= ~(1<<CS41); // no prescaler
      ICR4   |= 0x3FFF; // TOP to 16383;        
      TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
      TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B 
}

void setup()   /****** SETUP: RUNS ONCE ******/
{
  int i;
  Serial.begin(9600);
  //resetData();
  printf_begin();
  Serial.println("Nrf24L01 Receiver Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipe);
  radio.setAutoAck(false);
  radio.startListening();
  radio.printDetails();
  servoInitial();

  
  OCR3B = (16350); 
  delay(2000);
  delay(5000);//保留 逼逼至56712 油門行程設定
  
  /*
  delay(6000);
  OCR3B = (12000); 
  delay(5000);
  OCR3B = (16350); //可設定至剎車選項
  */ 
  /*
  OCR3B=12000;
  delay(5000);//正常使用開機過程
  */ 
}//--(end setup )---

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  //OCR3B = (16350);//min = 12182 max=16350;
  int temp;
  if ( radio.available() ) //radio.available 不會更改status
  {
    Serial.println("available");
    // Read the data payload until we've received everything
    bool done = false;
    while (!done)
    {
      // Fetch the data payload
      done = radio.read( joystick, sizeof(joystick) );
      
      /*
      Serial.print(" ");
      Serial.print(joystick[0]);
      Serial.print(" ");      
      Serial.print(joystick[1]);
       Serial.print(" ");      
      Serial.print(joystick[2]);
       Serial.print(" ");      
      Serial.println(joystick[3]); //加了以上會正常 不會印出 NO available
      */
      
      temp=map(joystick[0],0,1023,13000,16350);
      OCR3B = temp;
      servo_throttle = joystick[0];
      servo_yaw      = joystick[1];
      servo_pitch    = joystick[2];
      servo_roll     = joystick[3];
      /*
      myservo[0].writeMicroseconds(servo_throttle + servo_roll - servo_pitch);
      myservo[1].writeMicroseconds(servo_throttle - servo_roll - servo_pitch);
      myservo[2].writeMicroseconds(servo_throttle + servo_roll + servo_pitch); 
      myservo[3].writeMicroseconds(servo_throttle - servo_roll + servo_pitch);
      */
      
    }
  }
  else
  {   
      OCR3B = (16350);
      Serial.println("No radio available");
  }
}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/

//NONE
//*********( THE END )***********
