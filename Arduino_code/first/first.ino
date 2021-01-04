
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
#include <LiquidCrystal_I2C.h>
// I2Cdev and MPU6050 libraries required
#include "I2Cdev.h"
#include "MPU6050.h"
#include <AFMotor.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
AF_DCMotor motor(4);
LiquidCrystal_I2C lcd(0x27,20,4);

int16_t ax, ay, az;
int16_t gx, gy, gz;
double output;
double intError;
double rateError;
double error;
word VentPin = 3;
double kp;
double kd;
double ki;
double currentTime;
double elapsedTime;
double previousTime;
double angle;
double setpoint;
double lastError;

//encoder variables//
int encoderPin1 = 18;
int encoderPin2 = 19;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

//Constants for LCD

const int up = 40;        //Up button
boolean current_up = LOW; //Currant state
boolean last_up=LOW;      //Last state
int page_counter=1 ;      //page counter variable

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(up, INPUT);
    pinMode(VentPin, OUTPUT);
    pinMode(encoderPin1, INPUT); 
    pinMode(encoderPin2, INPUT);
    
    pwm25kHzBegin();
    lcd.init();
    lcd.backlight();

      //the model of encoder is with open-collector output
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

    //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(4, updateEncoder, CHANGE); 
  attachInterrupt(5, updateEncoder, CHANGE);
}



//function for detecting button change//
boolean debounce(boolean last, int pin)
{
boolean current = digitalRead(pin);
if (last != current) 
{
delay(5);
current = digitalRead(pin);
}
return current;
}
//End of function//



void loop() {
 setpoint = 30;
 kp=1;
 accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 currentTime = millis();
 elapsedTime = (double)(currentTime - previousTime);
 elapsedTime = elapsedTime/1000;
 Serial.print("Yaw angle:");
 Serial.print(encoderValue*0.225);

  current_up = debounce(last_up, up); //Debounce for Up button
 
  if (last_up== LOW && current_up == HIGH){  //When up button is pressed
      lcd.clear();                     //When page is changed, lcd clear to print new page  
      if(page_counter <9){              //Page counter never higher than 3(total of pages)
      page_counter= page_counter +1;   //Page up
      
      }
      else{
      page_counter = 1;  
      }
  }
  last_up=current_up; //store the last state of button

 angle = (atan2(ay,az))*RAD_TO_DEG; //function for degree calculation
  
 error = setpoint - angle;

 /////// Equation calculating integral error ///////
intError += error * elapsedTime;

/////// Equation calculating derivative error ///////
rateError = (error - lastError)/elapsedTime;

/////// Equation calculating PID controller output ///////
output = error*kp + intError*ki + rateError*kd;

/////// Constraining the output to not exceed the maximum speed of motors ///////
output = constrain(output,1.25,100);//constrain the output

/////// Setting motor speed to the output of the PID controller ///////

Serial.print("    Pitch angle:");
Serial.println(angle);
delay(10);
pwmDuty(50); // speed control% (range = 0-79 = 1.25-100%)

switch (page_counter) {
   
    case 1:{     //Design of home page 1
      lcd.setCursor(0,0);
      lcd.print("A for Scroll");
      lcd.setCursor(0,1);
      lcd.print("B for Select");
    }
    break;

    case 2: { //Design of page 2 
     lcd.setCursor(0,0);
     lcd.print("This is pos 1");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

    case 3: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("This is pos 2");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 4: {   //Design of page 4 
     lcd.setCursor(0,0);
     lcd.print("This is pos 3");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 5: {   //Design of page 5 
     lcd.setCursor(0,0);
     lcd.print("This is pos 4");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 6: {   //Design of page 6 
     lcd.setCursor(0,0);
     lcd.print("This is pos 5");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 7: {   //Design of page 7 
     lcd.setCursor(0,0);
     lcd.print("This is pos 6");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 8: {   //Design of page 8 
     lcd.setCursor(0,0);
     lcd.print("This is pos 7");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

    case 9: {   //Design of page 9 
     lcd.setCursor(0,0);
     lcd.print("Automode");
     lcd.setCursor(1,1);
     lcd.print("RUNING..");
    }
    break;

     
  }//switch end




previousTime = currentTime;
lastError = error;
}

void pwm25kHzBegin() {
  TCCR2A = 0;                               // TC2 Control Register A
  TCCR2B = 0;                               // TC2 Control Register B
  TIMSK2 = 0;                               // TC2 Interrupt Mask Register
  TIFR2 = 0;                                // TC2 Interrupt Flag Register
  TCCR2A |= (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // OC2B cleared/set on match when up/down counting, fast PWM
  TCCR2B |= (1 << WGM22) | (1 << CS21);     // prescaler 8
  OCR2A = 79;                               // TOP overflow value (Hz)
  OCR2B = 0;
}

void pwmDuty(byte ocrb) {
  OCR2B = ocrb;                             // PWM Width (duty)
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

//Serial.print(MSB); Serial.print("\t");
//Serial.print(LSB); Serial.print("\t");

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
}
