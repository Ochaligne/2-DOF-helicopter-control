/* FILE:    ARD_LCD_HCARDU0023_I2C_Hello_World_Example.pde
   DATE:    07/09/12
   VERSION: 0.1
   
DEVICE PINOUT (SPI Interface):

PIN 1: GND
PIN 2: +5V
PIN 3: SDA - Connect to Arduino analogue PIN 4
PIN 4: SCL - Connect to Arduino analogue PIN 5


You may copy, alter and reuse this code in any way you like but please leave 
reference to hobbycomponents.com in your comments if you redistribute this code. */


/* Include the SPI/IIC Library */
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


/* Initialise the LiquidCrystal library. Note that the displays will have a default I2C
    address of 0x27. */

//LiquidCrystal_I2C lcd(0x27,20,4);
LiquidCrystal_I2C lcd(0x27,20,4);
  /* Declare variables */
  const int up = 40;        //Up button
  boolean current_up = LOW; //Currant state
  boolean last_up=LOW;      //Last state
  int page_counter=1 ;      //page counter variable
  

void setup() 
{

  pinMode(up, INPUT);
  /* Initialise the LCD */
  //lcd.init();
  lcd.init();
  /* Make sure the backlight is turned on */
  lcd.backlight();
}
//function//
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
/* Main program loop */
void loop() 
{
  

  current_up = debounce(last_up, up); //Debounce for Up button
  Serial.print(current_up);
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

     case 4: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("This is pos 3");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 5: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("This is pos 4");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 6: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("This is pos 5");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 7: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("This is pos 6");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

     case 8: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("This is pos 7");
     lcd.setCursor(1,1);
     lcd.print("B for Select");
    }
    break;

    case 9: {   //Design of page 3 
     lcd.setCursor(0,0);
     lcd.print("Automode");
     lcd.setCursor(1,1);
     lcd.print("RUNING");
    }
    break;

     
  }//switch end
  
}
