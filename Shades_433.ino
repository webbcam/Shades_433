#include <RCSwitch.h>
#include <EEPROM.h>
#include "Signal_Parser.h"
#include <Servo.h>          //  Uncomment for 16-bit MCUs
//#include <Adafruit_SoftServo.h> //  Uncomment for 8-bit MCUs

#define PREAMBLE 666
#define SHADES_PREAMBLE 10667
#define PARSE_SHADES_PREAMBLE(sig)  (( (sig) >> 10) & 0x3FFF)

//  SHADES SETTINGS:
#define OFFSET 75
#define SPEED 15
#define INTERVAL 5

#define SHADES_MIN 704
#define SHADES_MAX 767

//  PINS:
#define SERVO_PIN 4
#define BUTTON_PIN 8
#define LED_PIN A1
#define INTERRUPT_PIN 0

//  SYNC SETTINGS:
#define TIMEOUT 3000
#define BLINK_SPD 100



Servo myServo;              //  Uncomment for 16-bit MCUs
//Adafruit_SoftServo myServo;   //  Uncomment for 8-bit MCUs

RCSwitch mySwitch = RCSwitch();

//  get shade ID
byte shade_ID = EEPROM.read(0);
//byte shade_ID = 0;

unsigned long remote_ID[3];         //  array to hold remote_IDs (20-bits long)
byte EEPROM_ptr = EEPROM.read(1);   //  used as EEPROM stack ptr (points to most recently added remote ID)

byte ID_ptr = EEPROM_ptr/2 - 1;     //  used to traverse thru remote_ID array-addresses (i.e. 0,1,2)
//  set initial shades postion
int pos = OFFSET;


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT);

  //  Initialize board if it has not been initialized
  if (shade_ID > 3) {
    EEPROM.write(0,0);
    shade_ID = 0;
  }
  if (EEPROM_ptr < 2 || EEPROM_ptr > 6) {
    EEPROM.write(1,2);
    EEPROM_ptr = 2;
    ID_ptr = EEPROM_ptr/2 - 1;
  }
  
  //  get remote IDs (x3)
  byte addr = 2;
  for (int i = 0; i < 3; i++) {
    byte  msb = EEPROM.read(addr);     //  1st byte (MSB)
    byte  lsb = EEPROM.read(addr + 1); //  2nd byte (LSB)
    byte ch;
    addr += 2;  //  move to next EEPROM address
    //  Store the Preamble in the remote_ID array
    remote_ID[i] = (msb << 2) | ((lsb & 0xC0) >> 6);  //  combine msb and lsb to make preamble
    remote_ID[i] <<= 10;  //  shift preamble to most significant 10-bits
    
//    remote_ID[i] = ((msb << 2) | ((lsb & 0xC0) >> 6)) << 10;

    //  Decode and store the Channel/Load in the remote_ID array
    ch = lsb & 0x7;
    switch(ch) {
      case 1: remote_ID[i] |= CH1; break;
      case 2: remote_ID[i] |= CH2; break;
      case 3: remote_ID[i] |= CH3; break;
      case 4: remote_ID[i] |= CH4; break;
      case 5: remote_ID[i] |= CH5; break;
    }
  }
  
  mySwitch.enableReceive(INTERRUPT_PIN);  //  Receiver on interrupt 1 => pin #3
}

void loop() {
  //  check for remote_ID reset
  if (digitalRead(BUTTON_PIN)) {
    sync();
  }
  //  check for new signal
  if ( mySwitch.available() ) {
    unsigned long value = mySwitch.getReceivedValue();
    unsigned int preamble = parse_preamble(value);
    byte postamble = parse_postamble(value);
    
    // get data from signal if using our preamble/range of signals
    if (preamble == PREAMBLE) {
      
      unsigned int load = parse_load(value);
      //  check for shades signal (from BLE Box)

        if (shade_ID == ((load >> 4) & 0x3)) {
                if (load >= SHADES_MIN && load <= SHADES_MAX) {

          int deg;
          //  calculate degree from signal with offset and 3 degree intervals
          if (postamble == POST_ON) {   //  positive value
            deg = OFFSET + INTERVAL*(load & 0xF);
            //  write calculated degree to servo
            adjust_shade(deg);
            
          } else if (postamble == POST_OFF) {
            deg = OFFSET - INTERVAL*(load & 0xF);  //   negative value
            //  write calculated degree to servo
            adjust_shade(deg);
            
          } /*else if (postamble < 12 && postamble > 7) { //  reset shade_ID
            set_shade_ID(postamble);
          }*/
        }
      } 
      //  check for signal from remote
    } else {
      unsigned long val = value >> 4; // remove postamble and store value
      if (val == remote_ID[0] || val == remote_ID[1] || val == remote_ID[2]) {
        if (postamble == POST_ON) {
          digitalWrite(LED_PIN, LOW);
          adjust_shade(25);    // open shades
        } else if (postamble == POST_OFF) {
          digitalWrite(LED_PIN, HIGH);
          adjust_shade(150);  // close shades
        }
      }
    }
    
    mySwitch.resetAvailable();
  }
}

void adjust_shade(int deg) {
  myServo.attach(SERVO_PIN);
  int sign;
  if (pos > deg)
    sign = -1;
  else
    sign = 1;
  while (pos != deg) {
    pos += sign;
    myServo.write(pos);
    delay(SPEED);
  }
  myServo.detach();
}

void sync(void) {
  unsigned long sig;      //  holds the signal
  unsigned long timeout;  //  if timeout occurs, erase stored signal
  //  clear any signal stored
  mySwitch.resetAvailable();
  //  receiver ready for signal
  digitalWrite(LED_PIN, LOW);
  timeout = millis() + TIMEOUT;
  while (!mySwitch.available() && millis() < timeout)
    ; // wait until we get a signal
    
  sig = mySwitch.getReceivedValue();
  if ( PARSE_SHADES_PREAMBLE(sig) == SHADES_PREAMBLE) {
    byte postamble = parse_postamble(sig);
    if (postamble > 7 && postamble < 12)
      set_shade_ID(postamble);
  } else {
    set_remote_ID(sig);
  }
  
  delay(500);
  mySwitch.resetAvailable();
  digitalWrite(LED_PIN, HIGH);
  
}

void set_shade_ID(byte id) {
  shade_ID = id & 0x3;  //  get the ID from postamble of form 10XX
  EEPROM.write(0, shade_ID);
  //  signal that the new shade_ID has been successfully set
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_SPD);
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_SPD);
  }
}

void set_remote_ID(unsigned long sig) {
  unsigned int ch;        //  holds the 10-bit channel/load
  byte lsb, msb;
  
  sig = sig >> 4;
  //  if new signal, save it
  if (remote_ID[ID_ptr] != sig) {
    remote_ID[ID_ptr] = sig;
    msb = (sig >> 12) & 0xFF;   //  first 8-bits of preamble
    lsb = (sig >> 4) & 0xC0;    //  last 2-bits of preamble

    ch = (sig & 0x3FF);         //  last 10-bits of signal (channel/load)

    switch (ch) {
      case CH1: lsb |= 1; break;
      case CH2: lsb |= 2; break;
      case CH3: lsb |= 3; break;
      case CH4: lsb |= 4; break;
      case CH5: lsb |= 5; break;
    }

    //  save the new remote_ID (big endian)
    EEPROM.write(EEPROM_ptr, msb);
    EEPROM.write(EEPROM_ptr + 1, lsb);
  }
    //  adjust pointers
    if (EEPROM_ptr != 6)
      EEPROM_ptr += 2;
    else
      EEPROM_ptr = 2;
    ID_ptr = EEPROM_ptr/2 - 1;

    EEPROM.write(1, EEPROM_ptr);

  //  signal that the new remote_ID has been successfully set
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_SPD);
    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_SPD);
  }

  mySwitch.resetAvailable();
}

