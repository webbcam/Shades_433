#include <RCSwitch.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Signal_Parser.h>

#define OFFSET 75
#define SHADES_MAX 767
#define SHADES_MIN 702
#define SPEED 15
#define INTERVAL 5
#define SERVO_PIN 9
#define BUTTON_PIN 3
#define LED_PIN 12
#define TIMEOUT 3000

Servo myServo;
RCSwitch mySwitch = RCSwitch();

//  get shade ID
byte shade_ID = EEPROM.read(0);
//  get remote ID
unsigned int remote_ID = ( EEPROM.read(1) << 8) | EEPROM.read(2);
//  set initial shades postion
int pos = OFFSET;


void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  mySwitch.enableReceive(0);  //  Receiver on interrupt 0 => pin #2
  //attachInterrupt(1, set_remote_ID, FALLING);  // Button on interrupt 1 => pin #3
  Serial.print("shade_ID is: ");
  Serial.println(shade_ID);
  Serial.print("remote_ID is: ");
  Serial.println(remote_ID);
}

void loop() {
  //  check for remote_ID reset
  if (digitalRead(BUTTON_PIN)) {
    set_remote_ID();
  }
  //  check for new signal
  if ( mySwitch.available() ) {
    unsigned long value = mySwitch.getReceivedValue();
    unsigned int preamble = parse_preamble(value);
    
    // get data from signal
    if (preamble == PREAMBLE) {
      byte postamble = parse_postamble(value);
      unsigned int load = parse_load(value);
      //  check for shades signal (from BLE Box)
      if (load >= SHADES_MIN && load <= SHADES_MAX) {
        //  check for corresponding shade
        if (shade_ID == load & 0x30) {
          int deg;
          Serial.println("RECIEVED SIGNAL FROM BLUETOOTH!!!");
          //  calculate degree from signal with offset and 3 degree intervals
          if (postamble == POST_ON)   //  positive value
            deg = OFFSET + INTERVAL*(load & 0xF);
          else if (postamble == POST_OFF)
            deg = OFFSET - INTERVAL*(load & 0xF);  //   negative value
          
          else if (postamble < 12 && postamble > 7) //  reset shade_ID
            set_shade_ID(postamble);
            
          //  write calculated degree to servo
          adjust_shade(deg);
        }
        //  check for signal from remote
      } else if (load == remote_ID) {
        Serial.println("RECIEVED SIGNAL FROM REMOTE!!!");
        if (postamble == POST_ON)
          adjust_shade(0);    // open shades
        else if (postamble == POST_OFF)
          adjust_shade(150);  // close shades
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

void set_shade_ID(byte id) {
  shade_ID = id & 0x3;  //  get the ID from postamble of form 10XX
  EEPROM.update(0, shade_ID);
  //  signal that the new shade_ID has been successfully set
  for (int i = 0; i < 4; i++) {
    digitalWrite(12, HIGH);
    delay(100);
    digitalWrite(12, LOW);
    delay(100);
  }
}

void set_remote_ID(void) {
  unsigned int load;
  unsigned long timeout;
  byte lsb, msb;
  //  clear any signal stored
  mySwitch.resetAvailable();
  //  signal ready for signal
  digitalWrite(12, HIGH);
  timeout = millis() + TIMEOUT;
  while (!mySwitch.available() && millis() < timeout)
    ; // wait until we get a signal
  load = parse_load(mySwitch.getReceivedValue());
  //  if new signal, save it
  if (remote_ID != load) {
    remote_ID = load;
    msb = (load >> 8) & 0xFF;
    lsb = load & 0xFF;

    //  save the new remote_ID (big endian)
    EEPROM.update(1, msb);
    EEPROM.update(2, lsb);
  }

  //  signal that the new remote_ID has been successfully set
  for (int i = 0; i < 4; i++) {
    digitalWrite(12, HIGH);
    delay(100);
    digitalWrite(12, LOW);
    delay(100);
  }

  mySwitch.resetAvailable();
}

