
#include <FastLED.h>
//#include "PinChangeInterrupt.h"
#define NUM_LEDS 2

#define BRIGHTNESS  255  // Put 0 to switch off all leds

#define DATA_PIN_STATUS 7
#define DATA_PIN_FRONT 8
#define DATA_PIN_BACK 9


#define MODE_WARNING 0
#define MODE_DISARMED_NO_GPS 1
#define MODE_DISARMED_GPS 2
#define MODE_ARMED_NO_GPS 3
#define MODE_ARMED_GPS 4

const unsigned int pulse_width = 1000;

unsigned long last_armed_status = 0;
unsigned long last_gps_status = 0;
unsigned long last_warn_status = 0;

bool armed = false;
bool gpsfix = false;
bool warning = false;
bool front_lights_disabled = false;


byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_3;

CRGB leds_status[1];
CRGB leds_front[NUM_LEDS];
CRGB leds_back[NUM_LEDS];

byte led_state = 0;


#define NUM_MODOS 5
byte led_mode = MODE_DISARMED_NO_GPS;

                                           // MODE WARNING              DISARMED_NO_GPS             DISARMED_GPS                ARMED_NO_GPS                ARMED_GPS  
CRGB led_front_on1[NUM_MODOS][NUM_LEDS]  = {{CRGB::Red,  CRGB::Black},{CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_off1[NUM_MODOS][NUM_LEDS] = {{CRGB::Black,CRGB::Black},{CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_on2[NUM_MODOS][NUM_LEDS]  = {{CRGB::Red,  CRGB::Black},{CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_off2[NUM_MODOS][NUM_LEDS] = {{CRGB::Black,CRGB::Black},{CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_on3[NUM_MODOS][NUM_LEDS]  = {{CRGB::Black,CRGB::Red},  {CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_off3[NUM_MODOS][NUM_LEDS] = {{CRGB::Black,CRGB::Black},{CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_on4[NUM_MODOS][NUM_LEDS]  = {{CRGB::Black,CRGB::Black},{CRGB::Cyan,CRGB::Cyan},     {CRGB::Cyan,CRGB::Cyan},   {CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};
CRGB led_front_off[NUM_MODOS][NUM_LEDS]  = {{CRGB::Black,CRGB::Black},{CRGB::Black,CRGB::Black},   {CRGB::Black, CRGB::Black},{CRGB::Red,  CRGB::Red},    {CRGB::Red,CRGB::Red}};

CRGB led_back_on1[NUM_MODOS][NUM_LEDS]   = {{CRGB::Black,CRGB::Black},{CRGB::Yellow, CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::Green,CRGB::Green}};
CRGB led_back_off1[NUM_MODOS][NUM_LEDS]  = {{CRGB::Black,CRGB::Black},{CRGB::Yellow ,CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::Black,CRGB::Black}};
CRGB led_back_on2[NUM_MODOS][NUM_LEDS]   = {{CRGB::Red,  CRGB::Black},{CRGB::Yellow, CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::Green,CRGB::Green}};    
CRGB led_back_off2[NUM_MODOS][NUM_LEDS]  = {{CRGB::Black,CRGB::Black},{CRGB::Yellow ,CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::Black,CRGB::Black}};
CRGB led_back_on3[NUM_MODOS][NUM_LEDS]   = {{CRGB::Black,CRGB::Red},  {CRGB::Yellow, CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::Black,CRGB::Black}};    
CRGB led_back_off3[NUM_MODOS][NUM_LEDS]  = {{CRGB::Black,CRGB::Black},{CRGB::Yellow ,CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::Black,CRGB::Black}};
CRGB led_back_on4[NUM_MODOS][NUM_LEDS]   = {{CRGB::Red,  CRGB::Red},  {CRGB::Yellow, CRGB::Yellow},{CRGB::Green, CRGB::Green},{CRGB::Yellow,CRGB::Yellow},{CRGB::LightGreen,CRGB::LightGreen}};    
CRGB led_back_off[NUM_MODOS][NUM_LEDS]   = {{CRGB::Black,CRGB::Black},{CRGB::Black , CRGB::Black}, {CRGB::Black, CRGB::Black},{CRGB::Black,CRGB::Black},  {CRGB::Black,CRGB::Black}};

CRGB led_status_on1[NUM_MODOS]           = { CRGB::Red,                CRGB::Blue,                  CRGB::Cyan,                 CRGB::Red,                 CRGB::Black };
CRGB led_status_off1[NUM_MODOS]          = { CRGB::Black,              CRGB::Black,                 CRGB::Black,                CRGB::Black,               CRGB::Black };
CRGB led_status_on2[NUM_MODOS]           = { CRGB::Red,                CRGB::Black,                 CRGB::Black,                CRGB::Black,               CRGB::Black };
CRGB led_status_off2[NUM_MODOS]          = { CRGB::Black,              CRGB::Black,                 CRGB::Black,                CRGB::Black,               CRGB::Black };
CRGB led_status_on3[NUM_MODOS]           = { CRGB::Red,                CRGB::Yellow,                CRGB::Green,                CRGB::Yellow,              CRGB::Black };
CRGB led_status_off3[NUM_MODOS]          = { CRGB::Black,              CRGB::Black,                 CRGB::Black,                CRGB::Black,               CRGB::Black };
CRGB led_status_on4[NUM_MODOS]           = { CRGB::Red,                CRGB::Black,                 CRGB::Black,                CRGB::Black,               CRGB::Black };
CRGB led_status_off[NUM_MODOS]           = { CRGB::Black,              CRGB::Black,                 CRGB::Black,                CRGB::Black,               CRGB::Black };


// Lights flashing adjustment
unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long next_interval = 0;      // next interval

int time_on = 20;
int time_off = 80;
int time_cycle = 400;


void setup() {
  
  Serial.begin(57600);

  //delay(5000); // sanity delay
  FastLED.addLeds<WS2811, DATA_PIN_STATUS, GRB>(leds_status, 1);
  FastLED.addLeds<WS2811, DATA_PIN_FRONT, GRB>(leds_front, NUM_LEDS);
  FastLED.addLeds<WS2811, DATA_PIN_BACK, GRB>(leds_back, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs
  PCICR |= (1 << PCIE2);    // set PCIE0 to enable PCMSK0 scan
  PCMSK2 |= (1 << PCINT18);  // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT19);  // set PCINT1 (digital input 9)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT20);  // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT21);  // set PCINT2 (digital input 10)to trigger an interrupt on state change

  startupAnimation();
}

void setMode(byte mode) {
  led_mode = mode;
  if (mode == MODE_ARMED_GPS) {
    time_on = 20;
    time_off = 80;
    time_cycle = 1000;
  } else if (mode == MODE_WARNING){
    time_on = 20;
    time_off = 80;
    time_cycle = 80;
  } else {
    time_on = 20;
    time_off = 80;
    time_cycle = 400;
  }
}

void updateState(unsigned long currentTime) {
  if (currentTime - last_armed_status >= pulse_width) {
    armed = true;
  } else armed = false;
  
  if (currentTime - last_gps_status >= pulse_width) {
    gpsfix = true;
  } else gpsfix = false;
  
  if (currentTime - last_warn_status >= pulse_width) {
    warning = false;
  } else warning = true;
  
  if (warning) {
    setMode(MODE_WARNING);
  } else {
    if (armed) {
      if (gpsfix) setMode(MODE_ARMED_GPS); else setMode(MODE_ARMED_NO_GPS);
    } else {
      if (gpsfix) setMode(MODE_DISARMED_GPS); else setMode(MODE_DISARMED_NO_GPS);
    }
  }

  // Disable Lights if receiver input is high
  front_lights_disabled = (receiver_input_channel_4 - 1520 > 0);

}

void readCommand() {
    String c;
   while (Serial.available()) {
     c = Serial.readString();
   }
   if (c == "0") {
    setMode(MODE_WARNING);
    Serial.println("Mode set to: WARNING");
   } else if (c =="1") {
    setMode(MODE_DISARMED_NO_GPS);
    Serial.println("Mode set to: DISARMED_NO_GPS");
   } else if (c == "2") {
    setMode(MODE_DISARMED_GPS);
    Serial.println("Mode set to: DISARMED_GPS_FIX");
   } else  if (c == "3") {
    setMode(MODE_ARMED_NO_GPS);
    Serial.println("Mode set to: ARMED_NO_GPS");
   } else if (c == "4") {
    setMode(MODE_ARMED_GPS);
    Serial.println("Mode set to: ARMED_GPS_FIX");
   } else if (c == "f") {
    front_lights_disabled = !front_lights_disabled;
    if (front_lights_disabled) Serial.println("Head Lights: OFF"); else Serial.println("Head Lights: ON");
   }
}

void startupAnimation() {
    int lowtime = 200;
    int midtime = 600;
    int hightime = 800;

    // PART 1
    leds_front[0] = CRGB::Red;
    leds_front[1] = CRGB::Red;
    leds_back[0]  = CRGB::Green;
    leds_back[1]  = CRGB::Green;
    FastLED.show();
    delay(lowtime);
    leds_front[0] = CRGB::Green;
    leds_front[1] = CRGB::Green;
    leds_back[0]  = CRGB::Blue;
    leds_back[1]  = CRGB::Blue;
    FastLED.show();
    delay(lowtime);
    leds_front[0] = CRGB::Blue;
    leds_front[1] = CRGB::Blue;
    leds_back[0]  = CRGB::Magenta;
    leds_back[1]  = CRGB::Magenta;
    FastLED.show();
    delay(lowtime);
    leds_front[0] = CRGB::Black;
    leds_front[1] = CRGB::Black;
    leds_back[0]  = CRGB::Black;
    leds_back[1]  = CRGB::Black;
    FastLED.show();
    delay(hightime);

    // PART 2
    leds_front[0] = CRGB::Cyan;
    leds_front[1] = CRGB::Black;
    leds_back[0]  = CRGB::Cyan;
    leds_back[1]  = CRGB::Black;
    FastLED.show();
    delay(lowtime);
    leds_front[0] = CRGB::Black;
    leds_front[1] = CRGB::Green;
    leds_back[0]  = CRGB::Black;
    leds_back[1]  = CRGB::Green;
    FastLED.show();
    delay(lowtime);
    leds_front[0] = CRGB::Blue;
    leds_front[1] = CRGB::Black;
    leds_back[0]  = CRGB::Blue;
    leds_back[1]  = CRGB::Black;
    FastLED.show();
    delay(lowtime);
    leds_front[0] = CRGB::White;
    leds_front[1] = CRGB::White;
    leds_back[0]  = CRGB::White;
    leds_back[1]  = CRGB::White;
    FastLED.show();
    delay(midtime);
    leds_front[0] = CRGB::Black;
    leds_front[1] = CRGB::Black;
    leds_back[0]  = CRGB::Black;
    leds_back[1]  = CRGB::Black;
    FastLED.show();
}

void loop() {
  readCommand();
  // Lights management
  // Light pulses: 2 quick flashes per second
  unsigned long currentMillis = millis();
  int i=0;
  //updateState(currentMillis);
  // Normal mode, lights on.
  if (currentMillis - previousMillis >= next_interval) {
    // Keep time last mode changed
    previousMillis = currentMillis;

    // Toggle lights depending on status
    switch (led_state){
      case 0:
        {
          next_interval=time_on;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_on1[led_mode][i];
            leds_back[i]=led_back_on1[led_mode][i];
          }
          leds_status[0]= led_status_on1[led_mode];
          break;
        }
        
      case 1:
        {
          next_interval=time_off;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_off1[led_mode][i];
            leds_back[i]=led_back_off1[led_mode][i];
          }
          leds_status[0] = led_status_off1[led_mode];
          break;
        }
        
      case 2:
        {
          next_interval=time_on;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_on2[led_mode][i];
            leds_back[i]=led_back_on2[led_mode][i];
          }
          leds_status[0] = led_status_on2[led_mode];
          break;
        }

      case 4:
        {
          next_interval=time_off;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_off2[led_mode][i];
            leds_back[i]=led_back_off2[led_mode][i];
          }
          leds_status[0] = led_status_off2[led_mode];
          break;
        }
        
      case 5:
        {
          next_interval=time_on;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_on3[led_mode][i];
            leds_back[i]=led_back_on3[led_mode][i];
          }
          leds_status[0] = led_status_on3[led_mode];
          break;
        }
        case 6:
        {
          next_interval=time_off;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_off3[led_mode][i];
            leds_back[i]=led_back_off3[led_mode][i];
          }
          leds_status[0] = led_status_off3[led_mode];
          break;
        }
        
      case 7:
        {
          next_interval=time_on;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_on4[led_mode][i];
            leds_back[i]=led_back_on4[led_mode][i];
          }
          leds_status[0] = led_status_on4[led_mode];
          break;
        }
        case 8:
        {
          next_interval=time_cycle;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_off[led_mode][i];
            leds_back[i]=led_back_off[led_mode][i];
          }
          leds_status[0] = led_status_off[led_mode];
          break;
        }
        
    }
    // Show leds
    FastLED.show();
    
    // Cycle status
    led_state++;
    if(led_state >= 9) led_state=0;

  }
}

ISR(PCINT2_vect){
  //Channel 1=========================================
  if(last_channel_1 == 0 && PIND & B00000100 ){         //Input 2 changed from 0 to 1 RISING
    last_channel_1 = 1;                                 //Remember current input state
    last_armed_status = millis();
  }
  else if(last_channel_1 == 1 && !(PIND & B00000100)){  //Input 2 from 1 to 0 FALLING
    last_channel_1 = 0;                                 //Remember current input state
  }
  //Channel 2=========================================
  if(last_channel_2 == 0 && PIND & B00001000 ){         //Input 3 changed from 0 to 1
    last_channel_2 = 1;                                 //Remember current input state
    last_gps_status = millis();
  }
  else if(last_channel_2 == 1 && !(PIND & B00001000)){  //Input 3 changed from 1 to 0
    last_channel_2 = 0;                                 //Remember current input state
  }
  //Channel 3=========================================
  if(last_channel_3 == 0 && PIND & B00010000 ){         //Input 4 changed from 0 to 1
    last_channel_3 = 1;                                 //Remember current input state
    last_warn_status = millis();
  }
  else if(last_channel_3 == 1 && !(PIND & B00010000)){  //Input 4 changed from 1 to 0
    last_channel_3 = 0;                                 //Remember current input state
  }
  //Channel 4=========================================
  if(last_channel_4 == 0 && PIND & B00100000 ){         //Input 4 changed from 0 to 1
    last_channel_4 = 1;                                 //Remember current input state
    timer_4 = micros();                                 //Set timer_4 to micros()
  }
  else if(last_channel_4 == 1 && !(PIND & B00100000)){  //Input 4 changed from 1 to 0
    last_channel_4 = 0;                                 //Remember current input state
    receiver_input_channel_4 = micros() - timer_4;      //Channel 4 is micros() - timer_4
  }
}
