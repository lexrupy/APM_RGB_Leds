
#include "FastLED.h"
#include "PinChangeInterrupt.h"
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

CRGB leds_status[1];
CRGB leds_front[NUM_LEDS];
CRGB leds_back[NUM_LEDS];

// Status: 0, 2, 4 for lights off, 1, 3 for lights on
// Pulses: flashing with two 20 ms pulses, 80 ms in between pulses and 1 Hz frequency
byte led_state = 0;

// Light modes: 1, normal mode. 0, off. 2, low battery.
#define NUM_MODOS 5
byte led_mode = MODE_ARMED_GPS;

CRGB led_front_on[NUM_MODOS][NUM_LEDS] = {
  // MODE_WARNING
  { CRGB::Red,   CRGB::Red },
  // MODE_DISARMED_NO_GPS
  { CRGB::Black, CRGB::Black },
  // MODE_DISARMED_GPS 
  { CRGB::Black,  CRGB::Black },
  // MODE_ARMED_NO_GPS
  { CRGB::Red,  CRGB::Red },
  // MODE_ARMED_GPS
  { CRGB::Red,  CRGB::Red }
};

CRGB led_front_on2[NUM_MODOS][NUM_LEDS] = {
    // MODE_WARNING
  { CRGB::Red,   CRGB::Red },
  // MODE_DISARMED_NO_GPS
  { CRGB::Yellow, CRGB::Yellow},
  // MODE_DISARMED_GPS 
  { CRGB::Yellow,  CRGB::Yellow },
  // MODE_ARMED_NO_GPS
  { CRGB::Red,  CRGB::Red },
  // MODE_ARMED_GPS
  { CRGB::Red,  CRGB::Red }
};

CRGB led_front_off[NUM_MODOS][NUM_LEDS] = {
    // MODE_WARNING
  { CRGB::Black, CRGB::Black  },
  // MODE_DISARMED_NO_GPS
  { CRGB::Black, CRGB::Black },
  // MODE_DISARMED_GPS 
  { CRGB::Black, CRGB::Black  },
  // MODE_ARMED_NO_GPS
  { CRGB::Red,  CRGB::Red },
  // MODE_ARMED_GPS
  { CRGB::Red,  CRGB::Red }
};



CRGB led_back_on[NUM_MODOS][NUM_LEDS] = {
  // MODE_WARNING
  { CRGB::Red,   CRGB::Red },
  // MODE_DISARMED_NO_GPS
  { CRGB::Cyan, CRGB::Cyan },
  // MODE_DISARMED_GPS 
  { CRGB::Green,  CRGB::Green },
  // MODE_ARMED_NO_GPS
  { CRGB::Cyan,  CRGB::Cyan },
  // MODE_ARMED_GPS
  { CRGB::Green,  CRGB::Green }
};

CRGB led_back_on2[NUM_MODOS][NUM_LEDS] = {
    // MODE_WARNING
  { CRGB::Red,   CRGB::Red },
  // MODE_DISARMED_NO_GPS
  { CRGB::Black, CRGB::Black },
  // MODE_DISARMED_GPS 
  { CRGB::Black,  CRGB::Black },
  //MODE_ARMED_NO_GPS
  { CRGB::Black,  CRGB::Black },
  // MODE_ARMED_GPS
  { CRGB::White,  CRGB::White }
};

CRGB led_back_off[NUM_MODOS][NUM_LEDS] = {
    // MODE_WARNING
  { CRGB::Black, CRGB::Black },
  // MODE_DISARMED_NO_GPS
  { CRGB::Black, CRGB::Black },
  // MODE_DISARMED_GPS 
  { CRGB::Black, CRGB::Black  },
  //MODE_ARMED_NO_GPS
  { CRGB::Black,  CRGB::Black },
  // MODE_ARMED_GPS
  { CRGB::Black,  CRGB::Black }
};


CRGB led_status_on[NUM_MODOS] = {
  // MODE_WARNING
  { CRGB::Red },
  // MODE_DISARMED_NO_GPS
  { CRGB::Black },
  // MODE_DISARMED_GPS 
  { CRGB::Black },
  // MODE_ARMED_NO_GPS
  { CRGB::Black},
  //MODE_ARMED_GPS
  { CRGB::Black}
};

CRGB led_status_on2[NUM_MODOS] = {
    // MODE_WARNING
  { CRGB::Red },
  // MODE_DISARMED_NO_GPS
  { CRGB::Blue },
  // MODE_DISARMED_GPS 
  { CRGB::Green },
  //MODE_ARMED_NO_GPS
  { CRGB::Black},
  //MODE_ARMED_GPS
  { CRGB::Black}
};

CRGB led_status_off[NUM_MODOS] = {
    // MODE_WARNING
  { CRGB::Black },
  // MODE_DISARMED_NO_GPS
  { CRGB::Black },
  // MODE_DISARMED_GPS 
  { CRGB::Black },
  //MODE_ARMED_NO_GPS
  { CRGB::Black},
  //MODE_ARMED_GPS
  { CRGB::Black}
};

// Lights flashing adjustment
unsigned long previousMillis = 0;     // will store last time LED was updated
unsigned long next_interval = 0;      // next interval
const long tiempo_on = 20;
const long tiempo_off = 80;
const long tiempo_descanso = 880;
int test_led_tipo = 4;


void setup() {
  interrupts();
  
  //delay(5000); // sanity delay
  FastLED.addLeds<WS2811, DATA_PIN_STATUS, GRB>(leds_status, 1);
  FastLED.addLeds<WS2811, DATA_PIN_FRONT, GRB>(leds_front, NUM_LEDS);
  FastLED.addLeds<WS2811, DATA_PIN_BACK, GRB>(leds_back, NUM_LEDS);
  FastLED.setBrightness( BRIGHTNESS );


  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), checkArmStatus, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), checkGPSStatus, FALLING);
  attachPCINT(digitalPinToPCINT(3), checkWarnStatus, FALLING);

  Serial.begin(57600);
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
    led_mode = MODE_WARNING;
  } else {
    if (armed) {
      if (gpsfix) { led_mode = MODE_ARMED_GPS; } else led_mode = MODE_ARMED_NO_GPS;
    } else {
      if (gpsfix) { led_mode = MODE_DISARMED_GPS; } else led_mode = MODE_DISARMED_NO_GPS;
    }
  }
}

void checkArmStatus() {
  last_armed_status = millis();
}

void checkGPSStatus() {
  last_gps_status = millis();
}

void checkWarnStatus() {
  last_warn_status = millis();
}

void loop() {
   

   String c;
   while (Serial.available()) {
     c = Serial.readString();
   }
   if (c =="1") {
    led_mode = 1;
   } else if (c == "2") {
    led_mode = 2;
   } else if (c == "0") {
    led_mode = 0;
   } else if (c == "3") {
    led_mode = 3;
   } else if (c == "4") {
    led_mode = 4;
   } else if (c == "f") {
    front_lights_disabled = !front_lights_disabled;
   }

  
  
  // Lights management
  // Light pulses: 2 quick flashes per second
  unsigned long currentMillis = millis();
  int i=0;
  updateState(currentMillis);
  // Normal mode, lights on.
  if (currentMillis - previousMillis >= next_interval) {
    // Keep time last mode changed
    previousMillis = currentMillis;

    // Toggle lights depending on status
    switch (led_state){
      case 0:
        {
          next_interval=tiempo_on;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_on[led_mode][i];
            leds_back[i]=led_back_on[led_mode][i];
          }
          leds_status[0]= led_status_on[led_mode];
          break;
        }
        
      case 1:
        {
          next_interval=tiempo_off;
          for(i=0;i<NUM_LEDS;i++){
            leds_front[i]=led_front_off[led_mode][i];
            leds_back[i]=led_back_off[led_mode][i];
          }
          leds_status[0] = led_status_off[led_mode];
          break;
        }
        
      case 2:
        {
          next_interval=tiempo_on;
          for(i=0;i<NUM_LEDS;i++){
            if (front_lights_disabled and not (led_mode==MODE_WARNING)) {
              leds_front[i]=CRGB::Black;
            } else leds_front[i]=led_front_on2[led_mode][i];
            leds_back[i]=led_back_on2[led_mode][i];
          }
          leds_status[0] = led_status_on2[led_mode];
          break;
        }

      case 3:
        {
          next_interval=tiempo_descanso;
          for(i=0;i<NUM_LEDS;i++){
            leds_front[i]=led_front_off[led_mode][i];
            leds_back[i]=led_back_off[led_mode][i];
          }
          leds_status[0] = led_status_off[led_mode];
          break;
        }
        
    }
    // Show leds
    FastLED.show();
    
    // Cycle status from 0 to 3
    led_state++;
    if(led_state >= 4) led_state=0;

  }
          
}
