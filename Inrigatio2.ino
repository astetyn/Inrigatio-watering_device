#include "avr/sleep.h"
#include "avr/power.h"
#include "avr/wdt.h"
#include "EEPROM.h"
#include <tinyNeoPixel_Static.h>

#define NUM_LEDS 3

typedef unsigned long seconds_t;

const int BUTTON_PIN = 4;
const int PUMP_PIN = 0;
const int LEDS_EN_PIN = 3;
const int LEDS_PIXEL_PIN = 2;

byte pixels[NUM_LEDS * 3];
tinyNeoPixel leds = tinyNeoPixel(NUM_LEDS, LEDS_PIXEL_PIN, NEO_GRB, pixels);

seconds_t press_start = 0;
seconds_t seconds_sleep_left = 0;
int wdt_timeout = 0;
int days_interval = 3;
int change_days_to = 0;
bool button_was_pressed = false;
int leds_last = -1;

void setup() {
  pinMode(LEDS_PIXEL_PIN, OUTPUT);
  pinMode(LEDS_EN_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(1, OUTPUT);

  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(LEDS_EN_PIN, HIGH);
  digitalWrite(1, LOW);
  enablePinChangeInterrupt();

  showAllLeds(10, 0, 0);
  delay(500);
  showAllLeds(0, 0, 0);
}


void loop() {

  if(isButtonPressed()) {

    if(change_days_to != leds_last) {
      switch(change_days_to) {
        case 0: showAllLeds(50, 50, 0); break;
        case 1: showAllLeds(0, 0, 0, 0, 0, 0, 0, 255, 0); break;
        case 2: showAllLeds(0, 0, 0, 0, 255, 0, 0, 255, 0); break;
        case 3: showAllLeds(0, 255, 0, 0, 255, 0, 0, 255, 0); break;
      }
      leds_last = change_days_to;
    }

    if(press_start == 0) {
      press_start = millis();
    }
    
    if(press_start + 5000 < millis()) { // instant water
      water(10000, true);
      press_start = 0;
      change_days_to = 0;
    }else if(press_start + 3000 < millis()) { // holding for 3 secs - 3 days
      change_days_to = 3;
    }else if(press_start + 2000 < millis()) { // holding for 2 secs - 2 days
      change_days_to = 2;
    }else if(press_start + 1000 < millis()) { // holding for 1 sec - set 1 day
      change_days_to = 1;
    }else {
      change_days_to = 0;
    }
  }else {

    // this code is executed every time user un-push button, controller will enter sleep mode
    if(change_days_to != 0) {
      days_interval = change_days_to;
      change_days_to = 0;
      seconds_sleep_left = 0;

      showAllLeds(0, 0, 0);
      delay(500);
      showAllLeds(100, 100, 100);
      delay(50);
      showAllLeds(0, 0, 0);
      delay(100);
      showAllLeds(100, 100, 100);
      delay(50);
    }

    if(seconds_sleep_left > 0) {
      seconds_sleep_left = enterSleep(seconds_sleep_left); // sleeping was interrupted by button, return back to sleeping
    }else {
      seconds_sleep_left = enterSleep(60 * 60 * 24 * days_interval); // start new sleep, good night zzZzzZz
    }

    if(seconds_sleep_left == 0) {
      water(10000, true); // sleeping is over! water the plant
    }
    press_start = 0;
    leds_last = -1;
  }
}


void enablePinChangeInterrupt() {
  cli();
  PCMSK |= (1 << digitalPinToPCMSKbit(BUTTON_PIN)); // Pin Change Enable
  // equivalent to: PCMSK |= (1 <<PCINT3);
  GIMSK |= (1 << digitalPinToPCICRbit(BUTTON_PIN)); // PCIE Pin Change Interrupt Enable
  // equivalent to: GIMSK |= (1 << PCIE);
  sei();
}


ISR(PCINT0_vect) {
  MCUCR &= ~(1<<SE);      //Disabling sleep mode inside interrupt routine
  button_was_pressed = true;
}

ISR(WDT_vect) {
  WDTCR |= bit ( WDIE ); // set WDIE
}


// returns value v mV
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(1); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert 
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


void water(seconds_t duration, bool with_leds) {
  digitalWrite(PUMP_PIN, HIGH);

  seconds_t start_t = millis();
  float counter = 0;
  
  while(start_t + duration > millis()) {
    if(with_leds) {
      uint8_t l1 = max(0, sin(counter)) * 100;
      uint8_t l2 = max(0, sin(counter + TWO_PI / 3)) * 100;
      uint8_t l3 = max(0, sin(counter + TWO_PI * 2 / 3)) * 100;
      
      showAllLeds(l1, l1, l1, l2, l2, l2, l3, l3, l3);
      counter += TWO_PI / 50;
    }
    delay(10);
  }
  showAllLeds(0, 0, 0);
  digitalWrite(PUMP_PIN, LOW);
}


bool isButtonPressed() {
  return digitalRead(BUTTON_PIN) == HIGH;
}

void showAllLeds(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, uint8_t r3, uint8_t g3, uint8_t b3) {
  leds.setPixelColor(0, leds.Color(r1, g1, b1));
  leds.setPixelColor(1, leds.Color(r2, g2, b2));
  leds.setPixelColor(2, leds.Color(r3, g3, b3));
  leds.show();
  delayMicroseconds(500);
}


void showLed(int led_num, uint8_t r, uint8_t g, uint8_t b) {
  leds.setPixelColor(led_num, leds.Color(r, g, b));
  leds.show();
  delayMicroseconds(500);
}


void showAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, leds.Color(r, g, b));
  }
  leds.show();
  delayMicroseconds(500);
}

void enableWDT(int seconds_macro) {
  if(seconds_macro == WDTO_8S) {
    wdt_timeout = 8;
  }else if(seconds_macro == WDTO_1S) {
    wdt_timeout = 1;
  }

  wdt_enable(seconds_macro);
  WDTCR |= bit ( WDIE ); // set WDIE - with this only interrupt will be called instead of reset
}


// returns how many seconds left to finish sleep
seconds_t enterSleep(seconds_t seconds) {
  digitalWrite(PUMP_PIN, LOW);
  showAllLeds(0, 0, 0);

  /*
  WARNING!!
  There is 0.91 secs extra for every 8 seconds (tested on 5V and 3.3V)
  so calculations must take this into consideration
  it is about 11 percent?
  */
  const float wdt_miss_time_factor = 0.104;

  seconds_t remaining_secs_sleep = (seconds_t) (seconds * (1 - wdt_miss_time_factor));

  // sleep bits
  MCUCR |= (1 << SM1);    // enabling sleep mode and powerdown sleep mode
  MCUCR |= (1 << SE);     //Enabling sleep enable bit

  ADCSRA &= ~(1<<ADEN); // Disable ADC

  enableWDT(WDTO_8S);

  while(remaining_secs_sleep > 0) {

    if(button_was_pressed) {
      button_was_pressed = false;
      break;
    }

    if(remaining_secs_sleep < wdt_timeout) {
      enableWDT(WDTO_1S);
    }

    remaining_secs_sleep -= wdt_timeout;

    __asm__ __volatile__ ( "sleep" "\n\t" :: ); //Sleep instruction to put controller to sleep
  }

  // return everything to normal
  MCUCR &= ~(1<<SE); // disable sleep enable bit

  wdt_disable();
  return (seconds_t) (remaining_secs_sleep / (1 - wdt_miss_time_factor));
}
