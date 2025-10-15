// Arduino LED ring (NeoPixel) with 8 digital inputs
#include <Adafruit_NeoPixel.h>

#define LED_PIN 6
#define LED_COUNT 60

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const uint8_t IN_LEFT_RED   = 2;
const uint8_t IN_LEFT_BLUE  = 3;
const uint8_t IN_FRONT_RED  = 4;
const uint8_t IN_FRONT_BLUE = 5;
const uint8_t IN_RIGHT_RED  = 7;
const uint8_t IN_RIGHT_BLUE = 8;
const uint8_t IN_BACK_RED   = 9;
const uint8_t IN_BACK_BLUE  = 10;

uint32_t C_RED, C_BLUE, C_OFF;

void setRange(uint16_t a, uint16_t b, uint32_t c) {
  if (a < 1) a = 1; if (b > LED_COUNT) b = LED_COUNT;
  for (uint16_t i = a-1; i < b; i++) strip.setPixelColor(i, c);
}

void applySegment(uint8_t r_pin, uint8_t b_pin, uint16_t a, uint16_t b) {
  int r = digitalRead(r_pin);
  int bl = digitalRead(b_pin);
  if (r && !bl) setRange(a,b,C_RED);
  else if (!r && bl) setRange(a,b,C_BLUE);
  else setRange(a,b,C_OFF);
}

void setup() {
  pinMode(IN_LEFT_RED,   INPUT);
  pinMode(IN_LEFT_BLUE,  INPUT);
  pinMode(IN_FRONT_RED,  INPUT);
  pinMode(IN_FRONT_BLUE, INPUT);
  pinMode(IN_RIGHT_RED,  INPUT);
  pinMode(IN_RIGHT_BLUE, INPUT);
  pinMode(IN_BACK_RED,   INPUT);
  pinMode(IN_BACK_BLUE,  INPUT);
  strip.begin();
  strip.show();
  C_RED = strip.Color(255,0,0);
  C_BLUE = strip.Color(0,0,255);
  C_OFF = strip.Color(0,0,0);
}

void loop() {
  applySegment(IN_LEFT_RED,  IN_LEFT_BLUE,  1, 15);
  applySegment(IN_FRONT_RED, IN_FRONT_BLUE, 16, 30);
  applySegment(IN_RIGHT_RED, IN_RIGHT_BLUE, 31, 45);
  applySegment(IN_BACK_RED,  IN_BACK_BLUE,  46, 60);
  strip.show();
  delay(10);
}
