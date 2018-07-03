#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define pixels1PIN 0
#define pixels1Count 22
#define pixels2PIN 1
#define pixels2Count 22
#define buttonPin 3

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(pixels1Count, pixels1PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(pixels1Count, pixels2PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

uint8_t mode = 0;
uint8_t num_modes = 1;

uint8_t wheelIdx;

unsigned long LEDDeltaT = 200;
unsigned long last_LED_time = 0;

unsigned long ButtonDeltaT = 200;
unsigned long last_button_time = 0;

uint8_t currentPixelIdx = 0;

uint8_t theaterChaseMode = 0;
uint8_t theaterChaseIdx = 0;

uint32_t currentColor = 0;

uint32_t brightness = 64;
int32_t brightness_delta = 1;


void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  // End of trinket special code

  //while (!Serial);
  /*
  Serial.begin(9600);
  Serial.println("Crown");
  */
  pinMode(buttonPin, INPUT_PULLUP);

  strip1.setBrightness(brightness);
  strip2.setBrightness(brightness);

  strip1.begin();
  strip2.begin();

  strip1.show(); // Initialize all pixels to 'off'
  strip2.show(); // Initialize all pixels to 'off'
}

void loop() {
  /*
  if (millis() - last_button_time > ButtonDeltaT) {
    if (digitalRead(buttonPin) == LOW)
    { 
      ++mode;
      mode %= num_modes;
      currentPixelIdx = 0;
      theaterChaseIdx = 0;
      wipePixels();
    }
    last_button_time = millis();
  }
  */

  if (millis() - last_LED_time > LEDDeltaT) {
    LEDLoop();
    last_LED_time = millis();
  }
}

void LEDLoop() {
  ++wheelIdx;
  wheelIdx % 256;

  LEDDeltaT = 10;
  currentColor = strip1.Color(255, 0, 0);
  //colorWipe(currentColor);
  rainbowCycle();

  /*
  brightness += brightness_delta;
  if ((brightness < 10) || (brightness > 100)) {
    brightness_delta *= -1;
  }
  strip1.setBrightness(brightness);
  strip2.setBrightness(brightness);
  */
  
  /*
  if (mode == 0) {
    LEDDeltaT = 50;
    rainbowCycle();
  } else if (mode == 1) {
    LEDDeltaT = 50;
    theaterChaseRainbow();
  } else if (mode == 2) {
    LEDDeltaT = 50;
    currentColor = strip.Color(255, 0, 0);
    theaterChase(currentColor);
  } else if (mode == 3) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 255, 0);
    theaterChase(currentColor);
  } else if (mode == 4) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 0, 255);
    theaterChase(currentColor);
  } else if (mode == 5) {
    LEDDeltaT = 50;
    currentColor = strip.Color(255, 255, 255);
    theaterChase(currentColor);
  } else if (mode == 6) {
    LEDDeltaT = 50;
    currentColor = strip.Color(255, 0, 0);
    colorWipe(currentColor);
  } else if (mode == 7) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 255, 0);
    colorWipe(currentColor);
  } else if (mode == 8) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 0, 255);
    colorWipe(currentColor);
  } else if (mode == 9) {
    LEDDeltaT = 50;
    currentColor = strip.Color(255, 255, 255);
    colorWipe(currentColor);
  } else if (mode == 10) {
    LEDDeltaT = 50;
    currentColor = strip.Color(255, 0, 0);
    sparkle(currentColor, 0.334);
  } else if (mode == 11) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 255, 0);
    sparkle(currentColor, 0.334);
  } else if (mode == 12) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 0, 255);
    sparkle(currentColor, 0.334);
  } else if (mode == 13) {
    LEDDeltaT = 50;
    currentColor = strip.Color(255, 255, 255);
    sparkle(currentColor, 0.334);
  } else if (mode == 14) {
    LEDDeltaT = 50;
    currentColor = strip.Color(0, 0, 255);
    rainbowSparkle(currentColor, 0.334);
  }
  */

  /*
  if (mode == 0) {
    theaterChaseRainbow(50);
  } else if (mode == 1) {
    // Some example procedures showing how to display to the pixels:
    LEDDeltaT = 50;
    colorWipe(strip.Color(255, 0, 0)); // Red
  } else if (mode == 2) {
    LEDDeltaT = 50;
    colorWipe(strip.Color(0, 255, 0)); // Green
  } else if (mode == 3) {
    LEDDeltaT = 50;
    colorWipe(strip.Color(0, 0, 255)); // Blue
  } else if (mode == 4) {
    LEDDeltaT = 50;
    colorWipe(strip.Color(0, 0, 0, 255)); // White RGBW
  } else if (mode == 5) {
    // Send a theater pixel chase in...
    LEDDeltaT = 50;
    theaterChase(strip.Color(127, 0, 0)); // Red
  } else if (mode == 6) {
    LEDDeltaT = 50;
    theaterChase(strip.Color(0, 0, 127)); // Blue
  } else if (mode == 7) {
    LEDDeltaT = 30;
    rainbowCycle();
  } else if (mode == 8) {
    sparkle(strip.Color(255, 0, 0), 0.25);
    LEDDeltaT = 50;
  } else if (mode == 9) {
    LEDDeltaT = 10;
    rainbowSparkle();
  }
  */
}

void wipePixels() {
  for (uint8_t idx = 0; idx < strip1.numPixels(); ++idx) {
    strip1.setPixelColor(idx, 0);
    strip2.setPixelColor(idx, 0);
  }
  strip1.show();
  strip2.show();
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c) {
  if (currentPixelIdx == 0) {
    wipePixels();
  }
  strip1.setPixelColor(currentPixelIdx, c);
  strip2.setPixelColor(currentPixelIdx, c);
  strip1.show();
  strip2.show();
  ++currentPixelIdx;
  currentPixelIdx %= strip1.numPixels();
}


