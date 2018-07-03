#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define pixelsPIN 1
#define buttonPin 0

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(9, pixelsPIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

uint8_t mode = 0;
uint8_t num_modes = 15;

uint8_t wheelIdx;

unsigned long LEDDeltaT = 200;
unsigned long last_LED_time = 0;

unsigned long ButtonDeltaT = 200;
unsigned long last_button_time = 0;

uint8_t currentPixelIdx = 0;

uint8_t theaterChaseMode = 0;
uint8_t theaterChaseIdx = 0;

uint32_t currentColor = 0;

uint32_t brightness = 10;
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

  strip.setBrightness(brightness);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
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
  rainbowCycle();

  brightness += brightness_delta;
  if ((brightness < 10) || (brightness > 100)) {
    brightness_delta *= -1;
  }
  strip.setBrightness(brightness);
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

// randomly light pixels
void sparkle(uint32_t c, float prob) {
  for(uint16_t pIdx=0; pIdx<strip.numPixels(); pIdx++) {
    float randNum = random(strip.numPixels()) / (float)strip.numPixels();
    if (randNum < prob) {
      strip.setPixelColor(pIdx, c);
      strip.show();
    } else {
      strip.setPixelColor(pIdx, 0);
    }
  }
  strip.show();
}

// randomly light pixels
void rainbowSparkle(uint32_t c, float prob) {
  for(uint16_t pIdx=0; pIdx<strip.numPixels(); pIdx++) {
    float randNum = random(strip.numPixels()) / (float)strip.numPixels();
    if (randNum < prob) {
      strip.setPixelColor(pIdx, Wheel(((pIdx * 256 / strip.numPixels()) + wheelIdx) & 255));
      strip.show();
    } else {
      strip.setPixelColor(pIdx, 0);
    }
  }
  strip.show();
}

void wipePixels() {
  for (uint8_t idx = 0; idx < strip.numPixels(); ++idx) {
    strip.setPixelColor(idx, 0);
  }
  strip.show();
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c) {
  if (currentPixelIdx == 0) {
    wipePixels();
  }
  strip.setPixelColor(currentPixelIdx, c);
  strip.show();
  ++currentPixelIdx;
  currentPixelIdx %= strip.numPixels();
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle() {
  for(uint16_t i=0; i< strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + wheelIdx) & 255));
  }
  strip.show();
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c) {
    if (theaterChaseMode == 0) {
      theaterChaseOn(c);
      theaterChaseMode = 1;
    } else {
      theaterChaseOff();
      theaterChaseMode = 0;
      ++theaterChaseIdx;
    }
    theaterChaseIdx %= 3;
    strip.show();
}

//Theatre-style crawling lights.
void theaterChaseRainbow() {
    if (theaterChaseMode == 0) {
      theaterChaseRainbowOn();
      theaterChaseMode = 1;
    } else {
      theaterChaseOff();
      theaterChaseMode = 0;
      ++theaterChaseIdx;
    }
    theaterChaseIdx %= 3;
    strip.show();
}

void theaterChaseOn(uint32_t c) {
  for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
    strip.setPixelColor(i+theaterChaseIdx, c);    //turn every third pixel on
  }
}

void theaterChaseRainbowOn() {
  for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
    uint32_t c = Wheel( (i+wheelIdx) % 255);
    strip.setPixelColor(i+theaterChaseIdx, c);    //turn every third pixel on
  }
}void theaterChaseOff() {
    for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
      strip.setPixelColor(i+theaterChaseIdx, 0);        //turn every third pixel off
    }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
