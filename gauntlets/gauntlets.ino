#include <Time.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_NeoPixel.h>

#define BRIGHTNESS 255
#define NUM_LAST_ACCEL_DATAS 10
#define USE_LSM 1

Adafruit_NeoPixel heartbeatLED = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strips[4];

#ifdef USE_LSM
// i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();
#endif

int numChannels = 4;
int numPixelsPerChannel = 10;
int numTotalPixels = numChannels*numPixelsPerChannel;

unsigned long current_time = 0;

int LEDPins[4] = {10, 9, 12, 6};

unsigned long heartbeatLEDDeltaT = 100;
unsigned long last_heartbeat_time = 0;

unsigned long LEDDeltaT = 200;
unsigned long last_LED_time = 0;

unsigned long LSMDeltaT = 10;
unsigned long last_LSM_time = 0;

unsigned long SummaryDeltaT = 500;
unsigned long last_summary_time = 0;

unsigned long accelCountDeltaT = 75;
unsigned long last_accelCount_time = 0.;

int minAccelXCount = 3;
int accelXCount = 0;
int minAccelYCount = 3;
int accelYCount = 0;
int minAccelZCount = 3;
int accelZCount = 0;

#ifdef USE_LSM
int lastAccelDatasIdx = NUM_LAST_ACCEL_DATAS;
Adafruit_LSM9DS0::lsm9ds0Vector_t accelData;
Adafruit_LSM9DS0::lsm9ds0Vector_t lastAccelDatas[NUM_LAST_ACCEL_DATAS];
float accelAvgX = 0.0;
float accelAvgY = 0.0;
float accelAvgZ = 0.0;
float accelAvgMag = 0.0;
#endif

int currentLEDIdx = 0;

boolean heartbeatLEDOn = false;
int heartbeatJ = 0;

int pattern = 0;
int num_patterns = 3;

int current_color_idx = 0;
int current_color = 0;

#ifdef USE_LSM
void calcAccelAvg() {
  accelAvgX = accelAvgY = accelAvgZ = 0.;
  
  for (int idx = 0; idx < NUM_LAST_ACCEL_DATAS; ++idx) {
    accelAvgX += lastAccelDatas[idx].x;
    accelAvgY += lastAccelDatas[idx].y;
    accelAvgZ += lastAccelDatas[idx].z;
  }

  accelAvgX /= (float)NUM_LAST_ACCEL_DATAS;
  accelAvgY /= (float)NUM_LAST_ACCEL_DATAS;
  accelAvgZ /= (float)NUM_LAST_ACCEL_DATAS;

  accelAvgMag = sqrt(accelAvgX*accelAvgX + accelAvgY*accelAvgY + accelAvgZ*accelAvgZ);
}

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
#endif

void setup() {
  heartbeatLED.setBrightness(10);
  heartbeatLED.begin();
  heartbeatLED.setPixelColor(0, heartbeatLED.Color(255, 255, 255));
  heartbeatLED.show();

  current_color = heartbeatLED.Color(255, 255, 255);

  //while (!Serial); // flora & leonardo
  
  Serial.begin(9600);
  Serial.println("LSM raw read demo");

  heartbeatLED.setPixelColor(0, heartbeatLED.Color(255, 0, 0));
  heartbeatLED.show();

#ifdef USE_LSM
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  

  lsm.read();

  accelData.x = lsm.accelData.x;
  accelData.y = lsm.accelData.y;
  accelData.z = lsm.accelData.z;

  heartbeatLED.setPixelColor(0, heartbeatLED.Color(0, 255, 0));
  heartbeatLED.show();
  
  for (uint8_t idx = 0; idx < NUM_LAST_ACCEL_DATAS; ++idx) {
    lastAccelDatas[idx].x = accelData.x;
    lastAccelDatas[idx].y = accelData.y;
    lastAccelDatas[idx].z = accelData.z;
  }
#endif

  heartbeatLED.setPixelColor(0, heartbeatLED.Color(0, 0, 255));
  heartbeatLED.show();
  
  for (uint16_t cIdx = 0; cIdx < numChannels; ++cIdx) {
    Serial.print("Initialized strip "); Serial.print(cIdx); Serial.print(" on pin "); Serial.print(LEDPins[cIdx]); Serial.println(" ");
    strips[cIdx] = Adafruit_NeoPixel(numPixelsPerChannel, LEDPins[cIdx], NEO_GRB + NEO_KHZ800);
    strips[cIdx].setBrightness(BRIGHTNESS);
    strips[cIdx].begin();
    strips[cIdx].show(); // .Initialize all pixels to 'off'
  }

  allOn(heartbeatLED.Color(128, 255, 0));

  heartbeatLED.setPixelColor(0, heartbeatLED.Color(0, 0, 255));
  heartbeatLED.show();  
}

unsigned long last_time = 0;
void loop() {
  heartbeat();
  
  current_time = millis();
  /*
  if (current_time - last_time > 50) {
    Serial.print("Time: "); Serial.print(current_time); Serial.println(" ");
    last_time = current_time;
  }
  */
  if (current_time - last_summary_time > SummaryDeltaT) {
    SummaryLoop();
    last_summary_time = millis();
  }

  current_time = millis();
  if (current_time - last_LED_time > LEDDeltaT) {
    LEDLoop();
    last_LED_time = millis();
  }

#ifdef USE_LSM
  current_time = millis();
  if (current_time - last_LSM_time > LSMDeltaT) {

    //unsigned long startT = millis();

    LSMLoop();
    last_LSM_time = millis();
    
    //Serial.print("LSMloop begin: "); Serial.print((unsigned long)startT); Serial.println(" ");
    //Serial.print("LSMloop end: "); Serial.print((unsigned long)last_LSM_time); Serial.println(" ");
    //Serial.print("LSMloop: "); Serial.println((unsigned long)(last_LSM_time - startT)); Serial.println(" ");
  }
#endif
}

void SummaryLoop()
{
  Serial.print("Time: "); Serial.print((unsigned long)millis());                     Serial.println(" ");
#ifdef USE_LSM
  Serial.print("Accel X: "); Serial.print((float)accelAvgX); Serial.print(" ");
  Serial.print("Y: "); Serial.print((float)accelAvgY);       Serial.print(" ");
  Serial.print("Z: "); Serial.print((float)accelAvgZ);     Serial.print(" ");
  Serial.print("Mag: "); Serial.print((float)accelAvgMag);     Serial.println(" ");
#endif
}

#ifdef USE_LSM
void LSMLoop() 
{
  //Serial.print("Begin LSM Loop... ");

  ++lastAccelDatasIdx;
  if (lastAccelDatasIdx > NUM_LAST_ACCEL_DATAS) {
    lastAccelDatasIdx = 0;
  }

  lastAccelDatas[lastAccelDatasIdx].x = accelData.x;
  lastAccelDatas[lastAccelDatasIdx].y = accelData.y;
  lastAccelDatas[lastAccelDatasIdx].z = accelData.z;

  calcAccelAvg();

  lsm.read();
  /*
  Serial.print("Accel X: "); Serial.print((int)lsm.accelData.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.accelData.y);       Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.accelData.z);     Serial.print(" ");
  Serial.print("Mag X: "); Serial.print((int)lsm.magData.x);     Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.magData.y);         Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.magData.z);       Serial.print(" ");
  Serial.print("Gyro X: "); Serial.print((int)lsm.gyroData.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)lsm.gyroData.y);        Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)lsm.gyroData.z);      Serial.print(" ");
  Serial.print("Temp: "); Serial.println((int)lsm.temperature);    Serial.println(" ");
  */

  accelData.x = lsm.accelData.x;
  accelData.y = lsm.accelData.y;
  accelData.z = lsm.accelData.z; 

  if (millis() - last_accelCount_time > accelCountDeltaT) {
    float absdiff = abs(accelData.x - accelAvgX);
    if (absdiff > 10000) {
      ++accelXCount;
      Serial.print("ACCELEROMETER UPDATE X: "); Serial.println(absdiff);
    }
    absdiff = abs(accelData.y - accelAvgY);
    if (absdiff > 10000) {
      ++accelYCount;
      Serial.println("Y UPDATE FOR THE ACCELEROMETER");
    }
    absdiff = abs(accelData.z - accelAvgZ);
    if (absdiff > 10000) {
      ++accelZCount;
      Serial.println("Z UPDATE FOR THE ACCELEROMETER");
    }
    /*
    if (accelXCount > minAccelXCount) {
      LEDDeltaT += 50;
      if (LEDDeltaT > 300) {
        LEDDeltaT = 50;
      }
      accelXCount = 0;
      last_accelCount_time = millis();
    }
    */
    if (accelYCount > minAccelYCount) {
      ++current_color_idx;
      current_color_idx %= 5;

      if (current_color_idx == 0) {
        current_color = heartbeatLED.Color(255, 0, 0);
      } else if (current_color_idx == 1) {
        current_color = heartbeatLED.Color(0, 255, 0);
      } else if (current_color_idx == 2) {
        current_color = heartbeatLED.Color(0, 0, 255);
      } else if (current_color_idx == 3) {
        current_color = heartbeatLED.Color(255, 255, 255);
      } else if (current_color_idx == 4) {
        current_color = Wheel(heartbeatJ & 255);
      }
      accelYCount = 0;
      last_accelCount_time = millis();
    }
    
    if (accelZCount > minAccelZCount) {
      ++pattern;
      pattern %= num_patterns;
      accelZCount = 0;
      last_accelCount_time = millis();
    }
  }
}
#endif

void LEDLoop() {
  //Serial.print("Begin LED Loop... ");

  // Some example procedures showing how to display to the pixels:
  //colorWipe(strips[0].Color(255, 0, 0), 500); // Red
  //colorWipe(strips[1].Color(0,255, 0), 500); // Green
  
  //colorWipe(strips[0].Color(255, 0, 0), 500); // Red
  //colorWipe(strips[0].Color(127, 255, 0), 500);
  //colorWipe(strips[0].Color(0, 255, 0), 500); // Green
  //colorWipe(strips[0].Color(127, 0, 255), 500);
  //colorWipe(strips[0].Color(0, 0, 255), 500); // Blue
  //colorWipe(strips[0].Color(127, 0, 255), 500);

  if (pattern == 0) {
    sparkle(current_color, 0.5);
  }
  //sparkle(strips[0].Color(127, 255, 0), 0.8);
  //sparkle(strips[0].Color(0, 255, 0), 0.8); // Green
  //sparkle(strips[0].Color(127, 0, 255), 0.8);
  //sparkle(strips[0].Color(0, 0, 255), 0.8); // Blue
  //sparkle(strips[0].Color(127, 0, 255), 0.8);

  //rainbowCycleSerial(3);
  //rainbowCycleParallel(3);
  // rainbowSparkle(3);

  else if (pattern == 1) {
    stepChaseParallel(current_color);
  }
  
  else if (pattern == 2) {
    stepChaseSerial(current_color);
  }
  //stepChaseSemiParallel(heartbeatLED.Color(255, 255, 255));
  //stepChaseSerial(heartbeatLED.Color(255, 255, 255));
  //allOn(heartbeatLED.Color(255, 0, 0));
  //allOn(heartbeatLED.Color(0, 255, 0));
  //allOn(heartbeatLED.Color(0, 0, 255));
  //int n = 64;
  //allOn(heartbeatLED.Color(n, n, n));
  else if (pattern == 3) {
    allOn(current_color);
  }

  //Serial.println("End LED Loop");
}

void heartbeat() {
  if (millis() - last_heartbeat_time > heartbeatLEDDeltaT) {
    //Serial.print("Heartbeat: "); Serial.print(heartbeatLEDOn); Serial.print(" "); Serial.print(heartbeatJ); Serial.println(" ");
    if (heartbeatLEDOn == false) {
      heartbeatLED.setPixelColor(0, Wheel(heartbeatJ & 255));
      heartbeatJ += 8;
      heartbeatJ %= 256;
      heartbeatLEDOn = true;
    } else {
      heartbeatLED.setPixelColor(0, 0);
      heartbeatLEDOn = false;
    }
    heartbeatLED.show();
    last_heartbeat_time = millis();
  }
}

void allOn(uint32_t c) {
  for (uint16_t cIdx = 0; cIdx < numChannels; ++cIdx) {
    for (uint16_t lIdx = 0; lIdx < numPixelsPerChannel; ++lIdx) {
        strips[cIdx].setPixelColor(lIdx, c);
    }
    strips[cIdx].show();
  }
}

void stepChaseSerial(uint32_t c) {
  if (currentLEDIdx % numTotalPixels == 0) {
    currentLEDIdx = 0;
  }
  for (uint16_t cIdx = 0; cIdx < numChannels; ++cIdx) {
    for (uint16_t lIdx = 0; lIdx < numPixelsPerChannel; ++lIdx) {
      if (cIdx*numPixelsPerChannel + lIdx == currentLEDIdx) {
        strips[cIdx].setPixelColor(lIdx, c);
      } else {
        strips[cIdx].setPixelColor(lIdx, 0);
      }
    }
    strips[cIdx].show();
  }

  ++currentLEDIdx;
}

void stepChaseSemiParallel(uint32_t c) {
  if (currentLEDIdx % (numTotalPixels/2) == 0) {
    currentLEDIdx = 0;
  }
  for (uint16_t cIdx = 0; cIdx < numChannels/2; ++cIdx) {
    for (uint16_t lIdx = 0; lIdx < numPixelsPerChannel; ++lIdx) {
      if (cIdx*numPixelsPerChannel + lIdx == currentLEDIdx) {
        strips[cIdx].setPixelColor(lIdx, c);
        strips[cIdx+2].setPixelColor(lIdx, c);
      } else {
        strips[cIdx].setPixelColor(lIdx, 0);
        strips[cIdx+2].setPixelColor(lIdx, 0);
      }
    }
    strips[cIdx].show();
    strips[cIdx+2].show();
  }

  ++currentLEDIdx;
}

void stepChaseParallel(uint32_t c) {
  if (currentLEDIdx % numPixelsPerChannel == 0) {
    currentLEDIdx = 0;
  }
  for (uint16_t lIdx = 0; lIdx < numPixelsPerChannel; ++lIdx) {
    if (lIdx == currentLEDIdx) {
      strips[0].setPixelColor(lIdx, c);
      strips[1].setPixelColor(lIdx, c);
      strips[2].setPixelColor(lIdx, c);
      strips[3].setPixelColor(lIdx, c);
    } else {
      strips[0].setPixelColor(lIdx, 0);
      strips[1].setPixelColor(lIdx, 0);
      strips[2].setPixelColor(lIdx, 0);
      strips[3].setPixelColor(lIdx, 0);
    }
    strips[0].show();
    strips[1].show();
    strips[2].show();
    strips[3].show();
  }
  ++currentLEDIdx;
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strips[0].numPixels(); i++) {
      strips[0].setPixelColor(i, c);
      strips[0].show();
      strips[1].setPixelColor(i, c);
      strips[1].show();
      delay(wait);
  }
}

// randomly light a pixel
void sparkle(uint32_t c, float prob) {
  for (uint16_t cIdx=0; cIdx < numChannels; ++cIdx) {
    for(uint16_t pIdx=0; pIdx<strips[0].numPixels(); pIdx++) {
      float randNum = random(numTotalPixels) / (float)numTotalPixels;
      if (randNum < prob) {
        strips[cIdx].setPixelColor(pIdx, c);
        strips[cIdx].show();
      } else {
        strips[cIdx].setPixelColor(pIdx, 0);
      }
    }
    strips[cIdx].show();
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycleSerial(uint8_t wait) {
  uint16_t i, j, stripIdx, pixelIdx;

  for(j=0; j<256; j++) {

    for(i=0; i< numTotalPixels; i+=5) {
      stripIdx = i / numPixelsPerChannel;
      pixelIdx = i % numPixelsPerChannel;
      strips[stripIdx].setPixelColor(i, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
      strips[stripIdx].setPixelColor(i+1, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
      strips[stripIdx].setPixelColor(i+2, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
      strips[stripIdx].setPixelColor(i+3, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
      strips[stripIdx].setPixelColor(i+4, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    }

    for(stripIdx=0; stripIdx < numChannels; ++stripIdx) {
      strips[stripIdx].show();
    }
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycleParallel(uint8_t wait) {
  uint16_t i, j, stripIdx, pixelIdx;

  for(j=0; j<256*1; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< numTotalPixels; i++) {
      stripIdx = i / numPixelsPerChannel;
      pixelIdx = i % numPixelsPerChannel;
      
      strips[stripIdx].setPixelColor(pixelIdx, Wheel(((pixelIdx * 256 / numTotalPixels) + j) & 255));
    }

    //for(i=0; i< strips[0].numPixels(); i+=5) {
    //  strips[0].setPixelColor(i, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+1, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+2, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+3, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+4, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //}
    
    for(stripIdx=0; stripIdx < numChannels; ++stripIdx) {
      strips[stripIdx].show();
    }
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowSparkle(uint8_t n) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strips[0].numPixels(); i++) {
      //strips[0].setPixelColor(i, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
      sparkle(Wheel(((i * 256 / strips[0].numPixels()) + j) & 255), n);
    }

    //for(i=0; i< strips[0].numPixels(); i+=5) {
    //  strips[0].setPixelColor(i, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+1, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+2, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+3, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //  strips[0].setPixelColor(i+4, Wheel(((i * 256 / strips[0].numPixels()) + j) & 255));
    //}

  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strips[0].Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strips[0].Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strips[0].Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
