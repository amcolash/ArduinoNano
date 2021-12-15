#include <Adafruit_NeoPixel.h>
#include "Adafruit_WS2801.h"
#include "ClickButton.h"
#include <Wire.h>

struct Section {
  uint8_t DATA_PIN;
  uint8_t CLOCK_PIN;
  uint8_t NUM_PIXELS;

  int randomCount;
  int count;
  int skipTime;

  float frequency;
  
  uint8_t color[3];
  
  Adafruit_WS2801 *strip;
  Adafruit_NeoPixel *strip2812;
  uint32_t *colors;
};

enum Mode {
  Standard,
  RainbowPastel,
  Rainbow,

  ModeLength
};

#define DATA_A_PIN  2
#define CLOCK_A_PIN 3

#define DATA_B_PIN  4
#define CLOCK_B_PIN 5

#define DATA_C_PIN  6
#define CLOCK_C_PIN 7

#define DATA_D_PIN  8

#define BUTTON_PIN 12

#define NUM_PIXELS_A 12
#define NUM_PIXELS_B 8
#define NUM_PIXELS_C 19
#define NUM_PIXELS_D 9

uint32_t colorsA[NUM_PIXELS_A];
uint32_t colorsB[NUM_PIXELS_B];
uint32_t colorsC[NUM_PIXELS_C];
uint32_t colorsD[NUM_PIXELS_D];

Adafruit_WS2801 stripA = Adafruit_WS2801(NUM_PIXELS_A, DATA_A_PIN, CLOCK_A_PIN);
Adafruit_WS2801 stripB = Adafruit_WS2801(NUM_PIXELS_B, DATA_B_PIN, CLOCK_B_PIN);
Adafruit_WS2801 stripC = Adafruit_WS2801(NUM_PIXELS_C, DATA_C_PIN, CLOCK_C_PIN);

// The WS2812 pixels are technically GRB, as are the WS2801. However, the 2801 library doesn't support this so I had
// to make a custom color function. To keep things simple, change the mode of the neopixels and everything works fine
Adafruit_NeoPixel stripD(NUM_PIXELS_D, DATA_D_PIN, NEO_BRG + NEO_KHZ800);

ClickButton toggleButton(BUTTON_PIN, LOW, CLICKBTN_PULLUP);

#define NUM_SECTIONS 4

Section sections[NUM_SECTIONS];

#define BRIGHTNESS_STEP_UP 3
#define BRIGHTNESS_STEP_DOWN 12

bool on = true;
uint8_t targetBrightness = 255;
uint8_t brightness = targetBrightness;

int mode = Standard;
unsigned long startMS = millis();

// Skip this many cycles
int delayCount = 0;
uint16_t rainbowCycle;

// I2C
uint8_t address = 0x03;

/* Main code */


void setup() {
  Serial.begin(9600);

  Wire.begin(address);
  Wire.onReceive(onRecieve);

  initSection(NUM_PIXELS_A, colorsA, &stripA, NULL, 0);
  initSection(NUM_PIXELS_B, colorsB, &stripB, NULL, 1);
  initSection(NUM_PIXELS_C, colorsC, &stripC, NULL, 2);
  initSection(NUM_PIXELS_D, colorsD, NULL, &stripD, 3);
}

void loop() {
  updateButtons();
  
  if (millis() - startMS > 60) {

    // Handle fading on/off if the brightness does not match target value
    if (on && brightness < targetBrightness) brightness = constrain(brightness + BRIGHTNESS_STEP_UP, 0, targetBrightness);
    if ((!on && brightness > 0) || brightness > targetBrightness) brightness = constrain(brightness - BRIGHTNESS_STEP_DOWN, 0, 255);
    
    startMS = millis();
    for (int i = 0; i < NUM_SECTIONS; i++) {
      updateSection(i);
    }

    delayCount = max(0, delayCount - 1);
    rainbowCycle += 1000;
  }
}

void onRecieve(int numBytes) {
  bool handled = false;
  
  while(Wire.available()) {
    if (!handled) {
      // Address Mapping

      // 0 - Mode
      // 1 - Power (On/Off)
      // 2 - Brightness
      
      uint8_t address = Wire.read();
      uint8_t value = Wire.read();
  
      if (address == 0) {
        if (value < ModeLength) updateMode(value, 15);
      } else if (address == 1) {
        on = value;
      } else if (address == 2) {
        targetBrightness = value;
      }

      handled = true;
    }
  }
}


/* Section functions */


void initSection(uint8_t numPixels, uint32_t colors[], Adafruit_WS2801 *strip, Adafruit_NeoPixel *strip2812, uint8_t sectionNumber) {
  Section *section = &sections[sectionNumber];
  
  section->NUM_PIXELS = numPixels;

  section->randomCount = generateRandomCount();
  section->count = 0;
  section->skipTime = generateSkipTime() / 2;

  section->frequency = generateFrequency(section->NUM_PIXELS);

  randomColor(section->color);

  section->colors = colors;
  if (strip != NULL) {
    section->strip = strip;
    strip->begin();
    strip->show();
  }
  
  else if (strip2812 != NULL) {
    section->strip2812 = strip2812;
    strip2812->begin();
    strip2812->show();
  }
}

void updateSection(uint8_t sectionIndex) {
  Section *section = &sections[sectionIndex];

  if (delayCount == 0) {
    if (mode == Rainbow) {
      // Offset the hue based on physical position so that everything lines up nicely
      uint16_t offset;
  
      switch(sectionIndex) {
        case 1:
        case 3:
        default:
          offset = 0;
          break;
        case 0:
          offset = 10000;
          break;
        case 2:
          offset = 15000;
          break;
      }
  
      uint32_t color = ColorHSV(rainbowCycle + offset, 255, 90);
      insert(section->colors, section->NUM_PIXELS, color);
    } else {
      if (section->skipTime == 0) {
        float factor = max(cos(millis() / section->frequency), 0);
        insert(section->colors, section->NUM_PIXELS, Color(section->color[0] * factor, section->color[1] * factor, section->color[2] * factor));
      } else {
        insert(section->colors, section->NUM_PIXELS, Color(0, 0, 0));
      }
    
      section->skipTime = max(0, section->skipTime - 1);
      section->count++;
      
      if (section->count > section->randomCount) {
        section->count = 0;
        section->frequency = generateFrequency(section->NUM_PIXELS);
    
        randomColor(section->color);
        section->randomCount = generateRandomCount();
        section->skipTime = generateSkipTime();
      }
    }
  }

  for (int i = 0; i < section->NUM_PIXELS; i++) {
    // Adjust color brightness (for fading on/off)
    uint32_t color = adjustBrightness(section->colors[i]);
    
    if (section->strip != NULL) section->strip->setPixelColor(i, color);
    else if (section->strip2812 != NULL) section->strip2812->setPixelColor(i, color);
  }

  if (section->strip != NULL) section->strip->show();
  else if (section->strip2812 != NULL) section->strip2812->show();
}


/* Helper functions */


void updateButtons() {
  toggleButton.Update();

  if (toggleButton.clicks == 1) {
    on = !on;
  }

  if (toggleButton.clicks == -1) {
    updateMode(mode + 1);
  }
}

void updateMode(uint8_t newMode) {
  updateMode(newMode, 30);
}

void updateMode(uint8_t newMode, uint8_t delay) {
  mode = newMode % ModeLength;
  
  uint32_t color;
  switch(mode) {
    case Standard:
      color = Color(0, 70, 40);
      break;
    case RainbowPastel:
      color = Color(70, 30, 30);
      break;
    case Rainbow:
      color = Color(80, 50, 0);
      break;
  }

  for (int i = 0; i < NUM_SECTIONS; i++) {
    fill(sections[i].colors, sections[i].NUM_PIXELS, color);
    randomColor(sections[i].color);
  }

  delayCount = delay;
}

float generateFrequency(uint8_t NUM_PIXELS) {
  switch(mode) {
    case RainbowPastel:
      return (float) random(150 / (NUM_PIXELS / 12.)) + 100;
    default:
      return (float) random(300 / (NUM_PIXELS / 12.)) + 100;
  }
}

int generateRandomCount() {
  switch(mode) {
    case RainbowPastel:
      return random(50) + 20;
    default:
      return random(100) + 20;
  }
}

int generateSkipTime() {
  switch(mode) {
    case RainbowPastel:
      return random(20);
    default:
      return random(60);
  }
}

void randomColor(uint8_t arr[]) {
  switch(mode) {
    case RainbowPastel:
      arr[0] = random(140);
      arr[1] = random(140);
      arr[2] = random(140);
      break;
    default:
      arr[0] = 0;
      arr[1] = random(40) + 160;
      arr[2] = random(50) + 120;
      break;
  }
}

void insert(uint32_t arr[], int size, int32_t color) {
  for (int i = size - 1; i > -1; i--) {
    if (i > 0) arr[i] = arr[i - 1];
  }

  arr[0] = color;
}

void fill(uint32_t arr[], int size, int32_t color) {
  for (int i = size - 1; i > -1; i--) {
    arr[i] = color;
  }
}

// Create a 24 bit color value from R,B,G
uint32_t Color(byte r, byte b, byte g)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

uint32_t adjustBrightness(uint32_t color) {
  uint8_t r,g,b;
  r = color >> 16;
  g = color >> 8;
  b = color;

  r = map(r, 0, 255, 0, brightness);
  g = map(g, 0, 255, 0, brightness);
  b = map(b, 0, 255, 0, brightness);

  return Color(r, g, b);
}

//Input a value 0 to 255 to get a color value.
//The colours are a transition r - g -b - back to r
uint32_t Wheel(byte WheelPos, float max)
{
  if (WheelPos < 85) {
   return Color(WheelPos * 3 * max, 255 - WheelPos * 3 * max, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return Color(255 - WheelPos * 3 * max, 0, WheelPos * 3 * max);
  } else {
   WheelPos -= 170; 
   return Color(0, WheelPos * 3 * max, 255 - WheelPos * 3 * max);
  }
}


// Code from megaTinyCore: https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/libraries/tinyNeoPixel_Static/tinyNeoPixel_Static.cpp
uint32_t ColorHSV(uint16_t hue, uint8_t sat, uint8_t val) {

  uint8_t r, g, b;

  // Remap 0-65535 to 0-1529.
  hue = (hue * 1530L + 32768) / 65536;

  // Convert hue to R,G,B (nested ifs faster than divide+mod+switch):
  if (hue < 510) {         // Red to Green-1
    b = 0;
    if (hue < 255) {       //   Red to Yellow-1
      r = 255;
      g = hue;            //     g = 0 to 254
    } else {              //   Yellow to Green-1
      r = 510 - hue;      //     r = 255 to 1
      g = 255;
    }
  } else if (hue < 1020) { // Green to Blue-1
    r = 0;
    if (hue <  765) {      //   Green to Cyan-1
      g = 255;
      b = hue - 510;      //     b = 0 to 254
    } else {              //   Cyan to Blue-1
      g = 1020 - hue;     //     g = 255 to 1
      b = 255;
    }
  } else if (hue < 1530) { // Blue to Red-1
    g = 0;
    if (hue < 1275) {      //   Blue to Magenta-1
      r = hue - 1020;     //     r = 0 to 254
      b = 255;
    } else {              //   Magenta to Red-1
      r = 255;
      b = 1530 - hue;     //     b = 255 to 1
    }
  } else {                // Last 0.5 Red (quicker than % operator)
    r = 255;
    g = b = 0;
  }

  // Apply saturation and value to R,G,B, pack into 32-bit result:
  uint32_t v1 =   1 + val; // 1 to 256; allows >>8 instead of /255
  uint16_t s1 =   1 + sat; // 1 to 256; same reason
  uint8_t  s2 = 255 - sat; // 255 to 0
  return ((((((r * s1) >> 8) + s2) * v1) & 0xff00) << 8) |
          (((((g * s1) >> 8) + s2) * v1) & 0xff00)       |
          (((((b * s1) >> 8) + s2) * v1)           >> 8);
}
