#include <Adafruit_NeoPixel.h>
#include "Adafruit_WS2801.h"
#include "ClickButton.h"

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

bool on = true;
int mode = Standard;
unsigned long startMS = millis();

// Skip this many cycles
int delayCount = 0;

/* Main code */


void setup() {
  Serial.begin(9600);

  initSection(NUM_PIXELS_A, colorsA, &stripA, NULL, 0);
  initSection(NUM_PIXELS_B, colorsB, &stripB, NULL, 1);
  initSection(NUM_PIXELS_C, colorsC, &stripC, NULL, 2);
  initSection(NUM_PIXELS_D, colorsD, NULL, &stripD, 3);
}

void loop() {
  updateButtons();
  
  if (millis() - startMS > 60) {
    startMS = millis();
    for (int i = 0; i < NUM_SECTIONS; i++) {
      updateSection(i);
    }

    delayCount = max(0, delayCount - 1);
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

  for (int i = 0; i < section->NUM_PIXELS; i++) {
    uint32_t color = on ? section->colors[i] : 0;
    
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
    mode = (mode + 1) % ModeLength;

    uint32_t color;
    switch(mode) {
      case Standard:
        color = Color(0, 70, 40);
        break;
      case Rainbow:
        color = Color(70, 30, 30);
        break;
    }

    for (int i = 0; i < NUM_SECTIONS; i++) {
      fill(sections[i].colors, sections[i].NUM_PIXELS, color);
      randomColor(sections[i].color);
    }

    delayCount = 30;
  }
}

float generateFrequency(uint8_t NUM_PIXELS) {
  switch(mode) {
    case Rainbow:
      return (float) random(200 * (NUM_PIXELS / 12.)) + 100;
    default:
      return (float) random(400 * (NUM_PIXELS / 12.)) + 100;
  }
}

int generateRandomCount() {
  switch(mode) {
    case Rainbow:
      return random(50) + 20;
    default:
      return random(100) + 20;
  }
}

int generateSkipTime() {
  switch(mode) {
    case Rainbow:
      return random(20);
    default:
      return random(100);
  }
}

void randomColor(uint8_t arr[]) {
  switch(mode) {
    case Rainbow:
      arr[0] = random(80);
      arr[1] = random(80);
      arr[2] = random(80);
      break;
    default:
      arr[0] = 0;
      arr[1] = random(20) + 50;
      arr[2] = random(30) + 10;
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
