#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <SPI.h>

#define DEBUG_PRINT true

#define LED_PIN 2

// Pins to be used for output to ps1
#define SS_DUPE_PIN 3 // Trigger an interrupt when SS changes state
#define ACK_PIN 9

// Pins to be used for input from controller
#define ATT_PIN 4
#define CMD_PIN 5
#define DAT_PIN 6
#define CLK_PIN 7

PsxControllerBitBang<ATT_PIN, CMD_PIN, DAT_PIN, CLK_PIN> psx;

#define DATA_BUFFER_SIZE 7
uint8_t dataBufferIndex = 0;
byte dataBuffer[DATA_BUFFER_SIZE];

uint16_t buttons;
byte lx, ly, rx, ry;
uint8_t rumble1 = 0;
uint8_t rumble1Index = 0x00;
uint8_t rumble2 = 0;
uint8_t rumble2Index = 0x01;

bool haveController = false;

long turboTimer;
bool turboX;

bool readingCard = false;
bool configMode = false;

uint8_t cmdIndex = 0;
byte currentCmd;

/** Interrupt Handlers */

// Called when SS changes state
void spiStateChange() {
  cmdIndex = 0;
  
  bool ss_enabled = !fastDigitalRead(SS_DUPE_PIN);

  // Reset things when we exit SS and were previously reading a memory card
  if (!ss_enabled && readingCard) {
    enableOutput();
  }
}

// Called whenever SPI data is recieved from PS1 or USB controller device
ISR (SPI_STC_vect) {
  byte cmd = SPDR;

  // Keep track of the main command
  if (cmdIndex == 1) {
    currentCmd = cmd;
  }

  // Turn off output when card starts being read
  if (!readingCard && cmd == 0x81) {
    disableOutput();
  }
  
  // Never go further when card is being accessed
  if (readingCard) return;

  SPDR = 0xFF;

  if (cmdIndex == 0) {
    if (cmd == 0x01) { // Using controller
      clearBuffer();
      dataBuffer[0] = 0x73;
    }
  } else if (cmdIndex == 1) {
    if (cmd == 0x42 || cmd == 0x43) { // Poll / Config Mode
      clearBuffer();
      dataBuffer[0] = 0x5A;
  
      // Unsure if this is necessary or just over-complicating things
//      if (cmd == 0x43 && configMode) return;
      
      dataBuffer[1] = (buttons << 8) >> 8;
      dataBuffer[2] = buttons >> 8;
      dataBuffer[3] = rx;
      dataBuffer[4] = ry;
      dataBuffer[5] = lx;
      dataBuffer[6] = ly;
  
      dataBufferIndex = 0;
    } else if (cmd == 0x45) { // Status
      dataBuffer[0] = 0x5A;
      dataBuffer[1] = 0x01; // Controller id
      dataBuffer[2] = 0x02;
      dataBuffer[3] = 0x01; // Analog on/off
      dataBuffer[4] = 0x02;
      dataBuffer[5] = 0x01;
      dataBuffer[6] = 0x00;

      dataBufferIndex = 0;
    } else if (cmd == 0x4D) { // Configure rumble
      clearBuffer();
      dataBuffer[0] = 0x5A;
    } else {
      clearBuffer();
  
      Serial.print("Unknown command: ");
      printByte(cmd);
      
      return;
    }
  // Handle commands that modify state here
  } else {
    if (currentCmd == 0x42 && (cmdIndex == rumble1Index || cmdIndex == rumble2Index)) {
      if (cmdIndex == rumble1Index) rumble1 = cmd;
      if (cmdIndex == rumble2Index) rumble2 = cmd;
    } else if (currentCmd == 0x43 && cmdIndex == 3) {
      configMode = cmd;
    } else if (currentCmd == 0x4D) {
      if (cmd == 0x00) rumble1Index = cmdIndex;
      if (cmd == 0x01) rumble2Index = cmdIndex;
    }
  }

  // ACK
  if (dataBufferIndex < DATA_BUFFER_SIZE) {
    fastDigitalWrite(ACK_PIN, LOW);
    delayMicroseconds(3);
    fastDigitalWrite(ACK_PIN, HIGH);
  }

  // Set up next byte to be sent
  if (dataBufferIndex < DATA_BUFFER_SIZE) SPDR = dataBuffer[dataBufferIndex];
  dataBufferIndex = constrain(dataBufferIndex + 1, 0, DATA_BUFFER_SIZE);

  // Increment cmdIndex after handled
  cmdIndex++;
}

void setup (void) {
  Serial.begin (115200);

  // Setup output (to ps1) pins
  fastPinMode(SCK, INPUT);
  fastPinMode(MOSI, INPUT);
  fastPinMode(MISO, OUTPUT);
  fastPinMode(SS, INPUT);

  fastPinMode(SS_DUPE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SS_DUPE_PIN), spiStateChange, CHANGE);

  fastPinMode(ACK_PIN, OUTPUT);
  fastDigitalWrite(ACK_PIN, HIGH);

  fastPinMode(LED_PIN, OUTPUT);
  
  for (int i = 0; i < 3; i++) {
    fastDigitalWrite(LED_PIN, HIGH);
    delay(50);
    fastDigitalWrite(LED_PIN, LOW);
    delay(50);
  }
  
  // turn on SPI in slave mode
  SPCR = _BV(SPE) | _BV(DORD) | _BV(SPR0) | ~_BV(MSTR);

  // Clear MISO to begin with
  SPDR = 0xFF;

  Serial.println();
  Serial.println("Starting virtual controller");
}

void loop (void) {
  updateController();
  updateTurboTimer();
}

/** Loop Functions */

void updateButtons() {
  buttons = ~psx.getButtonWord();
  psx.getLeftAnalog (lx, ly);
  psx.getRightAnalog (rx, ry);
}

void updateController() {
  if (!haveController) {
    if (psx.begin()) {
      Serial.println("Found PS1 controller");
      haveController = true;

      Serial.print("Entering config mode: ");
      Serial.println(psx.enterConfigMode());

      Serial.print("Enabling rumble: ");
      Serial.println(psx.enableRumble());

      Serial.print("Exiting config mode: ");
      Serial.println(psx.exitConfigMode());
      
      updateButtons();
    }
  } else {
    if (!psx.read()) {
      Serial.println("Lost PS1 controller");
      haveController = false;
    } else {

      psx.setRumble(rumble1 > 0, rumble2);
      updateButtons();
    }
  }

  fastDigitalWrite(LED_PIN, haveController);
}

void updateTurboTimer() {
  if (~buttons & PSB_L2) {
    if (millis() - turboTimer > 80) {
      turboX = !turboX;
      turboTimer = millis();
    }

    if (turboX) buttons |= PSB_CROSS;
    else buttons &= ~PSB_CROSS;

    fastDigitalWrite(LED_PIN, turboX);
  }
}

/** Controller Util */

// Disable output to MISO/ACK while card is being accessed
void disableOutput() {
  fastPinMode(MISO, INPUT);
  fastPinMode(ACK_PIN, INPUT);
  
  readingCard = true;
}

// Renable input after card read
void enableOutput() {
  fastPinMode(MISO, OUTPUT);
  fastPinMode(ACK_PIN, OUTPUT);
  fastDigitalWrite(ACK_PIN, HIGH);
  
  readingCard = false;
}

// Clear output buffer with 0xFF and reset index
void clearBuffer() {
  for (uint8_t i; i < DATA_BUFFER_SIZE; i++) {
    dataBuffer[i] = 0xFF;
  }

  dataBufferIndex = 0;
}

/** Util */

// From https://forum.arduino.cc/t/printing-binary/437864/2
void printByte(byte b) {
  if (!DEBUG_PRINT) return;
  
  for (int i = 7; i >= 0; i--)
  {
    if (i == 3) Serial.print(" ");
    Serial.print(bitRead(b, i));
  }

  Serial.print("\t[0x");
  Serial.print(b, HEX);
  Serial.println("]");
}

void printButtons() {
  printByte((buttons << 8) >> 8); // Buttons part 1
  printByte(buttons >> 8); // Buttons part 2
}
