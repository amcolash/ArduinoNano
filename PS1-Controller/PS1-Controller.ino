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

bool haveController = false;

long turboTimer;
bool turboX;

uint8_t cmdIndex = 0;

bool configMode = false;

bool readingCard = false;

void spiStateChange() {
  cmdIndex = 0;
  
  bool ss_enabled = !fastDigitalRead(SS_DUPE_PIN);

  // Reset things when we exit SS and were previously reading a memory card
  if (!ss_enabled && readingCard) {
    enableOutput();
  }
}

ISR (SPI_STC_vect) {
  byte cmd = SPDR;
  cmdIndex++;

  // Turn off output when card starts being read
  if (!readingCard && cmd == 0x81) {
    disableOutput();
  }
  
  // Never go further when card is being accessed
  if (readingCard) return;

  SPDR = 0xFF;

  if (cmd == 0x01) { // Using controller
    clearBuffer();
    dataBuffer[0] = 0x73;

  } else if (cmd == 0x42) { // || cmd == 0x43) { // Poll / Config Mode
    clearBuffer();
    dataBuffer[0] = 0x5A;

    // Unsure if this is necessary or just overcomplicating things
//      if (cmd == 0x42 || !configMode) {
      dataBuffer[1] = (buttons << 8) >> 8;
      dataBuffer[2] = buttons >> 8;
      dataBuffer[3] = rx;
      dataBuffer[4] = ry;
      dataBuffer[5] = lx;
      dataBuffer[6] = ly;
//      }

    dataBufferIndex = 0;
//    } else if (cmd == 0x45) { // Status
//      dataBuffer[0] = 0x5A;
//      dataBuffer[1] = 0x01; // Controller id
//      dataBuffer[2] = 0x02;
//      dataBuffer[3] = 0x01; // Analog on/off
//      dataBuffer[4] = 0x02;
//      dataBuffer[5] = 0x01;
//      dataBuffer[6] = 0x00;
//
//      dataBufferIndex = 0;
//    } else if (cmd == 0x4D) { // Configure rumble
//      clearBuffer();
//      dataBuffer[0] = 0x5A;
  } else if (cmd != 0x00) {
    clearBuffer();

    if (cmd != 0x43) {
      Serial.print("Unknown command: ");
      printByte(cmd);
    }
    
    return;
  }

  // ACK
  if (dataBufferIndex < DATA_BUFFER_SIZE) {
    fastDigitalWrite(ACK_PIN, LOW);
    delayMicroseconds(3);
    fastDigitalWrite(ACK_PIN, HIGH);
  }

  if (dataBufferIndex < DATA_BUFFER_SIZE) SPDR = dataBuffer[dataBufferIndex];

  dataBufferIndex = constrain(dataBufferIndex + 1, 0, DATA_BUFFER_SIZE);
}

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

void clearBuffer() {
  for (uint8_t i; i < DATA_BUFFER_SIZE; i++) {
    dataBuffer[i] = 0xFF;
  }

  dataBufferIndex = 0;
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

  Serial.println("All Set");
}

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

void updateController() {
  if (!haveController) {
    if (psx.begin()) {
      Serial.println("Found controller");
      haveController = true;

      updateButtons();
    }
  } else {
    if (!psx.read()) {
      Serial.println("Lost controller");
      haveController = false;
    } else {
      updateButtons();
    }
  }

  fastDigitalWrite(LED_PIN, haveController);
}

void printButtons() {
  printByte((buttons << 8) >> 8); // Buttons part 1
  printByte(buttons >> 8); // Buttons part 2
}

void updateButtons() {
  buttons = ~psx.getButtonWord();
  psx.getLeftAnalog (lx, ly);
  psx.getRightAnalog (rx, ry);
}

void updateTimer() {
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

void loop (void) {
  updateController();
  updateTimer();
}
