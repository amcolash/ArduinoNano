#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <SPI.h>

#define DEBUG_PRINT true

#define LED_PIN 2
#define READY_PIN 3

// Pins to be used for output to ps1
#define ACK_PIN 9

// Pins to be used for input from controller
#define ATT_PIN 4
#define CMD_PIN 5
#define DAT_PIN 6
#define CLK_PIN 7

PsxControllerBitBang<ATT_PIN, CMD_PIN, DAT_PIN, CLK_PIN> psx;

#define DATA_BUFFER_SIZE 7
byte dataBuffer[DATA_BUFFER_SIZE];
uint8_t dataBufferIndex = 0;
bool willAck = false;

uint16_t buttons;
byte lx, ly, rx, ry;

bool haveController = false;

long timer;
bool turboX;

bool readingCard = false;
unsigned long readingTimer;

ISR (SPI_STC_vect) {
  byte cmd = SPDR;

  // Turn off output when card starts being read
  if (cmd == 0x81) {
    disableOutput();
  } else if (cmd == 0x52 || cmd == 0x00) {
    // Keep resetting the read timer while card continues to be read
    // Later on we will turn output back on 2ms after last read in loop()
    readingTimer = micros();
  }

  // Never go here when card is being accessed
  if (readingCard) return;

  willAck = dataBufferIndex < DATA_BUFFER_SIZE;
  if (cmd == 0x01) {
    clearBuffer();
    dataBuffer[0] = 0x73;
    
    willAck = true;
  } else if (cmd == 0x42) {
    dataBuffer[0] = 0x5A;
    dataBuffer[1] = (buttons << 8) >> 8;
    dataBuffer[2] = buttons >> 8;
    dataBuffer[3] = rx;
    dataBuffer[4] = ry;
    dataBuffer[5] = lx;
    dataBuffer[6] = ly;

    dataBufferIndex = 0;
    willAck = true;
  } else if (cmd != 0x00) {
    clearBuffer();
    willAck = false;

    printByte(cmd);
  }

  // ACK
  if (willAck) {
    fastDigitalWrite(ACK_PIN, LOW);
    delayMicroseconds(3);
    fastDigitalWrite(ACK_PIN, HIGH);
  }

  SPDR = dataBuffer[dataBufferIndex];

  if (dataBufferIndex >= DATA_BUFFER_SIZE) SPDR = 0xFF;

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

  fastPinMode(ACK_PIN, OUTPUT);
  fastDigitalWrite(ACK_PIN, HIGH);

  fastPinMode(READY_PIN, OUTPUT);
  fastPinMode(LED_PIN, OUTPUT);
  
  for (int i = 0; i < 3; i++) {
    fastDigitalWrite(LED_PIN, HIGH);
    delay(50);
    fastDigitalWrite(LED_PIN, LOW);
    delay(50);
  }

  fastDigitalWrite(READY_PIN, LOW);
  
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
    if (DEBUG_PRINT) {
      Serial.println("TIMER");
      printButtons();
    }
    
    if (millis() - timer > 80) {
      turboX = !turboX;
      timer = millis();
    }

    if (turboX) buttons |= PSB_CROSS;
    else buttons &= ~PSB_CROSS;

    fastDigitalWrite(LED_PIN, turboX);
  }
}

void loop (void) {
  updateController();
  updateTimer();

  // If card was being read and we haven't seen activity in 2ms, re-enable output to the bus
  if (readingCard && micros() - readingTimer > 2000) {
    enableOutput();
  }
}
