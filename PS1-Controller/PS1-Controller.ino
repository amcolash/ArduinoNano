#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <SPI.h>

#define LED_PIN 2

// Pins to be used for output to ps1
#define ACK_PIN 9

// Pins to be used for input from controller
#define ATT_PIN 4
#define CMD_PIN 5
#define DAT_PIN 6
#define CLK_PIN 7

PsxControllerBitBang<ATT_PIN, CMD_PIN, DAT_PIN, CLK_PIN> psx;

uint16_t buttons;
byte lx, ly, rx, ry;

bool haveController = false;

void setup (void) {
  Serial.begin (115200);

  // Setup output (to ps1) pins
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  fastPinMode(ACK_PIN, OUTPUT);
  fastDigitalWrite(ACK_PIN, HIGH);

  fastPinMode(LED_PIN, OUTPUT);
  fastDigitalWrite(LED_PIN, LOW);
  
  // turn on SPI in slave mode
  SPCR = _BV(SPE) | _BV(DORD) | _BV(SPR0) | ~_BV(MSTR);

  SPI.detachInterrupt(); // This causes issues if not actually using interrupt (we are polling)
  
  Serial.println("All Set");
}

byte sendByte(byte b) {
  return sendByte(b, true);
}

byte sendByte(byte b, bool sendAck) {
  byte ret = SPI.transfer(b);
  
  if (sendAck && ret != 0x43) {
    delayMicroseconds(4); // Need to wait for last clock pulse before acking
    fastDigitalWrite(ACK_PIN, LOW);
    delayMicroseconds(2);
    fastDigitalWrite(ACK_PIN, HIGH);
  }
  
  return ret;
}

// From https://forum.arduino.cc/t/printing-binary/437864/2
void printByte(byte b) {
  for (int i = 7; i >= 0; i--)
  {
    if (i == 3) Serial.print(" ");
    Serial.print(bitRead(b, i));
  }

  Serial.print("\t[0x");
  Serial.print(b, HEX);
  Serial.println("]");
}

void updateButtons() {
  buttons = ~psx.getButtonWord();
  psx.getLeftAnalog (lx, ly);
  psx.getRightAnalog (rx, ry);
}

void loop (void) {
  // Get next command
  byte cmd = sendByte(0xFF);
  if (cmd == 0x01) cmd = sendByte(0x73, true);
  
  switch(cmd) {
    case 0x42: // Poll Buttons
      sendByte(0x5A); // Tell PS1 we are sending poll update

      sendByte((buttons << 8) >> 8); // Buttons part 1
      sendByte(buttons >> 8); // Buttons part 2
      
      sendByte(rx); // Right Joy X-Axis
      sendByte(ry); // Right Joy Y-Axis
      sendByte(lx); // Left Joy X-Axis
      sendByte(ly, false); // Left Joy Y-Axis
      
      break;
    default:
      Serial.print("Unknown Command: ");
      printByte(cmd);
    case 0x00: // NOP?
    case 0x01: // Init (should have already been handled...)
    case 0x43: // Config Mode (But also poll)
    case 0x4D: // Enable Rumble
    case 0xFF: // NOP?
      // TODO
      break;
  }

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
