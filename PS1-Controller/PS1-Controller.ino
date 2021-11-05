#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <SPI.h>
#include <Watchdog.h>

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

uint16_t buttons;
byte lx, ly, rx, ry;

bool haveController = false;

Watchdog watchdog;

long timer;
bool turboX;

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

  SPI.detachInterrupt(); // This causes issues if not actually using interrupt (we are polling)

  watchdog.enable(Watchdog::TIMEOUT_2S); // If SPI hangs, just restart
  
  Serial.println("All Set");
}

byte sendByte(byte b) {
  return sendByte(b, true);
}

byte sendByte(byte b, bool sendAck) {
  fastDigitalWrite(READY_PIN, HIGH);
  byte ret = SPI.transfer(b);
  fastDigitalWrite(READY_PIN, LOW);
  
  if (sendAck &&
    ret != 0x43 &&
    ret != 0x81) ack();
  
  return ret;
}

void ack() {
  delayMicroseconds(8); // Need to wait for last clock pulse before acking
  fastDigitalWrite(ACK_PIN, LOW);
//  delayMicroseconds(2);
  micros();
  fastDigitalWrite(ACK_PIN, HIGH);
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

void updateCmd() {
  // Get next command
  byte cmd = sendByte(0xFF);
  
  // Only proceed if we are starting with a "controller" command (excluding memory card w/ command 0x81)
  if (cmd != 0x01 || fastDigitalRead(SS)) return;

  // We are an analog dualshock controller
  cmd = sendByte(0x73);
  
  switch(cmd) {
    case 0x42: // Poll Buttons
      cmd = sendByte(0x5A); // Tell PS1 we are sending poll update

//      if (buttons != 0xFFFF) printButtons();

      if (cmd == 0x00) cmd = sendByte((buttons << 8) >> 8); // Buttons part 1
      if (cmd == 0x00) cmd = sendByte(buttons >> 8); // Buttons part 2
      
      if (cmd == 0x00) cmd = sendByte(rx); // Right Joy X-Axis
      if (cmd == 0x00) cmd = sendByte(ry); // Right Joy Y-Axis
      if (cmd == 0x00) cmd = sendByte(lx); // Left Joy X-Axis
      if (cmd == 0x00) cmd = sendByte(ly, false); // Left Joy Y-Axis
      
      break;
    default:
      Serial.print("Unknown Command: ");
      printByte(cmd);
      break;
    case 0x00: // NOP?
    case 0x01: // Init (should have already been handled...)
    case 0x43: // Config Mode (But also poll)
    case 0x4D: // Enable Rumble
    case 0xFF: // NOP?
      if (cmd != 0x43) {
        Serial.print("Ignoring Command: ");
        printByte(cmd);
      }
      break;
  }
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
    Serial.println("TIMER");
    printButtons();
    
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
  updateCmd();
  updateController();
  updateTimer();

  watchdog.reset();
}
