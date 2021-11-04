#include <SPI.h>

#define ACK 9

uint16_t buttons = 0;
unsigned long timer;

void setup (void) {
  Serial.begin (115200);   // debugging

  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
  pinMode(ACK, OUTPUT);

  digitalWrite(ACK, HIGH);
  
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
    digitalWrite(ACK, LOW);
    delayMicroseconds(2);
    digitalWrite(ACK, HIGH);
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

void loop (void) {

  // Get next command
  byte cmd = sendByte(0xFF);
  if (cmd == 0x01) cmd = sendByte(0x73, true);
  
  switch(cmd) {
    case 0x42: // Poll Buttons
      sendByte(0x5A); // Tell PS1 we are sending poll update

      sendByte(buttons >> 8); // Buttons part 1
      sendByte((buttons << 8) >> 8); // Buttons part 2
      
      sendByte(0x80); // Right Joy X-Axis
      sendByte(0x80); // Right Joy Y-Axis
      sendByte(0x80); // Left Joy X-Axis
      sendByte(0x80, false); // Left Joy Y-Axis
      
      break;
    default:
      Serial.print("Unknown Command: ");
      printByte(cmd);
    case 0x00: // NOP?
    case 0x43: // Config Mode (But also poll)
    case 0x4D: // Enable Rumble
    case 0xFF: // NOP?
      // TODO
      break;
  }

  if (millis() - timer > 100) {
    if (buttons == 0) buttons = 65535;
    else buttons /= 2;

    timer = millis();
  }
}
