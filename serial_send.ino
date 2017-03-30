// Read analog value and send it using serial
//
//   This program for Arduino reads one channel and sends the data
//   out through the serial port in 2 bytes.
//   For synchronization purposes, the following scheme was chosen:
//   A0 data:   A09 (MSB) A08 A07 A06 A05 A04 A03 A02 A01 A00 (LSB)
//   sent as byte 1:   1 1 1 A09 A08 A07 A06 A05
//       and byte 2:   0 1 1 A04 A03 A02 A01 A00
//
//
//
int sensorValue = 0;        // value read from the pot
byte lb;
byte hb;
int VOICE_ACTIVE_PIN = 1;

void setup() {
  // initialize serial communications at 115200 bps:
  Serial.begin(115200);

  // initialize digital pin as an output.
  
}

void loop() {
// Insert your Code here  -----------------------------
  
  // read sensor pin data:

  // if you are speaking, turn the LED on on pin
  
  // otherwise turn it off



// -----------------------------------------------------

  // Now send the data (don't touch this)  --------------
  send_data();


}

void send_data() {
  // shift sample by 3 bits, and select higher byte
  hb = highByte(sensorValue << 3);
  // set 3 most significant bits and send out
  Serial.write(hb | 0b11100000);
  // select lower byte and clear 3 most significant bits
  lb = (lowByte(sensorValue)) & 0b00011111;
  // set bits 5 and 6 and send out
  Serial.write(lb | 0b01100000);

}

