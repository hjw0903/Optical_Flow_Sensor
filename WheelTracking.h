/* Based on the code by Randy Mackay. DIYDrones.com
 ADNS3080.ino
 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

//Modified by Jungwon Hwang for tracking raw data

#include <SPI.h>

SPISettings spiSettings(2e6, MSBFIRST, SPI_MODE3); // 2 MHz, mode 3

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17

static const uint8_t RESET_PIN = 9;
static const uint8_t SS_PIN = 10; // Pin 10

static int32_t x, y;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to open

  SPI.begin();

  // Set SS and reset pin as output
  pinMode(SS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  reset();

  uint8_t config = spiRead(ADNS3080_CONFIGURATION_BITS);
  spiWrite(ADNS3080_CONFIGURATION_BITS, config | 0x10); // Set resolution to 1600 counts per inch
}

void loop() {

  updateSensor();

}

void updateSensor(void) {
  // Read sensor
  uint8_t buf[4];
  spiRead(ADNS3080_MOTION_BURST, buf, 4);
  uint8_t motion = buf[0];


  if (motion & 0x10){ // Check if we've had an overflow

    Serial.print("0");
    Serial.print('\n');}    

  else if (motion & 0x80) {
    int8_t dx = buf[1]; 
    int8_t dy = buf[2];
    uint8_t surfaceQuality = buf[3];

    x += dx;
    y += dy;

    // Print values
    Serial.print(x);
    Serial.print('\n');
    Serial.print(dx);
    Serial.print('\n');
    Serial.print(y);      
    Serial.print('\n');
    Serial.print(dy);
    Serial.print('\n');
    Serial.print(surfaceQuality);
    Serial.print('\n');
    Serial.flush();
  }
#if 0
  else
    Serial.println(motion, HEX);
#endif
  delay(10);
}

void reset(void) {
  digitalWrite(RESET_PIN, HIGH); // Set high
  delayMicroseconds(10);
  digitalWrite(RESET_PIN, LOW); // Set low
  delayMicroseconds(500); // Wait for sensor to get ready
}

void spiWrite(uint8_t reg, uint8_t data) {
  spiWrite(reg, &data, 1);
}

void spiWrite(uint8_t reg, uint8_t *data, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);

  SPI.transfer(reg | 0x80); // Indicate write operation
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  SPI.transfer(data, length); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t spiRead(uint8_t reg) {
  uint8_t buf;
  spiRead(reg, &buf, 1);
  return buf;
}

void spiRead(uint8_t reg, uint8_t *data, uint8_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(SS_PIN, LOW);

  SPI.transfer(reg); // Send register address
  delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
  memset(data, 0, length); // Make sure data buffer is 0
  SPI.transfer(data, length); // Write data

  digitalWrite(SS_PIN, HIGH);
  SPI.endTransaction();
}

