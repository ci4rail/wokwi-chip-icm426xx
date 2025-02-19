#include <SPI.h>
#include <string.h>

#define CS 10

#define ICM426XX_INT_STATUS 0x2D  // 8 bits
#define ICM426XX_FIFO_COUNTH 0x2E   // 16 bits
#define ICM426XX_FIFO_DATA 0x30 // any length
#define ICM426XX_GYRO_CONFIG0 0x4F // 8 bits  - ODR
#define ICM426XX_ACCEL_CONFIG0 0x50 // 8 bits - ODR
#define ICM426XX_PWR_MGMT0 0x4E // 8 bits - enable/disable
#define ICM426XX_FIFO_CONFIG1 0x5F // 8 bits - fifo packet
#define ICM426XX_REG_BANK_SEL 0x76 // 8 bits - select bank


void dumpBuffer(uint8_t *buffer, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


void spiTransfer(uint8_t *buffer, size_t len)
{
  digitalWrite(CS, LOW);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.transfer(buffer, len);
  SPI.end();
  digitalWrite(CS, HIGH);
}


void write_reg(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {0};
  buffer[0] = reg;
  buffer[1] = value;
  spiTransfer(buffer, sizeof(buffer));
}

uint8_t read_reg(uint8_t reg)
{
  uint8_t buffer[2] = {0};
  buffer[0] = 0x80 | reg;
  spiTransfer(buffer, sizeof(buffer));
  return buffer[1];
}


uint8_t read_reg16(uint8_t reg)
{
  uint8_t buffer[3] = {0};
  buffer[0] = 0x80 | reg;
  spiTransfer(buffer, sizeof(buffer));
  return (((uint16_t)buffer[2]) << 8) | buffer[1];
}



void setup()
{
  Serial.begin(115200);
  pinMode(CS, OUTPUT);

  write_reg(ICM426XX_REG_BANK_SEL, 0x00);
  write_reg(ICM426XX_GYRO_CONFIG0, 0xaa);
  uint8_t v;
  v = read_reg(ICM426XX_GYRO_CONFIG0);
  Serial.print("Gyro config: ");
  Serial.println(v, HEX);
}

void loop()
{
  delay(500);
}
