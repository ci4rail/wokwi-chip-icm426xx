#include <SPI.h>
#include <string.h>

#define CS 10

#define ICM426XX_INT_STATUS 0x2D  // 8 bits
#define ICM426XX_FIFO_COUNTH 0x2E   // 16 bits
#define ICM426XX_FIFO_DATA 0x30 // any length
#define ICM426XX_ACCEL_CONFIG0 0x50 // 8 bits - ODR 
#define ICM426XX_ACCEL_CONFIG2 0x60 // 8 bits - FIFO watermark
#define ICM426XX_ACCEL_CONFIG3 0x61 // 8 bits - FIFO watermark MSB
#define ICM426XX_PWR_MGMT0 0x4E // 8 bits - enable/disable
#define ICM426XX_REG_BANK_SEL 0x76 // 8 bits - select bank

typedef struct __attribute__((__packed__)){
  uint8_t header;
  uint16_t accel_x;
  uint16_t accel_y;
  uint16_t accel_z;
  uint16_t gyro_x;
  uint16_t gyro_y;
  uint16_t gyro_z;
  uint8_t temp;
  uint16_t timestamp;
} packet3_t;


void dumpBuffer(uint8_t *buffer, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}


void spi_transfer(uint8_t *buffer, size_t len)
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
  spi_transfer(buffer, sizeof(buffer));
}

uint8_t read_reg(uint8_t reg)
{
  uint8_t buffer[2] = {0};
  buffer[0] = 0x80 | reg;
  spi_transfer(buffer, sizeof(buffer));
  return buffer[1];
}


uint8_t read_reg16(uint8_t reg)
{
  uint8_t buffer[3] = {0};
  buffer[0] = 0x80 | reg;
  spi_transfer(buffer, sizeof(buffer));
  return (((uint16_t)buffer[2]) << 8) | buffer[1];
}

// caller shall discard the first byte of the buffer
void read_fifo(uint8_t *buffer, size_t size)
{
  uint8_t reg = ICM426XX_FIFO_DATA;
  buffer[0] = 0x80 | reg;
  spi_transfer(buffer, size);
}

void setup()
{
  Serial.begin(115200);
  pinMode(CS, OUTPUT);
  Serial.println("Setup");
  write_reg(ICM426XX_REG_BANK_SEL, 0x00);
  write_reg(ICM426XX_ACCEL_CONFIG0, 0x9); // 50Hz
  write_reg(ICM426XX_ACCEL_CONFIG2, 0x40); // watermark
  write_reg(ICM426XX_ACCEL_CONFIG3, 0x0); // watermark MSB
  uint8_t v;
  v = read_reg(ICM426XX_ACCEL_CONFIG0);
  Serial.print("Accel config: ");
  Serial.println(v, HEX);

  write_reg(ICM426XX_PWR_MGMT0, 0xf); // enable
}

uint8_t buffer[1500];
void loop()
{
  static int loop_cnt;
  delay(100);
  Serial.println("Loop");
  loop_cnt++;
  Serial.println(loop_cnt);
  uint8_t int_status;
  uint16_t fifo_count;

  int_status = read_reg(ICM426XX_INT_STATUS);
  Serial.print("INT_STATUS: ");
  Serial.println(int_status, HEX);

  fifo_count = read_reg16(ICM426XX_FIFO_COUNTH);
  Serial.print("FIFO count: ");
  Serial.println(fifo_count, HEX);

  if(int_status & 0x04){
    Serial.println("FIFO watermark");
    
    size_t size = sizeof(packet3_t)*fifo_count+1;
    if(size < sizeof(buffer)){
      read_fifo(buffer, size);
      dumpBuffer(buffer+1, size-1);
    } else {
      Serial.println("FIFO too large");
    }
  }
}
