#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// Registers to simulate
#define ICM426XX_INT_STATUS 0x2D  // 8 bits
#define ICM426XX_FIFO_COUNTH 0x2E   // 16 bits
#define ICM426XX_FIFO_DATA 0x30 // any length
#define ICM426XX_ACCEL_CONFIG0 0x50 // 8 bits - ODR 
#define ICM426XX_ACCEL_CONFIG2 0x60 // 8 bits - FIFO watermark
#define ICM426XX_ACCEL_CONFIG3 0x61 // 8 bits - FIFO watermark MSB
#define ICM426XX_PWR_MGMT0 0x4E // 8 bits - enable/disable
#define ICM426XX_REG_BANK_SEL 0x76 // 8 bits - select bank

typedef struct
{
  pin_t cs_pin;
  uint32_t spi;
  uint8_t spi_buffer[1];
  uint8_t spi_data_from_host[3]; // reg, data0, data1
  uint32_t spi_data_from_host_idx;
  uint8_t reg_addr;
  uint8_t selected_bank;
  bool is_write;
  uint16_t regs[0x80]; // bank 0 only
  uint8_t fifo[2048];
  uint16_t fifo_read_idx;
  uint16_t fifo_write_idx;
  uint16_t fifo_count;
  timer_t sample_timer;
  uint32_t accel_amplitude_attr;
  uint32_t gyro_amplitude_attr;
} chip_state_t;

typedef struct __attribute__((__packed__)){
  uint8_t header;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int8_t temp;
  uint16_t timestamp;
} packet3_t;

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count);
static uint32_t odr_to_millihz(uint8_t odr);

static void update_fifo_count(chip_state_t *chip, int16_t count)
{
  chip->fifo_count += count;
  //printf("FIFO count %d\n", chip->fifo_count);

  uint16_t records = chip->fifo_count/sizeof(packet3_t);
  chip->regs[ICM426XX_FIFO_COUNTH] = records & 0xff;
  chip->regs[ICM426XX_FIFO_COUNTH+1] = (records >> 8) & 0xff;
}

static void put_fifo_data(chip_state_t *chip, uint8_t *buffer, size_t size)
{
  size_t update_size = size;
  if(chip->fifo_count + size > sizeof(chip->fifo)){
    // discard oldest data
    chip->fifo_read_idx = (chip->fifo_read_idx + size) % sizeof(chip->fifo);
    update_size = 0;

    printf("FIFO overflow\n");
    chip->regs[ICM426XX_INT_STATUS] |= 0x02; // FIFO full
  }
  memcpy(chip->fifo + chip->fifo_write_idx, buffer, size);
  chip->fifo_write_idx = (chip->fifo_write_idx + size) % sizeof(chip->fifo);
  update_fifo_count(chip, update_size);

  uint16_t fifo_wm = chip->regs[ICM426XX_ACCEL_CONFIG2] | (chip->regs[ICM426XX_ACCEL_CONFIG3]<<8);
  if(chip->fifo_count/sizeof(packet3_t) >= fifo_wm){
    printf("FIFO watermark\n");
    chip->regs[ICM426XX_INT_STATUS] |= 0x04; // FIFO watermark
  }
}

static uint8_t read_fifo_reg(chip_state_t *chip)
{
  if(chip->fifo_count == 0){
    printf("FIFO empty!\n");
    return 0x80; // FIFO empty
  }
  uint8_t data = chip->fifo[chip->fifo_read_idx];
  chip->fifo_read_idx = (chip->fifo_read_idx + 1) % sizeof(chip->fifo);
  update_fifo_count(chip, -1);
  return data;
}

// amplitude: 0.0 to 1.0
static int16_t gen_random_int16(double amplitude)
{
  int16_t r = (uint32_t)(random() * amplitude) >> 16;
  if(random() & 1){
    r = -r;
  }
  return r;
}

static void on_sample_timer(void *user_data)
{
  chip_state_t *chip = user_data;
  if(!(chip->regs[ICM426XX_PWR_MGMT0] & 0x02)){
    // chip not turned on
    return;
  }
  float accel_amplitude = attr_read_float(chip->accel_amplitude_attr);
  float gyro_amplitude = attr_read_float(chip->gyro_amplitude_attr);

  packet3_t packet = {
      .header = 0x68, // HEADER_MSG=0 HEADER_ACCEL=1 HEADER_GYRO=1 HEADER_20=0 ...
      .accel_x = gen_random_int16(accel_amplitude),
      .accel_y = gen_random_int16(accel_amplitude),
      .accel_z = gen_random_int16(accel_amplitude),
      .gyro_x = gen_random_int16(gyro_amplitude),
      .gyro_y = gen_random_int16(gyro_amplitude),
      .gyro_z = gen_random_int16(gyro_amplitude),
      .temp = 44,
      .timestamp = (get_sim_nanos() / 1000) & 0xffff,
  };
  //printf("Sample timer pkt size %ld\n", sizeof(packet));
  put_fifo_data(chip, (uint8_t *)&packet, sizeof(packet));
}

void chip_init(void)
{
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  memset(chip->regs, 0, sizeof(chip->regs));

  chip->accel_amplitude_attr = attr_init_float("accelamplitude", 1.0);
  chip->gyro_amplitude_attr = attr_init_float("gyroamplitude", 1.0);

  chip->cs_pin = pin_init("CS", INPUT_PULLUP);

  chip->sample_timer = timer_init(&(timer_config_t){
      .user_data = chip,
      .callback = on_sample_timer,
  });

  const pin_watch_config_t watch_config = {
      .edge = BOTH,
      .pin_change = chip_pin_change,
      .user_data = chip,
  };
  pin_watch(chip->cs_pin, &watch_config);


  const spi_config_t spi_config = {
      .sck = pin_init("SCK", INPUT),
      .miso = pin_init("MISO", INPUT),
      .mosi = pin_init("MOSI", INPUT),
      .done = chip_spi_done,
      .user_data = chip,
      .mode = 0,
  };
  chip->spi = spi_init(&spi_config);
}

static uint8_t read_reg(chip_state_t *chip, uint8_t reg)
{
  uint8_t data = 0;
  if(chip->selected_bank != 0){
    return 0; // no other banks supported
  }
  if( reg != ICM426XX_FIFO_DATA){
    data = chip->regs[reg];
    switch(reg){
    case ICM426XX_INT_STATUS:
      //printf("Reading INT_STATUS\n");
      chip->regs[ICM426XX_INT_STATUS] = 0; // clear interrupt
      break;
    }
  } else {
    //printf("Reading FIFO data\n");
    data = read_fifo_reg(chip);
  }
  return data;
}

static void write_reg(chip_state_t *chip, uint8_t reg, uint8_t value)
{
  if(chip->selected_bank != 0){
    return; // no other banks supported
  }
  switch(reg){
  case ICM426XX_REG_BANK_SEL:
    printf("Select bank %d\n", value);
    chip->selected_bank = value;
    return;
  case ICM426XX_ACCEL_CONFIG0:
    printf("Setting accel ODR to %d\n", value);
    chip->regs[reg] = value;
    uint64_t tv = 1E12/odr_to_millihz(value&0xf);
    //printf("Setting timer to %lld\n", tv);
    timer_stop(chip->sample_timer);
    timer_start_ns(chip->sample_timer, tv, true);
    return;
  }
  printf("Writing reg %02x with value %02x\n", reg, value);
  chip->regs[reg] = value;
}


static void write_regs(chip_state_t *chip)
{
  uint8_t reg_addr = chip->spi_data_from_host[0] & 0x7f;

  for(int i=0; i<chip->spi_data_from_host_idx-1; i++){
    write_reg(chip, reg_addr+i, chip->spi_data_from_host[i+1]);
  }
}


static void chip_pin_change(void *user_data, pin_t pin, uint32_t value)
{
  chip_state_t *chip = (chip_state_t *)user_data;
  // Handle CS pin logic
  if (pin == chip->cs_pin)
  {
    if (value == LOW)
    {
      //printf("SPI chip selected\n");
      chip->spi_buffer[0] = ' '; // dummy byte
      chip->spi_data_from_host_idx = 0;
      // start transfer of next byte
      spi_start(chip->spi, chip->spi_buffer, 1);
    }
    else
    {
      //printf("SPI chip deselected\n");
      spi_stop(chip->spi);
      if(chip->is_write){
        write_regs(chip);
      }
    }
  }
}


void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count)
{
  chip_state_t *chip = (chip_state_t *)user_data;
  if (!count)
  {
    // This means that we got here from spi_stop, and no data was received
    return;
  }

  if(count != 1){
    printf("Received unexpected %d bytes\n", count);
    return;
  }

  uint8_t data = buffer[0];
  //printf("Received byte %02x idx %d\n", data, chip->spi_data_from_host_idx);
  
  if(chip->spi_data_from_host_idx < sizeof(chip->spi_data_from_host)){
    chip->spi_data_from_host[chip->spi_data_from_host_idx] = data;
  }
  if(chip->spi_data_from_host_idx==0){
    chip->reg_addr = data & 0x7f;
    chip->is_write = (data & 0x80) == 0;
  } else {
    if( chip->reg_addr != ICM426XX_FIFO_DATA){
      if(chip->spi_data_from_host_idx>1){
        chip->reg_addr++;
      }
    }
  }
  //printf("Reg addr %02x write=%d\n", chip->reg_addr, chip->is_write);
  if(!chip->is_write){
    buffer[0] = read_reg(chip, chip->reg_addr);
    //printf("Sending byte %02x\n", buffer[0]);
  } else {
    buffer[0] = 0;
  }

  chip->spi_data_from_host_idx++;
  if (pin_read(chip->cs_pin) == LOW)
  {
    // Continue with the next character
    spi_start(chip->spi, chip->spi_buffer, 1);
  } else {
    //printf("CS high\n");
  }
}

static uint32_t odr_to_millihz(uint8_t odr)
{
  switch (odr)
  {
  case 1:
    return 32000*1000;
  case 2:
    return 16000*1000;
  case 3:
    return 8000*1000;
  case 4:
    return 4000*1000;
  case 5:
    return 2000*1000;
  case 6:
    return 1000*1000;
  case 7:
    return 200*1000;  
  case 8:
    return 100*1000;
  case 9:
    return 50*1000;
  case 10:
    return 25*1000;
  case 11:
    return 12500;
  case 12:
    return 6250;
  case 13:
    return 3125;
  case 14:
    return 1563;
  case 15:
    return 500*1000;
  default:
    return 0;
  }
}
