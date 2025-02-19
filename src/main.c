#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Registers to simulate
#define ICM426XX_INT_STATUS 0x2D  // 8 bits
#define ICM426XX_FIFO_COUNTH 0x2E   // 16 bits
#define ICM426XX_FIFO_DATA 0x30 // any length
#define ICM426XX_GYRO_CONFIG0 0x4F // 8 bits  - ODR
#define ICM426XX_ACCEL_CONFIG0 0x50 // 8 bits - ODR
#define ICM426XX_PWR_MGMT0 0x4E // 8 bits - enable/disable
#define ICM426XX_FIFO_CONFIG1 0x5F // 8 bits - fifo packet
#define ICM426XX_REG_BANK_SEL 0x76 // 8 bits - select bank
typedef struct
{
  pin_t cs_pin;
  uint32_t spi;
  uint8_t spi_buffer[1];
  uint32_t spi_data_from_host[3]; // reg, data0, data1
  uint32_t spi_data_from_host_idx;
  uint8_t reg_addr;
  uint8_t selected_bank;
  bool is_write;
  uint16_t regs[0x80]; // bank 0 only
  timer_t sample_timer;
  uint32_t accel_amplitude_attr;
  uint32_t gyro_amplitude_attr;
} chip_state_t;

static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_spi_done(void *user_data, uint8_t *buffer, uint32_t count);

static void dump_buffer(uint8_t *buffer, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    printf("%02X ", buffer[i]);
  }
  printf("\n");
}



// static void on_dcdc_timer(void *user_data)
// {
//   chip_state_t *chip = user_data;
//   printf("Enabled DCDC\n");
//   chip->regs[REG_STATUS] &= ~STATUS_SEC_FAIL;
// }

void chip_init(void)
{
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  memset(chip->regs, 0, sizeof(chip->regs));

  chip->cs_pin = pin_init("CS", INPUT_PULLUP);

  // chip->dcdc_timer = timer_init(&(timer_config_t){
  //     .user_data = chip,
  //     .callback = on_dcdc_timer,
  // });

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
  if(chip->selected_bank != 0){
    return 0; // no other banks supported
  }
  if( reg != ICM426XX_FIFO_DATA){
    return chip->regs[reg]; 
  } else {
    printf("Reading FIFO data\n");
    return 0xff; // TODO:
  }
}

static void write_reg(chip_state_t *chip, uint8_t reg, uint8_t value)
{
  if(chip->selected_bank != 0){
    return; // no other banks supported
  }
  if( reg == ICM426XX_REG_BANK_SEL){
    printf("Select bank %d\n", value);
    chip->selected_bank = value;
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
      printf("SPI chip selected\n");
      chip->spi_buffer[0] = ' '; // dummy byte
      chip->spi_data_from_host_idx = 0;
      // start transfer of next byte
      spi_start(chip->spi, chip->spi_buffer, 1);
    }
    else
    {
      printf("SPI chip deselected\n");
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
    dump_buffer(buffer, count);
    return;
  }

  if( chip->spi_data_from_host_idx >= sizeof(chip->spi_data_from_host) ){
    printf("Received too many %d bytes\n", count);
    return;
  }
  uint8_t data = buffer[0];
  printf("Received byte %02x\n", data);
  chip->spi_data_from_host[chip->spi_data_from_host_idx] = data;

  if(chip->spi_data_from_host_idx==0){
    chip->reg_addr = data & 0x7f;
    chip->is_write = (data & 0x80) == 0;
  } else {
    if( chip->reg_addr != ICM426XX_FIFO_DATA){
      chip->reg_addr++;
    }
  }
  if(!chip->is_write){
    buffer[0] = read_reg(chip, chip->reg_addr);
  } else {
    buffer[0] = 0;
  }

  chip->spi_data_from_host_idx++;
  if (pin_read(chip->cs_pin) == LOW)
  {
    // Continue with the next character
    spi_start(chip->spi, chip->spi_buffer, 1);
  }
}

