# Wokwi ICM426XX Chip

Simulates a ICM426XX IMU chip in Wokwi.

Accel and gyro data is generated as random noise, where the amplitude can be controlled via wokwi controls.

Most configurations are ignored. We assume the following:

* Only packet3 supported
* ODR must be same for accel and gyro (gyro ODR ignored)
* FIFO watermark in records only
* FIFO count & data in little endian only
* timestamp res 1us

## Pin names

| Name | Description              |
| ---- | ------------------------ |
| SCK | SPI Clock             |
| MISO | SPI Master In Slave Out |
| MOSI | SPI Master Out Slave In |
| CS | SPI Chip Select |
| GND | Ground (not used)|
| VCC | Power (not used)|

## Usage

To use this chip in your project, include it as a dependency in your `diagram.json` file:

```json
  "dependencies": {
    "chip-icm426xx": "github:ci4rail/wokwi-chip-icm426xx@1.0.0"
  }
```

Then, add the chip to your circuit by adding a `chip-icm426xx` item to the `parts` section of diagram.json:

```json
  "parts": {
    ...,
    { "type": "chip-icm426xx", "id": "chip1", "attrs": {} }
  },
```


