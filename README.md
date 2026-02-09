# 7Semi MMC5983MA 3-axis magnetometer sensor

Arduino library for the **MMC5983MA** 3-axis magnetometer with optional temperature readout and compass helpers.

- Supports **I2C** 
- Reads **X/Y/Z magnetometer** (18-bit)
- Reads **temperature**
- Supports **single-shot** and **continuous** measurement
- Supports **SET / RESET** and **Auto Set/Reset**
- Provides **2D heading** and **tilt-compensated heading** (requires external roll/pitch)

## Hardware Wiring

### I2C 

| MMC5983MA | Arduino UNO/Nano | ESP32 (typical) |
|----------|-------------------|-----------------|
| VCC      | 3.3V              | 3.3V            |
| GND      | GND               | GND             |
| SDA      | A4 (SDA)          | GPIO21 (SDA)    |
| SCL      | A5 (SCL)          | GPIO22 (SCL)    |

Notes:
- Many breakout boards already include SDA/SCL pull-ups.
- If your board has no pull-ups, add ~4.7k–10k pull-ups to **3.3V**.


## Installation

1. Download or clone this repository into your Arduino `libraries` folder.
2. Restart Arduino IDE.
3. Examples will appear under: `File > Examples > 7Semi_MMC5983MA`

## Library Reference (Quick Description)

- `beginI2C(wire, address, clockHz, i2cSDA, i2cSCL)`  
  Starts the sensor in I2C mode and applies default configuration. Returns `false` if the device is not detected.

- `begin()`  
  Common init routine used internally after selecting bus. Checks product ID, performs reset, and writes default settings.

- `getProductID(id)`  
  Reads and validates the product ID register. Returns `false` if the ID does not match expected.

- `readMagnetometer(x_mG, y_mG, z_mG)`  
  Reads magnetometer in milli-Gauss (mG) with offset/scale applied and optional smoothing/calibration.

- `readMagnetometerRaw(x, y, z)`  
  Reads raw signed magnetometer counts (centered around zero) without scaling.

- `readTemperature(tempC)`  
  Triggers a temperature measurement and returns temperature in °C. Returns `false` on timeout/read failure.

- `initMagMeasurement()`  
  Triggers a single-shot magnetometer conversion. Useful if you want manual timing.

- `initTempMeasurement()`  
  Triggers a single-shot temperature conversion. Useful if you want manual timing.

- `isMagReady(ready)`  
  Reads the status bit indicating magnetometer measurement is complete.

- `isTempReady(ready)`  
  Reads the status bit indicating temperature measurement is complete.

- `enableContinuousMode(enable, freq)`  
  Enables/disables continuous conversion and sets the output update rate using `Frequency`.

- `setBandwidth(bw)`  
  Sets the bandwidth option using `Bandwidth` to trade off response speed vs noise.

- `sensorReset()`  
  Performs a software reset and returns the sensor to a known state.

- `enableAutoSetReset(enable, as)`  
  Enables/disables Auto Set/Reset and selects its periodicity using `autoSet`.

- `opSet()`  
  Performs a manual SET operation (self-clearing control bit).

- `opReset()`  
  Performs a manual RESET operation (self-clearing control bit).

- `enableSmoothing(enable)`  
  Enables/disables advanced smoothing for magnetometer axis output.

- `setSmoothingResponse(response)`  
  Adjusts smoothing behavior (lower = smoother output, higher = faster response).

- `setAxisMap(map)`  
  Sets axis mapping for heading calculations to match board orientation.

- `setOffset(x_off_mG, y_off_mG, z_off_mG)`  
  Sets hard-iron offsets in mG for X/Y/Z.

- `setScale(x_scale, y_scale, z_scale)`  
  Sets soft-iron scale factors (unitless). Use 1.0 for no scaling.

- `calibrationReset2D()`  
  Clears 2D calibration state so you can start collecting new min/max values.

- `update2DCalibrartion()`  
  Updates 2D calibration min/max values using current readings (call repeatedly while rotating the device).

- `finish2DCalibration()`  
  Computes final offset/scale from collected values and marks 2D calibration as ready.

- `calibrationReady2D()`  
  Returns `true` when 2D calibration has been finalized.

- `enableAutoCal3D(enable)`  
  Enables/disables best-effort background calibration over time while the device is moved naturally.

- `getHeading2D(declination_deg)`  
  Returns 2D heading in degrees (0–360). Works best when the device is roughly level.

- `getHeadingTilt(roll_deg, pitch_deg, declination_deg)`  
  Returns tilt-compensated heading in degrees using external roll/pitch angles.


## Quick Start (I2C)

```cpp
#include <Wire.h>
#include "7Semi_MMC5983MA.h"

MMC5983MA_7Semi mag;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!mag.beginI2C(Wire))
  {
    Serial.println("MMC5983MA not found");
    while (1) delay(10);
  }

  mag.enableAutoSetReset(true, MMC5983MA_7Semi::MES_100);

  mag.enableSmoothing(true);
  mag.setSmoothingResponse(1.0f);

  mag.setFilterAlpha(0.25f);
}

void loop()
{
  float x, y, z;

  if (mag.readMagnetometer(x, y, z))
  {
    Serial.print("mG: ");
    Serial.print(x, 2); Serial.print(", ");
    Serial.print(y, 2); Serial.print(", ");
    Serial.println(z, 2);
  }

  delay(100);
}
