/**
 * 7Semi MMC5983MA Example
 * 2D Heading + Tilt Heading with 360° Calibration
 *
 * Wiring
 * VCC -> 3.3V
 * GND -> GND
 * SDA -> SDA
 * SCL -> SCL
 */

#include <7Semi_MMC5983MA.h>

MMC5983MA_7Semi mag;

/* Local magnetic declination (degrees) */
static const float DECLINATION_DEG = 0.0f;

/* Hard-iron offsets (computed by calibration) */
float magOffsetX = 0.0f;
float magOffsetY = 0.0f;
float magOffsetZ = 0.0f;

/* -------------------------------------------------- */
/* 360° Magnetometer Calibration (hard-iron only)     */
/* -------------------------------------------------- */
void calibrateMagnetometer(uint32_t duration_ms)
{
  float mx, my, mz;

  float minX =  1e6, minY =  1e6, minZ =  1e6;
  float maxX = -1e6, maxY = -1e6, maxZ = -1e6;

  Serial.println();
  Serial.println("=== Magnetometer Calibration ===");
  Serial.println("Rotate sensor slowly in ALL directions");
  Serial.println("Running...");
  Serial.println();

  uint32_t start = millis();
  while (millis() - start < duration_ms)
  {
    mag.readMagnetometer(mx, my, mz);

    minX = min(minX, mx);  maxX = max(maxX, mx);
    minY = min(minY, my);  maxY = max(maxY, my);
    minZ = min(minZ, mz);  maxZ = max(maxZ, mz);

    delay(20);
  }

  magOffsetX = (maxX + minX) * 0.5f;
  magOffsetY = (maxY + minY) * 0.5f;
  magOffsetZ = (maxZ + minZ) * 0.5f;

  Serial.println("Calibration complete");
  Serial.print("Offsets -> X: "); Serial.print(magOffsetX);
  Serial.print("  Y: "); Serial.print(magOffsetY);
  Serial.print("  Z: "); Serial.println(magOffsetZ);
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  Wire.begin();

  if (!mag.beginI2C(Wire))
  {
    Serial.println("MMC5983MA not found");
    while (1) delay(10);
  }

  mag.enableAutoSetReset(true, MMC5983MA_7Semi::MES_100);
  mag.setFilterAlpha(0.25f);

  Serial.println("Heading demo started");

  /* Run 360° calibration (rotate board now) */
  calibrateMagnetometer(15000);   // 15 seconds
}

void loop()
{
  float mx, my, mz;
  mag.readMagnetometer(mx, my, mz);

  /* Apply calibration offsets */
  mx -= magOffsetX;
  my -= magOffsetY;
  mz -= magOffsetZ;

  /* -------- 2D Heading (level) -------- */
  float h2d = atan2(my, mx) * RAD_TO_DEG + DECLINATION_DEG;
  if (h2d < 0)   h2d += 360;
  if (h2d >= 360) h2d -= 360;

  /* -------- Tilt Heading (placeholder IMU values) -------- */
  float roll_deg  = 0.0f;   // replace with IMU fusion output
  float pitch_deg = 0.0f;

  float roll  = roll_deg  * DEG_TO_RAD;
  float pitch = pitch_deg * DEG_TO_RAD;

  float Xh = mx * cos(pitch) + mz * sin(pitch);
  float Yh = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

  float htilt = atan2(Yh, Xh) * RAD_TO_DEG + DECLINATION_DEG;
  if (htilt < 0)   htilt += 360;
  if (htilt >= 360) htilt -= 360;

  Serial.print("H2D: ");
  Serial.print(h2d, 1);
  Serial.print("  HTilt: ");
  Serial.println(htilt, 1);

  delay(100);
}
