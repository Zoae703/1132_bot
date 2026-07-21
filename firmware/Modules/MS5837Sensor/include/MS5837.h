/* Blue Robotics MS5837-30BA / 02BA Pressure & Temperature Sensor Library
 * STM32F4 HAL port (C++ class).
 *
 * Original Arduino library:
 *   Authors: Rustom Jehangir, Adam Šimko, Blue Robotics Inc. (MIT License)
 *   https://github.com/bluerobotics/BlueRobotics_MS5837_Library
 *
 * Port notes:
 *   - Replaces Arduino.h / <Wire.h> with <stdint.h> + "stm32f4xx_hal.h".
 *   - I2C peripheral handle is supplied at runtime via init(I2C_HandleTypeDef*).
 *   - All HAL_I2C_* calls use a 100 ms timeout. init() returns false on any
 *     HAL transmission error or CRC failure. read() silently keeps the last
 *     valid sample on transmission failure.
 *   - calculate() and crc4() are pure integer math, kept verbatim from the
 *     original library.
 *
 * License: MIT (inherits from BlueRobotics_MS5837_Library).
 */

#ifndef MS5837_STM32_HAL_H
#define MS5837_STM32_HAL_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

class MS5837 {
public:
	static const float Pa;
	static const float bar;
	static const float mbar;

	static const uint8_t MS5837_30BA;
	static const uint8_t MS5837_02BA;
	static const uint8_t MS5837_UNRECOGNISED;

	MS5837();

	/** Bind to a HAL I2C peripheral and bring up the sensor.
	 *  Returns true on success (PROM read OK and CRC matched), false otherwise.
	 */
	bool init(I2C_HandleTypeDef *hi2c);

	/** Alias for init(), kept for API parity with the original Arduino library. */
	bool begin(I2C_HandleTypeDef *hi2c);

	/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
	 * and MS5837::MS5837_02BA.
	 */
	void setModel(uint8_t model);
	uint8_t getModel();

	/** Provide the density of the working fluid in kg/m^3. Default is for
	 * seawater. Should be 997 for freshwater.
	 */
	void setFluidDensity(float density);

	/** Trigger a blocking pressure + temperature acquisition.
	 *  The call takes up to ~40 ms (two 8192 OSR conversions of ~20 ms each).
	 *  On I2C error, the last valid sample is retained.
	 */
	void read();

	/** Pressure returned in mbar or mbar*conversion rate.
	 */
	float pressure(float conversion = 1.0f);

	/** Temperature returned in deg C.
	 */
	float temperature();

	/** Depth returned in meters (valid for operation in incompressible
	 *  liquids only. Uses density that is set for fresh or seawater.
	 */
	float depth();

	/** Altitude returned in meters (valid for operation in air only).
	 */
	float altitude();

private:
	/* HAL I2C peripheral handle bound by init(). */
	I2C_HandleTypeDef *_hi2c;

	uint16_t C[8];
	uint32_t D1_pres, D2_temp;
	int32_t TEMP;
	int32_t P;
	uint8_t _model;

	float fluidDensity;

	/** Performs calculations per the sensor data sheet for conversion and
	 *  second order compensation.
	 */
	void calculate();

	uint8_t crc4(uint16_t n_prom[]);
};

#endif /* MS5837_STM32_HAL_H */
