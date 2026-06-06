/**
 * BMI088 driver — blocking SPI implementation.
 * Simplified from RoboMaster Lower_Level_Controller:
 *   - No DMA
 *   - No IST8310 magnetometer
 *   - No ellipsoid calibration
 *   - No temperature PID
 */

#include "BMI088driver.h"
#include "BMI088reg.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* ---------- Platform abstraction (inline) ---------- */

// Default CS pins for RoboMaster C-Board: PA4 (accel), PB0 (gyro)
#ifndef CS1_ACCEL_GPIO_Port
#define CS1_ACCEL_GPIO_Port GPIOA
#endif
#ifndef CS1_ACCEL_Pin
#define CS1_ACCEL_Pin GPIO_PIN_4
#endif
#ifndef CS1_GYRO_GPIO_Port
#define CS1_GYRO_GPIO_Port GPIOB
#endif
#ifndef CS1_GYRO_Pin
#define CS1_GYRO_Pin GPIO_PIN_0
#endif

static inline void BMI088_ACCEL_NS_L(void) { HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET); }
static inline void BMI088_ACCEL_NS_H(void) { HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET); }
static inline void BMI088_GYRO_NS_L(void)  { HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET); }
static inline void BMI088_GYRO_NS_H(void)  { HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET); }

static inline uint8_t BMI088_read_write_byte(uint8_t txdata) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx, 1, 1000);
    return rx;
}

static inline void BMI088_delay_ms(uint16_t ms) { HAL_Delay(ms); }
static inline void BMI088_delay_us(uint16_t us) {
    for (uint16_t i = 0; i < us; i++) { volatile int a = 10; while (a--); }
}

/* ---------- Low-level SPI register access ---------- */

static void BMI088_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *data) {
    BMI088_read_write_byte(reg | 0x80);
    *data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
    BMI088_read_write_byte(reg | 0x80);
    while (len--) { *buf++ = BMI088_read_write_byte(0x55); }
}

/* ---------- Accel macros ---------- */

#define BMI088_accel_write_single_reg(reg, data) \
    { BMI088_ACCEL_NS_L(); BMI088_write_single_reg((reg), (data)); BMI088_ACCEL_NS_H(); }

#define BMI088_accel_read_single_reg(reg, data) \
    { BMI088_ACCEL_NS_L(); BMI088_read_write_byte((reg) | 0x80); BMI088_read_write_byte(0x55); \
      (data) = BMI088_read_write_byte(0x55); BMI088_ACCEL_NS_H(); }

#define BMI088_accel_read_muli_reg(reg, data, len) \
    { BMI088_ACCEL_NS_L(); BMI088_read_write_byte((reg) | 0x80); \
      BMI088_read_muli_reg(reg, data, len); BMI088_ACCEL_NS_H(); }

/* ---------- Gyro macros ---------- */

#define BMI088_gyro_write_single_reg(reg, data) \
    { BMI088_GYRO_NS_L(); BMI088_write_single_reg((reg), (data)); BMI088_GYRO_NS_H(); }

#define BMI088_gyro_read_single_reg(reg, data) \
    { BMI088_GYRO_NS_L(); BMI088_read_single_reg((reg), &(data)); BMI088_GYRO_NS_H(); }

#define BMI088_gyro_read_muli_reg(reg, data, len) \
    { BMI088_GYRO_NS_L(); BMI088_read_muli_reg((reg), (data), (len)); BMI088_GYRO_NS_H(); }

/* ---------- Sensitivity (compile-time selected) ---------- */

static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
static float BMI088_GYRO_SEN  = BMI088_GYRO_2000_SEN;

/* ---------- Register config tables ---------- */

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

/* ---------- Self-test ---------- */

static uint8_t bmi088_accel_self_test(void) {
    int16_t self_test_accel[2][3];
    uint8_t buf[6] = {0};
    volatile uint8_t res = 0;

    static const uint8_t st_regs[6][3] = {
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
    };

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

    for (uint8_t i = 0; i < 4; i++) {
        BMI088_accel_write_single_reg(st_regs[i][0], st_regs[i][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_accel_read_single_reg(st_regs[i][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != st_regs[i][1]) return st_regs[i][2];
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    }

    for (uint8_t i = 0; i < 2; i++) {
        BMI088_accel_write_single_reg(st_regs[i + 4][0], st_regs[i + 4][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_accel_read_single_reg(st_regs[i + 4][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != st_regs[i + 4][1]) return st_regs[i + 4][2];
        BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
        self_test_accel[i][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[i][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[i][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    if (res != BMI088_ACC_SELF_TEST_OFF) return BMI088_ACC_SELF_TEST_ERROR;

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) ||
        (self_test_accel[0][1] - self_test_accel[1][1] < 1365) ||
        (self_test_accel[0][2] - self_test_accel[1][2] < 680))
        return BMI088_SELF_TEST_ACCEL_ERROR;

    return BMI088_NO_ERROR;
}

static uint8_t bmi088_gyro_self_test(void) {
    uint8_t res = 0;
    uint8_t retry = 0;

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    do {
        BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10) return BMI088_SELF_TEST_GYRO_ERROR;
    if (res & BMI088_GYRO_BIST_FAIL) return BMI088_SELF_TEST_GYRO_ERROR;

    return BMI088_NO_ERROR;
}

/* ---------- Init ---------- */

static uint8_t bmi088_accel_init(void) {
    volatile uint8_t res = 0;

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

    for (uint8_t i = 0; i < BMI088_WRITE_ACCEL_REG_NUM; i++) {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[i][0], write_BMI088_accel_reg_data_error[i][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[i][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != write_BMI088_accel_reg_data_error[i][1])
            return write_BMI088_accel_reg_data_error[i][2];
    }
    return BMI088_NO_ERROR;
}

static uint8_t bmi088_gyro_init(void) {
    uint8_t res = 0;

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_GYRO_CHIP_ID_VALUE) return BMI088_NO_SENSOR;

    for (uint8_t i = 0; i < BMI088_WRITE_GYRO_REG_NUM; i++) {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[i][0], write_BMI088_gyro_reg_data_error[i][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[i][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != write_BMI088_gyro_reg_data_error[i][1])
            return write_BMI088_gyro_reg_data_error[i][2];
    }
    return BMI088_NO_ERROR;
}

uint8_t BMI088_init(void) {
    uint8_t error = BMI088_NO_ERROR;

    if (bmi088_accel_self_test() != BMI088_NO_ERROR)
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    else
        error |= bmi088_accel_init();

    if (bmi088_gyro_self_test() != BMI088_NO_ERROR)
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    else
        error |= bmi088_gyro_init();

    return error;
}

/* ---------- Read ---------- */

void BMI088_read(float gyro[3], float accel[3], float *temperate) {
    uint8_t buf[8] = {0};
    int16_t raw;

    // Read accelerometer
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
    raw = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = raw * BMI088_ACCEL_SEN;
    raw = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = raw * BMI088_ACCEL_SEN;
    raw = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = raw * BMI088_ACCEL_SEN;

    // Read gyroscope (read from CHIP_ID to get 8 bytes: id + reserved + 6 data)
    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        raw = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = raw * BMI088_GYRO_SEN;
        raw = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = raw * BMI088_GYRO_SEN;
        raw = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = raw * BMI088_GYRO_SEN;
    }

    // Read temperature
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    raw = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;
    *temperate = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}
