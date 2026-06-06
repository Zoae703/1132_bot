#include "IST8310driver.h"

// --- Internal I2C helpers (merged from middleware) ---
static I2C_HandleTypeDef* i2c_handle = nullptr;

static uint8_t i2c_read_reg(uint8_t reg) {
    uint8_t val = 0;
    HAL_I2C_Mem_Read(i2c_handle, IST8310_IIC_ADDRESS, reg,
                     I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return val;
}

static void i2c_write_reg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(i2c_handle, IST8310_IIC_ADDRESS, reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static void i2c_read_multi(uint8_t reg, uint8_t* buf, uint8_t len) {
    HAL_I2C_Mem_Read(i2c_handle, IST8310_IIC_ADDRESS, reg,
                     I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

// --- RSTN control ---
static void rst_low()  { HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_RESET); }
static void rst_high() { HAL_GPIO_WritePin(IST8310_RSTN_GPIO_Port, IST8310_RSTN_Pin, GPIO_PIN_SET); }

// --- Init sequence (from reference ist8310driver.cpp) ---
uint8_t ist8310_init(I2C_HandleTypeDef* hi2c) {
    i2c_handle = hi2c;

    static const uint8_t cfg[4][2] = {
        {0x0B, 0x08},  // CTRL_REG3: enable interrupt
        {0x41, 0x09},  // CTRL_REG1: 200Hz, continuous
        {0x42, 0xC0},  // CTRL_REG2: ±1600µT range
        {0x0A, 0x0B}   // STATR_REG: clear status
    };

    // Hardware reset
    rst_low();
    HAL_Delay(50);
    rst_high();
    HAL_Delay(50);

    // WHO_AM_I check
    if (i2c_read_reg(0x00) != 0x10)
        return 0x40;  // IST8310_NO_SENSOR

    HAL_Delay(1);

    for (int i = 0; i < 4; i++) {
        i2c_write_reg(cfg[i][0], cfg[i][1]);
        HAL_Delay(1);
        if (i2c_read_reg(cfg[i][0]) != cfg[i][1])
            return i + 1;
    }
    return 0;
}

void ist8310_read_mag(I2C_HandleTypeDef* hi2c, float mag[3]) {
    i2c_handle = hi2c;
    uint8_t buf[6];
    i2c_read_multi(0x03, buf, 6);
    mag[0] = MAG_SEN * (int16_t)((buf[1] << 8) | buf[0]);
    mag[1] = MAG_SEN * (int16_t)((buf[3] << 8) | buf[2]);
    mag[2] = MAG_SEN * (int16_t)((buf[5] << 8) | buf[4]);
}
