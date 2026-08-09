#include "icm42688.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "main.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi3;

// SPI操作宏
#define ICM42688_CS_LOW()     HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_RESET)
#define ICM42688_CS_HIGH()    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_SET)

static float accSensitivity  = 0.244f;
static float gyroSensitivity = 32.8f;
#define ICM42688DelayMs(_nms)  HAL_Delay(_nms)

static uint8_t icm42688_spi_read_write(uint8_t tx_data){
    uint8_t rx_data = 0x00;
    HAL_SPI_TransmitReceive(&hspi3, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t regval = 0;

    ICM42688_CS_LOW();      //选中icm42688设备
    icm42688_spi_read_write(reg | 0x80);    //发送地址
    regval = icm42688_spi_read_write(0x00); //读取数据
    ICM42688_CS_HIGH();
    return regval;
}

static void icm42688_read_regs(uint8_t reg, uint8_t* buf, uint16_t len)
{
    // 临时修复：使用单字节读取代替多字节读取
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = icm42688_read_reg(reg + i);
    }
}

static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
    ICM42688_CS_LOW();
    icm42688_spi_read_write(reg & 0x7F);
    icm42688_spi_read_write(value);
    ICM42688_CS_HIGH();
    return 0;
}

float bsp_Icm42688GetAres(uint8_t Ascale)
{
    switch (Ascale) {
        case AFS_2G:  accSensitivity = 2000.0f / 32768.0f; break;
        case AFS_4G:  accSensitivity = 4000.0f / 32768.0f; break;
        case AFS_8G:  accSensitivity = 8000.0f / 32768.0f; break;
        case AFS_16G: accSensitivity = 16000.0f / 32768.0f; break;
    }
    return accSensitivity;
}

float bsp_Icm42688GetGres(uint8_t Gscale)
{
    switch (Gscale) {
        case GFS_250DPS:    gyroSensitivity = 250.0f / 32768.0f; break;
        case GFS_500DPS:    gyroSensitivity = 500.0f / 32768.0f; break;
        case GFS_1000DPS:   gyroSensitivity = 1000.0f / 32768.0f; break;
        case GFS_2000DPS:   gyroSensitivity = 2000.0f / 32768.0f; break;
    }
    return gyroSensitivity;
}

int8_t bsp_Icm42688Init(void)
{
    uint8_t reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);

    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01);
    ICM42688DelayMs(100);

    if (reg_val == ICM42688_ID) {
        bsp_Icm42688GetAres(AFS_2G);
        reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG0);
        reg_val &= ~((0x3 << 5) | 0x0F);
        reg_val |= (AFS_2G << 5);
        reg_val |= (AODR_200Hz);
        icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

        bsp_Icm42688GetGres(GFS_1000DPS);
        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG0);
        reg_val &= ~((0x3 << 5) | 0x0F);
        reg_val |= (GFS_1000DPS << 5);
        reg_val |= (GODR_200Hz);
        icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x02);
        reg_val = icm42688_read_reg(ICM42688_ACCEL_CONFIG_STATIC3);
        reg_val = (reg_val & ~0x70) | (AAVG_4X << 4);
        reg_val = (reg_val & ~0x0F) | AAF_1_16_ODR;
        icm42688_write_reg(ICM42688_ACCEL_CONFIG_STATIC3, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x01);
        reg_val = icm42688_read_reg(ICM42688_GYRO_CONFIG_STATIC3);
        reg_val = (reg_val & ~0x70) | (GAVG_4X << 4);
        reg_val = (reg_val & ~0x0F) | GF_1_16_ODR;
        icm42688_write_reg(ICM42688_GYRO_CONFIG_STATIC3, reg_val);

        icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00);
        reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0);
        reg_val &= ~(1 << 5);
        reg_val |= (3 << 2);
        reg_val |= 3;
        icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
        ICM42688DelayMs(1);

        return 0;
    }

    return -1;
}

int8_t bsp_IcmGetTemperature(int16_t* pTemp)
{
    uint8_t buffer[2];
    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2);
    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48f + 25.0f);
    return 0;
}

int8_t bsp_IcmGetAccelerometer(icm42688RawData_t* accData, float* ax, float* ay, float* az)
{
    uint8_t buffer[6];
    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6);

    int16_t raw_x = ((uint16_t)buffer[0] << 8) | buffer[1];
    int16_t raw_y = ((uint16_t)buffer[2] << 8) | buffer[3];
    int16_t raw_z = ((uint16_t)buffer[4] << 8) | buffer[5];

    accData->x = raw_x;
    accData->y = raw_y;
    accData->z = raw_z;

    *ax = raw_x * accSensitivity / 1000.0f;
    *ay = raw_y * accSensitivity / 1000.0f;
    *az = raw_z * accSensitivity / 1000.0f;

    return 0;
}

int8_t bsp_IcmGetGyroscope(icm42688RawData_t* gyroData, float* gx, float* gy, float* gz)
{
    uint8_t buffer[6];
    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6);

    int16_t raw_x = ((uint16_t)buffer[0] << 8) | buffer[1];
    int16_t raw_y = ((uint16_t)buffer[2] << 8) | buffer[3];
    int16_t raw_z = ((uint16_t)buffer[4] << 8) | buffer[5];

    gyroData->x = raw_x;
    gyroData->y = raw_y;
    gyroData->z = raw_z;

    *gx = raw_x * gyroSensitivity;
    *gy = raw_y * gyroSensitivity;
    *gz = raw_z * gyroSensitivity;

    return 0;
}

int8_t bsp_IcmGetAllSensorData(icm42688SensorData_t* data)
{
    uint8_t buffer[12];

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    int16_t raw_ax = ((uint16_t)buffer[0] << 8) | buffer[1];
    int16_t raw_ay = ((uint16_t)buffer[2] << 8) | buffer[3];
    int16_t raw_az = ((uint16_t)buffer[4] << 8) | buffer[5];
    int16_t raw_gx = ((uint16_t)buffer[6] << 8) | buffer[7];
    int16_t raw_gy = ((uint16_t)buffer[8] << 8) | buffer[9];
    int16_t raw_gz = ((uint16_t)buffer[10] << 8) | buffer[11];

    data->ax = raw_ax * accSensitivity / 1000.0f;
    data->ay = raw_ay * accSensitivity / 1000.0f;
    data->az = raw_az * accSensitivity / 1000.0f;

    data->gx = raw_gx * gyroSensitivity;
    data->gy = raw_gy * gyroSensitivity;
    data->gz = raw_gz * gyroSensitivity;

    return 0;
}
