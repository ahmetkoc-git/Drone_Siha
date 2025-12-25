/*
 * bmp280.c
 *
 *  Created on: Dec 5, 2025
 *      Author: emitn
 */

/*
 *@gurultu cok fazla sensorde yuksekligi tam deger olarak almamiza gerek yok
 * pid icin yol gosterici olmasi yeterli
 * +-0.5 yukseklik degeri noise olarak alinacak.
 * pidde +-0.6 error = 0 error sayilacak
 * joystickden alacagimiz m/sn verilerini yukseklik = +-joystick verileri yapilacak.
 * setpoint degerleri boyle olusacak.
 */

#include "bmp280.h"
#include "math.h"

/* Calibration parameters */
uint16_t dig_t1;
int16_t  dig_t2;
int16_t  dig_t3;
uint16_t dig_p1;
int16_t  dig_p2;
int16_t  dig_p3;
int16_t  dig_p4;
int16_t  dig_p5;
int16_t  dig_p6;
int16_t  dig_p7;
int16_t  dig_p8;
int16_t  dig_p9;

int BMP280_ScanDevıceID(I2C_HandleTypeDef *hi2cx)
{
    for (uint8_t address = 0; address < 255; ++address) {
        if (HAL_I2C_IsDeviceReady(hi2cx, address, 1, 1000) == HAL_OK) {
            return address;
        }
    }
    return -1;
}

BMP280_ReadStatus BMP280_ReadRegisterData(I2C_HandleTypeDef *hi2cx,
                                         uint8_t registerAddress,
                                         uint8_t *dataBuffer,
                                         uint8_t sizeofData)
{
    if (HAL_I2C_Mem_Read(hi2cx,
                          BMP280_DEVICE_ADDRESS,
                          registerAddress,
                          1,
                          dataBuffer,
                          sizeofData,
                          1000) == HAL_OK)
    {
        return READ_BMP_SUCCESS;
    }
    return READ_BMP_FAIL;
}

BMP280_WriteStatus BMP280_WriteRegisterData(I2C_HandleTypeDef *hi2cx,
                                            uint8_t registerAddress,
                                            uint8_t value)
{
    uint8_t data[2] = {0};
    data[0] = registerAddress;
    data[1] = value;

    if (HAL_I2C_Master_Transmit(hi2cx,
                                BMP280_DEVICE_ADDRESS,
                                data,
                                2,
                                1000) == HAL_OK)
    {
        return WRITE_BMP_SUCCESS;
    }
    return WRITE_BMP_FAIL;
}

BMP280_InitStatus BMP280_Init(I2C_HandleTypeDef *hi2cx)
{
    uint8_t dataBuffer = 0;
    uint8_t tempReg = 0xB6; // reset

    BMP280_WriteRegisterData(hi2cx, BMP280_REG_RESET, tempReg);
    BMP280_ReadRegisterData(hi2cx, BMP280_REG_ID, &dataBuffer, 1);

    if (dataBuffer != 0x58) {
        return INIT_BMP_FAIL;
    }

    CTRL_MEAS_REGISTER_T MEAS_REG = {0};
    MEAS_REG.mode   = 3;
    MEAS_REG.osrs_p = 3;
    MEAS_REG.osrs_t = 1;

    tempReg = *((uint8_t*)&MEAS_REG);
    BMP280_WriteRegisterData(hi2cx, BMP280_REG_CTRL_MEAS, tempReg);

    CONFIG_REGISTER_BMP CONFIG_BMP = {0};
    CONFIG_BMP.filter   = 4;
    CONFIG_BMP.reserved = 0;
    CONFIG_BMP.spi3w_en = 0;
    CONFIG_BMP.t_sb     = 0;

    tempReg = *((uint8_t*)&CONFIG_BMP);
    BMP280_WriteRegisterData(hi2cx, BMP280_REG_CONFIG, tempReg);

    return INIT_BMP_SUCCESS;
}

void ReadCalibrationData(I2C_HandleTypeDef *hi2cx)
{
    uint8_t calib[24];
    BMP280_ReadRegisterData(hi2cx, BMP280_REG_CALIB_START, calib, 24);

    dig_t1 = (uint16_t)(calib[0]  | (calib[1]  << 8));
    dig_t2 = (int16_t )(calib[2]  | (calib[3]  << 8));
    dig_t3 = (int16_t )(calib[4]  | (calib[5]  << 8));
    dig_p1 = (uint16_t)(calib[6]  | (calib[7]  << 8));
    dig_p2 = (int16_t )(calib[8]  | (calib[9]  << 8));
    dig_p3 = (int16_t )(calib[10] | (calib[11] << 8));
    dig_p4 = (int16_t )(calib[12] | (calib[13] << 8));
    dig_p5 = (int16_t )(calib[14] | (calib[15] << 8));
    dig_p6 = (int16_t )(calib[16] | (calib[17] << 8));
    dig_p7 = (int16_t )(calib[18] | (calib[19] << 8));
    dig_p8 = (int16_t )(calib[20] | (calib[21] << 8));
    dig_p9 = (int16_t )(calib[22] | (calib[23] << 8));
}

int32_t BMP280_ReadCompensateTemperature(int32_t adc_temp, int32_t *fine_temp)//static
{
    int32_t var1, var2, T;

    var1 = ((((adc_temp >> 3) - ((int32_t)dig_t1 << 1))) * (int32_t)dig_t2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)dig_t1) *
              ((adc_temp >> 4) - (int32_t)dig_t1)) >> 12) *
              (int32_t)dig_t3) >> 14;

    *fine_temp = var1 + var2;
    T = (*fine_temp * 5 + 128) >> 8;

    return T;
}

int32_t BMP280_ReadCompensatePressure(int32_t adc_press, int32_t fine_temp)//static
{
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dig_p6;
    var2 = var2 + ((var1 * (int64_t)dig_p5) << 17);
    var2 = var2 + (((int64_t)dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_p3) >> 8) + ((var1 * (int64_t)dig_p2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dig_p1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dig_p9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dig_p8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_p7 << 4);
    return p;
}

void BMP280_ReadSensorRegister(I2C_HandleTypeDef *hi2cx,int32_t *temperature,uint32_t *pressure)//static
{
    uint8_t data[6] = {0};
    int32_t adcTemperature;
    int32_t adcPressure;
    int32_t fine_temp;

    BMP280_ReadRegisterData(hi2cx, BMP280_REG_PRESS_MSB, data, 6);

    adcPressure    = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    adcTemperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    *temperature = BMP280_ReadCompensateTemperature(adcTemperature, &fine_temp);
    *pressure    = BMP280_ReadCompensatePressure(adcPressure, fine_temp);
}
void BMP280_ReadSensorData(I2C_HandleTypeDef *hi2cx,float *temperature,float *pressure)
{
    int32_t fixed_temperature;
    uint32_t fixed_pressure;

    BMP280_ReadSensorRegister(hi2cx, &fixed_temperature, &fixed_pressure);

    *temperature = (float)fixed_temperature / 100.0f;
    *pressure    = (float)(fixed_pressure / 256.0f) / 100.0f;
}

float PressureToAltitude(float pressure, float p0_hPa)
{
    return 44330.0f * (1.0f - powf(pressure / p0_hPa, 0.1903f));
}
float FilterAltitude(float altitude, float filtered_altitude){
	const float alpha = 0.05;
	filtered_altitude = (alpha * altitude) + ((1 - alpha) * filtered_altitude);
	return filtered_altitude;
}
