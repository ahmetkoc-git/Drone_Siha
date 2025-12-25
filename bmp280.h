/*
 * bmp280.h
 *
 *  Created on: Dec 5, 2025
 *      Author: emitn
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"

#define BMP280_DEVICE_ADDRESS  0xEC

#define BMP280_REG_ID              0xD0
#define BMP280_REG_RESET           0xE0
#define BMP280_REG_STATUS          0xF3
#define BMP280_REG_CTRL_MEAS       0xF4
#define BMP280_REG_CONFIG          0xF5
#define BMP280_REG_PRESS_MSB       0xF7
#define BMP280_REG_PRESS_LSB       0xF8
#define BMP280_REG_PRESS_XLSB      0xF9
#define BMP280_REG_TEMP_MSB        0xFA
#define BMP280_REG_TEMP_LSB        0xFB
#define BMP280_REG_TEMP_XLSB       0xFC
#define BMP280_REG_CALIB_START     0x88
#define BMP280_REG_CALIB_END       0xA1
#define BMP280_CALIB_LENGTH        24

typedef enum{
    READ_BMP_SUCCESS = 1,
    READ_BMP_FAIL    = 0
} BMP280_ReadStatus;

typedef enum{
    WRITE_BMP_SUCCESS = 1,
    WRITE_BMP_FAIL    = 0
} BMP280_WriteStatus;

typedef enum{
    INIT_BMP_SUCCESS = 1,
    INIT_BMP_FAIL    = 0
} BMP280_InitStatus;

typedef struct{
    uint8_t mode   : 2;
    uint8_t osrs_p : 3;
    uint8_t osrs_t : 3;
} CTRL_MEAS_REGISTER_T;

typedef struct{
    uint8_t spi3w_en : 1;
    uint8_t reserved : 1;
    uint8_t filter   : 3;
    uint8_t t_sb     : 3;
} CONFIG_REGISTER_BMP;

/* Function prototypes */
int BMP280_ScanDevıceID(I2C_HandleTypeDef *hi2cx);
BMP280_WriteStatus BMP280_WriteRegisterData(I2C_HandleTypeDef *hi2cx, uint8_t registerAddress, uint8_t value);
BMP280_ReadStatus BMP280_ReadRegisterData(I2C_HandleTypeDef *hi2cx, uint8_t registerAddress, uint8_t *dataBuffer, uint8_t sizeofData);
BMP280_InitStatus BMP280_Init(I2C_HandleTypeDef *hi2cx);
void ReadCalibrationData(I2C_HandleTypeDef *hi2cx);
int32_t BMP280_ReadCompensateTemperature(int32_t adc_temp, int32_t *fine_temp);
int32_t BMP280_ReadCompensatePressure(int32_t adc_press, int32_t fine_temp);
void BMP280_ReadSensorRegister(I2C_HandleTypeDef *hi2cx, int32_t *temperature, uint32_t *pressure);
void BMP280_ReadSensorData(I2C_HandleTypeDef *hi2cx, float *temperature, float *pressure);
float PressureToAltitude(float pressure, float p0_hPa);
float FilterAltitude(float altitude, float filtered_altitude);
#endif /* INC_BMP280_H_ */
