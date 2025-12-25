/*
 * mpu6050.c
 *
 *  Created on: Sep 7, 2025
 *      Author: emitn
 */
#include "mpu6050.h"
#include "math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG  (180.0f / (float)M_PI)
#define ALPHA 0.98


static float s_gyrobias[3] = {0};
static float s_roll_deg      = 0.0f;  // fused roll açısı (°)
static uint32_t s_t_prev_ms  = 0;     // zaman damgası
static float s_pitch_deg    = 0.0f;

int MPU6050_ScanDeviceID(I2C_HandleTypeDef *hi2cx){
	for (uint8_t address = 0; address < 255; ++address) {
		if (HAL_I2C_IsDeviceReady(hi2cx, address, 1, 1000)==HAL_OK) {
			return address;
		}

	}

	return -1;
}

static MPU6050ReadStatus MPU6050_ReadRegisterData(I2C_HandleTypeDef *hi2cx , uint16_t registerAddress, uint16_t sizeofData, uint8_t *dataBuffer){

	if (HAL_I2C_Mem_Read(hi2cx, MPU6050_DEVICE_ADDRESS, registerAddress, 1, dataBuffer, sizeofData, TIMEOUT) == HAL_OK) {
			return READ_SUCCESS;
		}
		return READ_FAIL;
}

static MPU6050WriteStatus MPU6050_WriteRegisterData(I2C_HandleTypeDef *hi2cx,
		uint16_t registerAddress, uint16_t value) {

	uint8_t data[2] = { 0 };
	data[0] = registerAddress;
	data[1] = value;

	if (HAL_I2C_Master_Transmit(hi2cx, MPU6050_DEVICE_ADDRESS, data,
			sizeof(data), TIMEOUT) == HAL_OK) {
		return WRITE_SUCCESS;
	}

	return WRITE_FAIL;
}

MPU6050InitStatus MPU6050_Init(I2C_HandleTypeDef *hi2cx, uint8_t AFS_SEL, uint8_t FS_SEL) {

	uint8_t dataBuffer = 0;

	MPU6050_ReadRegisterData(hi2cx, MPU6050_REG_WHO_AM_I, 1, &dataBuffer);

	if (dataBuffer != 0x68) {

			return INIT_FAIL;

		}

	uint8_t tempReg = 0;
	PowerManagementRegister_t powerManagement = { 0 };

		powerManagement.ClkSel = 1;
		powerManagement.Temp_Dis = 0;
		powerManagement.Reserved = 0;
		powerManagement.Cycle = 0;
		powerManagement.Sleep = 0;
		powerManagement.Device_Reset = 0;

		tempReg = *((uint8_t*) &powerManagement);

		MPU6050_WriteRegisterData(hi2cx, MPU6050_REG_PWR_MGMT_1, tempReg);

		AccelConfigRegister_t accelConfig = { 0 };

			accelConfig.Reserved = 0;
			accelConfig.AFS_Sel = AFS_SEL;
			accelConfig.ZA_ST = 0;
			accelConfig.YA_ST = 0;
			accelConfig.XA_ST = 0;

			tempReg = *((uint8_t*) &accelConfig);
			MPU6050_WriteRegisterData(hi2cx, MPU6050_REG_ACCEL_CONFIG, tempReg);

	  	GyroConfigRegister_t gyroConfig = { 0 };

				gyroConfig.Reserved = 0;
				gyroConfig.FS_Sel = FS_SEL;
				gyroConfig.ZG_ST = 0;
				gyroConfig.YG_ST = 0;
				gyroConfig.XG_ST = 0;

			tempReg = *((uint8_t*) &gyroConfig);
			MPU6050_WriteRegisterData(hi2cx, MPU6050_REG_GYRO_CONFIG, tempReg);

	  	SMPLRT_DIV_Register_t SmplrtConfig = {0};

			    SmplrtConfig.SMPLRT_DIV = 1;

		    tempReg = *((uint8_t*) &SmplrtConfig);
		    MPU6050_WriteRegisterData(hi2cx, SMPRT_DIV_CONFIG, tempReg);


        Config_DLPF_t configConfig = {0};

                 configConfig.DLPF_CFG = 3;
				 configConfig.EXT_SYNC_SET = 0;

				 tempReg = *((uint8_t*) &configConfig);
		 		 MPU6050_WriteRegisterData(hi2cx, MPU6050_REG_CONFIG, tempReg);
 return INIT_SUCCESS;
}

void MPU6050_getAccelValue(I2C_HandleTypeDef *hi2cx, int16_t *accelData) {

	uint8_t data[6] = { 0 };

	MPU6050_ReadRegisterData(hi2cx, MPU6050_REG_ACCEL_XOUT_H, 6, data);

	accelData[0] = (int16_t) (data[0] << 8 | data[1]);
	accelData[1] = (int16_t) (data[2] << 8 | data[3]);
	accelData[2] = (int16_t) (data[4] << 8 | data[5]);

}

void MPU6050_getGyroValue(I2C_HandleTypeDef *hi2cx, int16_t *gyroData){

	uint8_t data[6] = { 0 };

	MPU6050_ReadRegisterData(hi2cx, MPU6050_REG_GYRO_XOUT_H, 6, data);

	gyroData[0] = (int16_t) (data[0] << 8 | data[1]);
	gyroData[1] = (int16_t) (data[2] << 8 | data[3]);
	gyroData[2] = (int16_t) (data[4] << 8 | data[5]);

}
float MPU6050_getTempValue(I2C_HandleTypeDef *hi2cx, int16_t *tempData){

	uint8_t data[2] = {0};
	float temperature = 0;

	MPU6050_ReadRegisterData(hi2cx, MPU6050_REG_TEMP_OUT_H, 2, data);

	*tempData = (int16_t) (data[0] << 8 | data[1]);

	temperature = (float) *tempData / 340.0 + 36.53;

	return temperature;


}

void MPU6050_getAccelInG(int16_t *accelData, uint8_t AFS_SEL,float *accelDataInG) {

	if (AFS_SEL == 0x00) {
		accelDataInG[0] = (float) accelData[0] / 16384.0;
		accelDataInG[1] = (float) accelData[1] / 16384.0;
		accelDataInG[2] = (float) accelData[2] / 16384.0;
	} else if (AFS_SEL == 0x01) {
		accelDataInG[0] = (float) accelData[0] / 8192.0;
		accelDataInG[1] = (float) accelData[1] / 8192.0;
		accelDataInG[2] = (float) accelData[2] / 8192.0;
	}else if (AFS_SEL == 0x02) {
		accelDataInG[0] = (float) accelData[0] / 4096.0;
		accelDataInG[1] = (float) accelData[1] / 4096.0;
		accelDataInG[2] = (float) accelData[2] / 4096.0;
	}else if (AFS_SEL == 0x03) {
		accelDataInG[0] = (float) accelData[0] / 2048.0;
		accelDataInG[1] = (float) accelData[1] / 2048.0;
		accelDataInG[2] = (float) accelData[2] / 2048.0;
	}
}

void MPU6050_getGyroIns(int16_t *gyroData, uint8_t FS_SEL,float *gyroDataIns) {

	if (FS_SEL == 0x00) {
		gyroDataIns[0] = (float) gyroData[0] / 131.0;
		gyroDataIns[1] = (float) gyroData[1] / 131.0;
		gyroDataIns[2] = (float) gyroData[2] / 131.0;
	} else if (FS_SEL == 0x01) {
		gyroDataIns[0] = (float) gyroData[0] / 65.5;
		gyroDataIns[1] = (float) gyroData[1] / 65.5;
		gyroDataIns[2] = (float) gyroData[2] / 65.5;
	}else if (FS_SEL == 0x02) {
		gyroDataIns[0] = (float) gyroData[0] / 32.8;
		gyroDataIns[1] = (float) gyroData[1] / 32.8;
		gyroDataIns[2] = (float) gyroData[2] / 32.8;
	}else if (FS_SEL == 0x03) {
		gyroDataIns[0] = (float) gyroData[0] / 16.4;
		gyroDataIns[1] = (float) gyroData[1] / 16.4;
		gyroDataIns[2] = (float) gyroData[2] / 16.4;
	}
}

void CalibrateGyroBias(I2C_HandleTypeDef *hi2cx,uint8_t FS_SEL, uint16_t Sample){

	int16_t gyroRaw[3];//ham degerler
	float gyroDps[3]; //fiziksel degerler

	double sumx=0.0,sumy=0.0,sumz=0.0;

	for (int i = 0;  i < Sample; ++i) {
		MPU6050_getGyroValue(hi2cx, gyroRaw);
		MPU6050_getGyroIns(gyroRaw, FS_SEL, gyroDps);
		sumx+=gyroDps[0];
		sumy+=gyroDps[1];
		sumz+=gyroDps[2];

		HAL_Delay(2);
	}
	s_gyrobias[0] =(float)(sumx/Sample);
	s_gyrobias[1] =(float)(sumy/Sample);
	s_gyrobias[2] =(float)(sumz/Sample);
}

void InitRollAndPitchFromAccel(I2C_HandleTypeDef *hi2cx,uint8_t AFS_SEL){

	int16_t accRaw[3];
	float accDps[3];

	MPU6050_getAccelValue(hi2cx, accRaw);
	MPU6050_getAccelInG(accRaw, AFS_SEL, accDps);

	float roll_acc = atan2f(accDps[1],accDps[2]) * RAD2DEG;
	float pitch_acc = atan2f(-accDps[0], sqrtf(accDps[1]*accDps[1] + accDps[2]*accDps[2])) * RAD2DEG;

//global we will use filter
	 s_roll_deg   = roll_acc;
	 s_pitch_deg = pitch_acc;

	 s_t_prev_ms  = HAL_GetTick();
}

void UpdateRollAndPitch(I2C_HandleTypeDef *hi2cx, uint8_t AFS_SEL, uint8_t FS_SEL, float *roll_out, float *pitch_out){

	uint32_t t_now = HAL_GetTick();
	float dt = (t_now - s_t_prev_ms) / 1000.0f;
    if (dt <= 0.0f){
    	dt = 1e-3f;
    }
    s_t_prev_ms = t_now;

    int16_t acc_raw[3], gyro_raw[3];
    float   acc_Dps[3], gyro_dps[3];

    MPU6050_getAccelValue(hi2cx, acc_raw);
    MPU6050_getGyroValue (hi2cx, gyro_raw);

    MPU6050_getAccelInG(acc_raw,  AFS_SEL, acc_Dps);
    MPU6050_getGyroIns (gyro_raw, FS_SEL,  gyro_dps);

    float gx = gyro_dps[0] - s_gyrobias[0];  // gercekten gercek degerler kaymayi engelledik cikarip
    float gy = gyro_dps[1] - s_gyrobias[1];

    float roll_acc  = atan2f(acc_Dps[1], acc_Dps[2]) * RAD2DEG;
    float pitch_acc = atan2f(-acc_Dps[0], sqrtf(acc_Dps[1]*acc_Dps[1] + acc_Dps[2]*acc_Dps[2])) * RAD2DEG;


    float roll_gyro  = s_roll_deg  + gx * dt; // acc tabanli roll pitch hesabi
    float pitch_gyro = s_pitch_deg + gy * dt;




    s_roll_deg  = ALPHA * roll_gyro  + (1.0f - ALPHA) * roll_acc; //complementary filter
    s_pitch_deg = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_acc;

    if (roll_out){
    	*roll_out  = s_roll_deg;
    }
        if (pitch_out){
        *pitch_out = s_pitch_deg;
        }
}











