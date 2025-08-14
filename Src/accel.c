#include "usart.h"
#include "i2c.h"
#include "bno055.h"
#include "accel.h"

uint8_t GPwrMode 	= NormalG;
uint8_t Gscale 		= GFS_2000DPS;
//uint8_t Godr	 	= GODR_250Hz;
uint8_t Gbw 		= GBW_230Hz;

uint8_t Ascale 		= AFS_16G;
uint8_t APwrMode 	= NormalA;
uint8_t Abw 		= ABW_250Hz;

//uint8_t Mscale 	= MFS_4Gauss;
uint8_t MOpMode 	= EnhancedRegular;
uint8_t MPwrMode 	= Normal;
uint8_t Modr 		= MODR_30Hz;

uint8_t PWRMode 	= Normalpwr;
uint8_t OPRMode 	= ACCGYRO;

uint8_t status;
float aRes, gRes, mRes;

uint8_t cal_sys 	= 0;
uint8_t cal_gyro 	= 0;
uint8_t cal_acc 	= 0;
uint8_t cal_mag 	= 0;
uint8_t cal_imu 	= 0;

const uint8_t num_of_bytes_read = 18;

const char read_devid[] 	= {START_BYTE, REG_READ, BNO055_CHIP_ID, 0x01};
const char read_calib[2] 	= {REG_READ, BNO055_CALIB_STAT};
const char reset_sensor[3]	= {REG_WRITE, BNO055_SYS_TRIGGER, 0x01 << 5};
uint8_t get_readings[1] 	= {BNO055_ACC_DATA_X_LSB};


void BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device) {
	uint8_t opr_config_mode[2] = {BNO055_OPR_MODE, CONFIGMODE};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_config_mode, sizeof(opr_config_mode), 10);
	HAL_Delay(10);

	uint8_t conf_page1[2] = {BNO055_PAGE_ID, 0x01};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page1, sizeof(conf_page1), 10);
	HAL_Delay(10);

	uint8_t conf_acc[2] = {BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_acc, sizeof(conf_acc), 10);
	HAL_Delay(10);

	uint8_t conf_gyro[2] = {BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro, sizeof(conf_gyro), 10);
	HAL_Delay(10);

	uint8_t conf_gyro_pwr[2] = {BNO055_GYRO_CONFIG_1, GPwrMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro_pwr, sizeof(conf_gyro_pwr), 10);
	HAL_Delay(10);

	uint8_t conf_mag_pwr[4] = {REG_WRITE, BNO055_MAG_CONFIG, 0x01, MPwrMode << 5 | MOpMode << 3 | Modr};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_mag_pwr, sizeof(conf_mag_pwr), 10);
	HAL_Delay(10);

	uint8_t conf_page0[2] = {BNO055_PAGE_ID, 0x00};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page0, sizeof(conf_page0), 10);
	HAL_Delay(10);

	uint8_t pwr_pwrmode[2] = {BNO055_PWR_MODE, PWRMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, pwr_pwrmode, sizeof(pwr_pwrmode), 10);
	HAL_Delay(10);

	uint8_t opr_oprmode[2] = {BNO055_OPR_MODE, OPRMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_oprmode, sizeof(opr_oprmode), 10);
	HAL_Delay(50);
}

uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	return HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES, 100);
}


uint8_t GetGyroData(I2C_HandleTypeDef* hi2c_device, uint8_t* gyro_buffer) {
	return HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_GYR_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, gyro_buffer, GYRO_NUMBER_OF_BYTES, 100);
}


void ReadGyroXYZ(int16_t *destination, uint8_t* rawData) {
	destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];
	destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
}

uint8_t GetAccelChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id) {
	return HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, chip_id, 1, 100);
}

uint8_t GetAccelTemp(I2C_HandleTypeDef* hi2c_device) {
	uint8_t temp;
	HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_TEMP, I2C_MEMADD_SIZE_8BIT, &temp, 1);
	while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return temp;
}

uint8_t BNO055_Get_Calibration(I2C_HandleTypeDef* hi2c_device) {
	uint8_t calibration;
	HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT, &calibration, 1);
	while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY);
	return calibration;
}

void BNO055_Calc_Calibration(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag) {
	*cal_system = (calibration >> 6) & 0x03;
	*cal_gyro 	= (calibration >> 4) & 0x03;
	*cal_acc 	= (calibration >> 2) & 0x03;
	*cal_mag 	= (calibration) & 0x03;
}
