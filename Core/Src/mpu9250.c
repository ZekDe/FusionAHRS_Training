#include "stm32f4xx_hal.h"
#include "mpu9250.h"
#include "math.h"

float getMagRes(void)
{
	return 0.6F; //	uT/LSB
}
//***********************************************************************************************//
float getGyroRes(gyroScale_t gyroScale)
{
	float sc = -1.0F;

	switch (gyroScale)
	{
	case GFS_250DPS:
		sc = 0.00762939F;
		break;
	case GFS_500DPS:
		sc = 0.01526718F;
		break;
	case GFS_1000DPS:
		sc = 0.030487805F;
		break;
	case GFS_2000DPS:
		sc = 0.060975610F;
		break;
	default:
		break;
	}

	return sc; // dps/LSB
}
//***********************************************************************************************//
float getAccRes(accScale_t accScale)
{
	float sc;

	switch (accScale)
	{
	case AFS_2G:
		sc = 0.00006104F;
		break;
	case AFS_4G:
		sc = 0.00012207F;
		break;
	case AFS_8G:
		sc = 0.00024414F;
		break;
	case AFS_16G:
		sc = 0.00048828F;
		break;
	default:
		break;
	}
	return sc; // g/LSB
}
//***********************************************************************************************//
_Bool MPU9250_isDataReady(I2C_HandleTypeDef *hi2c)
{
	uint8_t status;
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, INT_STATUS, I2C_MEMADD_SIZE_8BIT,
			&status, 1, 100);

	return ((status & 0x01) == 0x01);
}
//***********************************************************************************************//
void getAccRaw(I2C_HandleTypeDef *hi2c, int16_t *accRaw)
{
	uint8_t out[6] =
	{ 0 };
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
			out, 6, 100);

	accRaw[0] = (int16_t) (((int16_t) out[0] << 8) | out[1]);
	accRaw[1] = (int16_t) (((int16_t) out[2] << 8) | out[3]);
	accRaw[2] = (int16_t) (((int16_t) out[4] << 8) | out[5]);
}
//***********************************************************************************************//
void getGyroRaw(I2C_HandleTypeDef *hi2c, int16_t *gyroRaw)
{
	uint8_t out[6] =
	{ 0 };
	;
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT,
			out, 6, 100);

	gyroRaw[0] = (int16_t) (((int16_t) out[0] << 8) | out[1]);
	gyroRaw[1] = (int16_t) (((int16_t) out[2] << 8) | out[3]);
	gyroRaw[2] = (int16_t) (((int16_t) out[4] << 8) | out[5]);
}
//***********************************************************************************************//
void getMagRaw(I2C_HandleTypeDef *hi2c, int16_t *magRaw)
{
	uint8_t out[7] =
	{ 0 };
	uint8_t temp = 0;
	HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS, ST1, I2C_MEMADD_SIZE_8BIT, &temp, 1,
			100);

	if ((temp & 0x01) == 0x01)
	{
		HAL_I2C_Mem_Read(hi2c, AK8963_ADDRESS, HXL, I2C_MEMADD_SIZE_8BIT, out,
				7, 100);

		if (!(out[6] & 0x08)) // Check if magnetic sensor overflow set, if not then report data
			magRaw[0] = (int16_t) (((int16_t) out[1] << 8) | out[0]);
		magRaw[1] = (int16_t) (((int16_t) out[3] << 8) | out[2]);
		magRaw[2] = (int16_t) (((int16_t) out[5] << 8) | out[4]);
	}
}
//***********************************************************************************************//
void MPU9250_init(I2C_HandleTypeDef *hi2c, accScale_t accScale,
		gyroScale_t gyroScale)
{
	// Initialize MPU9250 device
	// wake up device
	uint8_t temp = 0x00;
	uint8_t out;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);  // Clear sleep mode bit (6), enable all sensors
	HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	temp = 0x01;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	temp = 0x03;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	temp = 0x04;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100); // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100); // get current GYRO_CONFIG register value

	// c = c & ~0xE0; // Clear self-test bits [7:5]
	temp = (uint8_t) gyroScale;
	temp = temp << 3;
	out = out & ~0x02; // Clear Fchoice bits [1:0]
	out = out & ~0x18; // Clear AFS bits [4:3]
	out = out | temp; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100);  // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	temp = (uint8_t) accScale;
	temp = temp << 3;
	out = out & ~0x18;  // Clear AFS bits [4:3]
	out = out | temp; // Set full scale range for the accelerometer
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100);
	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100); //get current ACCEL_CONFIG2 register value

	out = out & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	out = out | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2,
	I2C_MEMADD_SIZE_8BIT, &out, 1, 100); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	temp = 0x22;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100);
	temp = 0x01;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, INT_ENABLE, I2C_MEMADD_SIZE_8BIT,
			&out, 1, 100);  // Enable data ready (bit 0) interrupt
}
//***********************************************************************************************//
void MPU9250_reset(I2C_HandleTypeDef *hi2c)
{
	unsigned char ADXL345Setup = 0x80;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&ADXL345Setup, 1, 100);
}

void MPU9250_calibrate(I2C_HandleTypeDef *hi2c, float *gyro, float *acc)
{

	static uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t packetCount, fifoCount;
	int16_t gyroBias[3] =
	{ 0, 0, 0 }, accelBias[3] =
	{ 0, 0, 0 };
	int16_t accelTemp[3] =
	{ 0, 0, 0 }, gyroTemp[3] =
	{ 0, 0, 0 };
	// reset device, reset all registers, clear gyro and accelerometer bias registers
	uint8_t temp = 0x80;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	temp = 0x01;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	HAL_Delay(200);

// Configure device for bias calculation
	temp = 0x00; // Disable all interrupts
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, INT_ENABLE, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Disable FIFO
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, FIFO_EN, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Turn on internal clock source
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Disable I2C master
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, I2C_MST_CTRL, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Disable FIFO and I2C master modes
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, USER_CTRL, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x0C; // Reset FIFO and DMP
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, USER_CTRL, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	HAL_Delay(15);

// Configure MPU9250 gyro and accelerometer for bias calculation
	temp = 0x01; // Set low-pass filter to 188 Hz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Set sample rate to 1 kHz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00; // Set accelerometer full-scale to 2 g, maximum sensitivity
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);

// Configure FIFO to capture accelerometer and gyro data for bias calculation
	temp = 0x40; // Enable FIFO
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, USER_CTRL, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x78; // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, FIFO_EN, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
	temp = 0x00;  // Disable gyro and accelerometer sensors for FIFO
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, FIFO_EN, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, FIFO_COUNTH, I2C_MEMADD_SIZE_8BIT,
			data, 2, 100);	// read FIFO sample count
	fifoCount = ((uint16_t) data[0] << 8) | data[1];
	packetCount = fifoCount / 12;// How many sets of full gyro and accelerometer data for averaging

	for (uint8_t i = 0; i < packetCount; i++)
	{
		HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, FIFO_R_W, I2C_MEMADD_SIZE_8BIT,
				data, 12, 100); // read data for averaging

		// Form signed 16-bit integer for each sample in FIFO
		accelTemp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]);
		accelTemp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
		accelTemp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
		gyroTemp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
		gyroTemp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
		gyroTemp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

		accelBias[0] += (int32_t) accelTemp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accelBias[1] += (int32_t) accelTemp[1];
		accelBias[2] += (int32_t) accelTemp[2];
		gyroBias[0] += (int32_t) gyroTemp[0];
		gyroBias[1] += (int32_t) gyroTemp[1];
		gyroBias[2] += (int32_t) gyroTemp[2];

	}
	accelBias[0] /= (int32_t) packetCount; // Normalize sums to get average count biases
	accelBias[1] /= (int32_t) packetCount;
	accelBias[2] /= (int32_t) packetCount;
	gyroBias[0] /= (int32_t) packetCount;
	gyroBias[1] /= (int32_t) packetCount;
	gyroBias[2] /= (int32_t) packetCount;
	// = 16384 LSB/g    = 131 LSB/degrees/sec
	if (accelBias[2] > 0L)
	{
		accelBias[2] -= (int32_t) 16384;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else
	{
		accelBias[2] += (int32_t) 16384;
	}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyroBias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyroBias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyroBias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyroBias[1] / 4) & 0xFF;
	data[4] = (-gyroBias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyroBias[2] / 4) & 0xFF;

/// Push gyro biases to hardware registers
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, XG_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
//			&data[0], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, XG_OFFSET_L, I2C_MEMADD_SIZE_8BIT,
//			data[1], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, YG_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
//			data[2], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, YG_OFFSET_L, I2C_MEMADD_SIZE_8BIT,
//			data[3], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ZG_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
//			data[4], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ZG_OFFSET_L, I2C_MEMADD_SIZE_8BIT,
//			data[5], 1, 100);
	gyro[0] = (float) gyroBias[0] / (float) 131; // construct gyro bias in deg/s for later manual subtraction
	gyro[1] = (float) gyroBias[1] / (float) 131;
	gyro[2] = (float) gyroBias[2] / (float) 131;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

	int32_t accelBiasReg[3] =
	{ 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
			data, 2, 100); // Read factory accelerometer trim values
	accelBiasReg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
			data, 2, 100);
	accelBiasReg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
			data, 2, 100);
	accelBiasReg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t maskBit[3] =
	{ 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (uint8_t i = 0; i < 3; i++)
	{
		if (accelBiasReg[i] & mask)
		{
			maskBit[i] = 0x01;
		} // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accelBiasReg[0] -= (accelBias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accelBiasReg[1] -= (accelBias[1] / 8);
	accelBiasReg[2] -= (accelBias[2] / 8);

	data[0] = (accelBiasReg[0] >> 8) & 0xFF;
	data[1] = (accelBiasReg[0]) & 0xFF;
	data[1] = data[1] | maskBit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accelBiasReg[1] >> 8) & 0xFF;
	data[3] = (accelBiasReg[1]) & 0xFF;
	data[3] = data[3] | maskBit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accelBiasReg[2] >> 8) & 0xFF;
	data[5] = (accelBiasReg[2]) & 0xFF;
	data[5] = data[5] | maskBit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, XA_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
//			data[5], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, XA_OFFSET_L, I2C_MEMADD_SIZE_8BIT,
//				data[5], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, YA_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
//				data[5], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, YA_OFFSET_L, I2C_MEMADD_SIZE_8BIT,
//				data[5], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ZA_OFFSET_H, I2C_MEMADD_SIZE_8BIT,
//				data[5], 1, 100);
//	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ZA_OFFSET_L, I2C_MEMADD_SIZE_8BIT,
//				data[5], 1, 100);
// Output scaled accelerometer biases for manual subtraction in the main program
	acc[0] = (float) accelBias[0] / (float) 16384;
	acc[1] = (float) accelBias[1] / (float) 16384;
	acc[2] = (float) accelBias[2] / (float) 16384;
}
//****************************************************************************************************************//
//****************************************************************************************************************//
//****************************************************************************************************************//
// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250_selfTest(I2C_HandleTypeDef *hi2c, float *destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] =
	{ 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6], temp;
	int32_t gAvg[3] =
	{ 0 }, aAvg[3] =
	{ 0 }, aSTAvg[3] =
	{ 0 }, gSTAvg[3] =
	{ 0 };
	float factoryTrim[6];
	uint8_t FS = 0;

	temp = 0x00; // Set gyro sample rate to 1 kHz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, INT_ENABLE, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x02; // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = FS << 3; // Set full scale range for the gyro to 250 dps
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x02; // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG2,
	I2C_MEMADD_SIZE_8BIT, &temp, 1, 100);
	temp = FS << 3; // Set full scale range for the accelerometer to 2 g
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);

	for (uint8_t i = 0; i < 200; i++)
	{
		// get average current values of gyro and acclerometer
		HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ACCEL_XOUT_H,
		I2C_MEMADD_SIZE_8BIT, rawData, 6, 100); // Read the six raw data registers into data array

		aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		// Read the six raw data registers sequentially into data array
		HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, GYRO_XOUT_H,
		I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

		gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

	}

	for (uint8_t i = 0; i < 3; i++)
	{ // Get average of 200 values and store as average current readings
		aAvg[i] /= 200;
		gAvg[i] /= 200;
	}

// Configure the accelerometer for self-test
	temp = 0xE0; // Enable self test on all three axes and set accelerometer range to +/- 2 g
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0xE0; // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);

	HAL_Delay(25); // Delay a while to let the device stabilize

	for (int i = 0; i < 200; i++)
	{ // get average self-test values of gyro and acclerometer
	  // Read the six raw data registers into data array
		HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, ACCEL_XOUT_H,
		I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

		aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		// Read the six raw data registers sequentially into data array
		HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, GYRO_XOUT_H,
		I2C_MEMADD_SIZE_8BIT, rawData, 6, 100);

		gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (uint8_t i = 0; i < 3; i++)
	{ // Get average of 200 values and store as average self-test readings
		aSTAvg[i] /= 200;
		gSTAvg[i] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	temp = 0x00;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	temp = 0x00;
	HAL_I2C_Mem_Write(hi2c, MPU9250_ADDRESS, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&temp, 1, 100);
	HAL_Delay(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, SELF_TEST_X_ACCEL,
	I2C_MEMADD_SIZE_8BIT, &selfTest[0], 1, 100); // X-axis accel self-test results
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, SELF_TEST_Y_ACCEL,
	I2C_MEMADD_SIZE_8BIT, &selfTest[1], 1, 100); // Y-axis accel self-test results
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, SELF_TEST_Z_ACCEL,
	I2C_MEMADD_SIZE_8BIT, &selfTest[2], 1, 100); // Z-axis accel self-test results
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, SELF_TEST_X_GYRO,
	I2C_MEMADD_SIZE_8BIT, &selfTest[3], 1, 100); // X-axis gyro self-test results
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, SELF_TEST_Y_GYRO,
	I2C_MEMADD_SIZE_8BIT, &selfTest[3], 1, 100); // Y-axis gyro self-test results
	HAL_I2C_Mem_Read(hi2c, MPU9250_ADDRESS, SELF_TEST_Z_GYRO,
	I2C_MEMADD_SIZE_8BIT, &selfTest[4], 1, 100); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++)
	{
		destination[i] = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))
				/ factoryTrim[i] - 100.; // Report percent differences
		destination[i + 3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))
				/ factoryTrim[i + 3] - 100.; // Report percent differences
	}

}
