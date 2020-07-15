/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen MAJERLE
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "tm_stm32_mpu9250.h"


TM_MPU9250_Result_t TM_MPU9250_Init(TM_MPU9250_t* MPU9250, TM_MPU9250_Device_t dev) {
    uint8_t data;
    
    /* Set values */
    MPU9250->I2C_Addr = MPU9250_I2C_ADDR | (uint8_t)dev;
    MPU9250->I2C_Addr_Mag = MPU9250_I2C_ADDR_MAG;
    
    /* Init delay */
    TM_DELAY_Init();
    
    /* Init I2C */
    TM_I2C_Init(MPU9250_I2C, MPU9250_I2C_PP, MPU9250_I2C_CLOCK);
    
    /* Check if device connected */
    if (TM_I2C_IsDeviceConnected(MPU9250_I2C, MPU9250->I2C_Addr) != TM_I2C_Result_Ok) {
        return TM_MPU9250_Result_DeviceNotConnected;
    }
    
    /* Check who I am */
    TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, WHO_AM_I_MPU9250, &data);
    if (data != IS_MPU_9555) {
        return TM_MPU9250_Result_DeviceNotConnected;
    }
    
    // wake up device
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    Delayms(100); // Wait for all registers to reset 

    // get stable time source
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    Delayms(200); 

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                            // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, GYRO_CONFIG, &data); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    data &= ~0x02; // Clear Fchoice bits [1:0] 
    data &= ~0x18; // Clear AFS bits [4:3]
    data |= 0x00 << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, GYRO_CONFIG, data); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_CONFIG, &data); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    data &= ~0x18;  // Clear AFS bits [4:3]
    data |= 0x00 << 3; // Set full scale range for the accelerometer 
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_CONFIG, data); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_CONFIG2, &data); // get current ACCEL_CONFIG2 register value
    data &= ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    data |= 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_CONFIG2, data); // Write new ACCEL_CONFIG2 register value
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, INT_PIN_CFG, 0x22);
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, INT_ENABLE, 0x01);
    
    /* Check if device connected */
    if (TM_I2C_IsDeviceConnected(MPU9250_I2C, MPU9250->I2C_Addr_Mag) != TM_I2C_Result_Ok) {
        return TM_MPU9250_Result_DeviceNotConnected;
    }
    
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr_Mag, AK8963_CNTL, 0x00); // Power down magnetometer  
    Delayms(10);
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr_Mag, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    Delayms(10);
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr_Mag, AK8963_CNTL, 0x00); // Power down magnetometer  
    Delayms(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr_Mag, AK8963_CNTL, 1 << 4 | 2); // Set magnetometer data resolution and sample ODR
    Delayms(10);
    
    /* Calculate multiplicators */
    MPU9250->AMult = 2.0f / 32768.0f;
    MPU9250->GMult = 250.0f / 32768.0f;
    MPU9250->MMult = 10.0f * 4912.0f / 32768.0f;
    
    return TM_MPU9250_Result_Ok;
}

TM_MPU9250_Result_t TM_MPU9250_ReadAcce(TM_MPU9250_t* MPU9250) {
    uint8_t data[6];
    
    /* Read accelerometer data */
    TM_I2C_ReadMulti(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_XOUT_H, data, 6);
    
    MPU9250->Ax_Raw = ((int16_t)data[0] << 8) | data[1];
    MPU9250->Ay_Raw = ((int16_t)data[2] << 8) | data[3];  
    MPU9250->Az_Raw = ((int16_t)data[4] << 8) | data[5];
    
    MPU9250->Ax = (float)MPU9250->Ax_Raw * MPU9250->AMult;
    MPU9250->Ay = (float)MPU9250->Ay_Raw * MPU9250->AMult;
    MPU9250->Az = (float)MPU9250->Az_Raw * MPU9250->AMult;
}

TM_MPU9250_Result_t TM_MPU9250_ReadGyro(TM_MPU9250_t* MPU9250) {
    uint8_t data[6];
    TM_I2C_ReadMulti(MPU9250_I2C, MPU9250->I2C_Addr, GYRO_XOUT_H, data, 6);
    
    MPU9250->Gx_Raw = ((int16_t)data[0] << 8) | data[1];
    MPU9250->Gy_Raw = ((int16_t)data[2] << 8) | data[3];  
    MPU9250->Gz_Raw = ((int16_t)data[4] << 8) | data[5];
    
    MPU9250->Gx = (float)MPU9250->Gx_Raw * MPU9250->GMult;
    MPU9250->Gy = (float)MPU9250->Gy_Raw * MPU9250->GMult;
    MPU9250->Gz = (float)MPU9250->Gz_Raw * MPU9250->GMult;
}

TM_MPU9250_Result_t TM_MPU9250_ReadMag(TM_MPU9250_t* MPU9250) {
    uint8_t data[7];
    uint8_t check;
    
    /* Check status */
    TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr_Mag, AK8963_ST1, &check);
    
    if (check & 0x01) {
        TM_I2C_ReadMulti(MPU9250_I2C, MPU9250->I2C_Addr_Mag, AK8963_XOUT_L, data, 7);
        if (!(data[6] & 0x08)) {
            MPU9250->Mx_Raw = ((int16_t)data[1] << 8) | data[0];
            MPU9250->My_Raw = ((int16_t)data[3] << 8) | data[2];
            MPU9250->Mz_Raw = ((int16_t)data[5] << 8) | data[4]; 
        }
    }
}

TM_MPU9250_Result_t TM_MPU9250_DataReady(TM_MPU9250_t* MPU9250) {
    uint8_t data;
    TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, INT_STATUS, &data);
    
    if (data & 0x01) {
        return TM_MPU9250_Result_Ok;
    }
    return TM_MPU9250_Result_Error;
}

//--------------------------------------------------------------------------------------------------------------------------------//

TM_MPU9250_Result_t TM_MPU9250_SetGyroscope(TM_MPU9250_t* MPU9250, TM_MPU9250_GyroSens_t GyroscopeSensitivity) {
	uint8_t temp;
	
	/* Config gyroscope */
	TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, GYRO_CONFIG, &temp);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, GYRO_CONFIG, temp);
	
	switch (GyroscopeSensitivity) {
		case TM_MPU9250_GyroSens_250DPS:
			MPU9250->GMult = (float)1 / MPU_9250_GYRO_SENS_250; 
			break;
		case TM_MPU9250_GyroSens_500DPS:
			MPU9250->GMult = (float)1 / MPU_9250_GYRO_SENS_500; 
			break;
		case TM_MPU9250_GyroSens_1000DPS:
			MPU9250->GMult = (float)1 / MPU_9250_GYRO_SENS_1000; 
			break;
		case TM_MPU9250_GyroSens_2000DPS:
			MPU9250->GMult = (float)1 / MPU_9250_GYRO_SENS_2000; 
		default:
			break;
	}
	
	/* Return OK */
	return TM_MPU9250_Result_Ok;
}

TM_MPU9250_Result_t TM_MPU9250_SetAccelerometer(TM_MPU9250_t* MPU9250, TM_MPU9250_AcceSens_t AccelerometerSensitivity) {
	uint8_t temp;
	
	/* Config accelerometer */
	TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr ,ACCEL_CONFIG, &temp);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_CONFIG, temp);

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case TM_MPU9250_AcceSens_2G:
			MPU9250->AMult = (float)1 / MPU_9250_ACCE_SENS_2; 
			break;
		case TM_MPU9250_AcceSens_4G:
			MPU9250->AMult = (float)1 / MPU_9250_ACCE_SENS_4; 
			break;
		case TM_MPU9250_AcceSens_8G:
			MPU9250->AMult = (float)1 / MPU_9250_ACCE_SENS_8; 
			break;
		case TM_MPU9250_AcceSens_16G:
			MPU9250->AMult = (float)1 / MPU_9250_ACCE_SENS_16; 
		default:
			break;
	}
	
	/* Return OK */
	return TM_MPU9250_Result_Ok;

}

TM_MPU9250_Result_t TM_MPU9250_SetDataRate(TM_MPU9250_t* MPU9250, uint8_t rate) {
	/* Set data sample rate */
	if (TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, SMPLRT_DIV, rate) != TM_I2C_Result_Ok) {
		/* Return error */
		return TM_MPU9250_Result_Error;
	}

	/* Return OK */
	return TM_MPU9250_Result_Ok;
}

/* sets the DLPF bandwidth to values other than default */
TM_MPU9250_Result_t  TM_MPU9250_SetDLPFBandwidth(TM_MPU9250_t* MPU9250, DLPFBandwidth bandwidth)
{
	if (TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, ACCEL_CONFIG2, bandwidth) != TM_I2C_Result_Ok) {
		/* Return error */
		return TM_MPU9250_Result_Error;
	}
if (TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, CONFIG, bandwidth) != TM_I2C_Result_Ok) {
		/* Return error */
		return TM_MPU9250_Result_Error;
	}
	/* Return OK */
	return TM_MPU9250_Result_Ok;
}


TM_MPU9250_Result_t TM_MPU9250_EnableInterrupts(TM_MPU9250_t* MPU9250) {
	uint8_t temp;	
	
	/* Enable interrupts for data ready and motion detect */
	// 0x01 Solo DATA_READY_ENABLE_MSK 
	#define DATA_READY_ENABLE_MSK 0x01
	TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, INT_ENABLE, DATA_READY_ENABLE_MSK);
	
	/* Clear IRQ flag on any read operation */
	TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, INT_PIN_CFG, &temp);
	temp |= 0x10;
	TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, INT_PIN_CFG, temp);
	
	/* Return OK */
	return TM_MPU9250_Result_Ok;
}

TM_MPU9250_Result_t TM_MPU9250_DisableInterrupts(TM_MPU9250_t* MPU9250) {
	/* Disable interrupts */
		// 0x00 Solo DATA_READY_ENABLE_MSK 
	#define DATA_READY_DISABLE_MSK 0x00
	if (TM_I2C_Write(MPU9250_I2C, MPU9250->I2C_Addr, INT_ENABLE, DATA_READY_DISABLE_MSK) != TM_I2C_Result_Ok) {
		/* Return error */
		return TM_MPU9250_Result_Error;
	}
	
	/* Return OK */
	return TM_MPU9250_Result_Ok;
}

TM_MPU9250_Result_t TM_MPU9250_ReadInterrupts(TM_MPU9250_t* MPU9250, TM_MPU9250_Interrupt_t* InterruptsStruct) {
	uint8_t read;
	
	/* Reset structure */
	InterruptsStruct->Status = 0;
	
	/* Read interrupts status register */
	if (TM_I2C_Read(MPU9250_I2C, MPU9250->I2C_Addr, INT_STATUS, &read) != TM_I2C_Result_Ok) {
		/* Return error */
		return TM_MPU9250_Result_Error;
	}
	
	/* Fill value */
	InterruptsStruct->Status = read;
	
	/* Return OK */
	return TM_MPU9250_Result_Ok;
}

