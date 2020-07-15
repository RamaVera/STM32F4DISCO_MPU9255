/**
 * @author  Tilen MAJERLE
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    
 * @version v1.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   Library template 
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen MAJERLE

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef TM_LIBRARY_H
#define TM_LIBRARY_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup TM_LIBNAME
 * @brief    Library description here
 * @{
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
  - First release
\endverbatim
 *
 * \par Dependencies
 *
\verbatim
 - STM32Fxxx HAL
 - defines.h
\endverbatim
 */

#include "stm32fxxx_hal.h"
#include "defines.h"
#include "tm_stm32_i2c.h"
#include "tm_stm32_delay.h"

/**
 * @defgroup TM_LIB_Macros
 * @brief    Library defines
 * @{
 */
/* Magnetometer */
#define WHO_AM_I_AK8963     0x00 // Result = 0x48
#define INFO                0x01
#define AK8963_ST1          0x02
#define AK8963_XOUT_L	    0x03
#define AK8963_XOUT_H	    0x04
#define AK8963_YOUT_L	    0x05
#define AK8963_YOUT_H	    0x06
#define AK8963_ZOUT_L	    0x07
#define AK8963_ZOUT_H	    0x08
#define AK8963_ST2          0x09
#define AK8963_CNTL         0x0A
#define AK8963_ASTC         0x0C
#define AK8963_I2CDIS       0x0F
#define AK8963_ASAX         0x10
#define AK8963_ASAY         0x11
#define AK8963_ASAZ         0x12

/* MPU9250 data */
#define SELF_TEST_X_GYRO    0x00                  
#define SELF_TEST_Y_GYRO    0x01                                                                          
#define SELF_TEST_Z_GYRO    0x02

#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E    
#define SELF_TEST_Z_ACCEL   0x0F

#define SELF_TEST_A         0x10

#define XG_OFFSET_H         0x13
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19


#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG2       0x1D
#define LP_ACCEL_ODR        0x1E   
#define WOM_THR             0x1F   

#define MOT_DUR             0x20
#define ZMOT_THR            0x21
#define ZRMOT_DUR           0x22

#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24   
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1          0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2          0x6C
#define DMP_BANK            0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT          0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG             0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1           0x70
#define DMP_REG_2           0x71 
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I_MPU9250    0x75 // Should return 0x71
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

#define IS_MPU_9555					0x73
#define IS_MPU_9250					0x71

/* Gyro sensitivities in degrees/s */
#define MPU_9250_GYRO_SENS_250		((float) 131)
#define MPU_9250_GYRO_SENS_500		((float) 65.5)
#define MPU_9250_GYRO_SENS_1000		((float) 32.8)
#define MPU_9250_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU_9250_ACCE_SENS_2			((float) 16384)
#define MPU_9250_ACCE_SENS_4			((float) 8192)
#define MPU_9250_ACCE_SENS_8			((float) 4096)
#define MPU_9250_ACCE_SENS_16		((float) 2048)

#ifndef MPU9250_I2C
#define MPU9250_I2C             I2C1
#define MPU9250_I2C_PP          TM_I2C_PinsPack_3
#endif

#define MPU9250_I2C_CLOCK       400000

#define MPU9250_I2C_ADDR        0xD0  
#define MPU9250_I2C_ADDR_MAG    (0x0C << 1)

/**
 * @}
 */
 
/**
 * @defgroup TM_LIB_Typedefs
 * @brief    Library Typedefs
 * @{
 */

typedef enum _TM_MPU9250_Result_t {
    TM_MPU9250_Result_Ok = 0x00,
    TM_MPU9250_Result_Error,
    TM_MPU9250_Result_DeviceNotConnected
} TM_MPU9250_Result_t;

typedef enum _TM_MPU9250_Device_t {
    TM_MPU9250_Device_0 = 0x00,
    TM_MPU9250_Device_1 = 0x02
} TM_MPU9250_Device_t;

typedef enum _TM_MPU9250_AcceSens_t {
    TM_MPU9250_AcceSens_2G = 0x00,
    TM_MPU9250_AcceSens_4G = 0x01,
    TM_MPU9250_AcceSens_8G = 0x02,
    TM_MPU9250_AcceSens_16G = 0x03
} TM_MPU9250_AcceSens_t;

typedef enum _TM_MPU9250_GyroSens_t {
    TM_MPU9250_GyroSens_250DPS = 0x00,
    TM_MPU9250_GyroSens_500DPS = 0x01,
    TM_MPU9250_GyroSens_1000DPS = 0x02,
    TM_MPU9250_GyroSens_2000DPS = 0x03
} TM_MPU9250_GyroSens_t;

typedef enum _TM_MPU9250_MagSens_t {
    TM_MPU9250_MagSens_14Bit = 0x00,    // 0.6 mG per LSB
    TM_MPU9250_MagSens_16Bit            // 0.15 mG per LSB
} TM_MPU9250_MagSens_t;


typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_184HZ = 0,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

typedef enum SampleRateDivider_ {
	LP_ACCEL_ODR_0_24HZ = 0,
	LP_ACCEL_ODR_0_49HZ,
	LP_ACCEL_ODR_0_98HZ,
	LP_ACCEL_ODR_1_95HZ,
	LP_ACCEL_ODR_3_91HZ,
	LP_ACCEL_ODR_7_81HZ,
	LP_ACCEL_ODR_15_63HZ,
	LP_ACCEL_ODR_31_25HZ,
	LP_ACCEL_ODR_62_50HZ,
	LP_ACCEL_ODR_125HZ,
	LP_ACCEL_ODR_250HZ,
	LP_ACCEL_ODR_500HZ
} SampleRateDivider;


typedef struct _TM_MPU9250_t {
    float Ax, Ay, Az;         /*!< Accelerometer raw data */
    float Gx, Gy, Gz;         /*!< Gyroscope raw data */
    float Mx, My, Mz;         /*!< Magnetometer raw data */
    int16_t Ax_Raw, Ay_Raw, Az_Raw;         /*!< Accelerometer raw data */
    int16_t Gx_Raw, Gy_Raw, Gz_Raw;         /*!< Gyroscope raw data */
    int16_t Mx_Raw, My_Raw, Mz_Raw;         /*!< Magnetometer raw data */
    
    float AMult, GMult, MMult;
    
    uint8_t I2C_Addr;
    uint8_t I2C_Addr_Mag;
} TM_MPU9250_t;

typedef union _TM_MPU9250_Interrupt_t {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} TM_MPU9250_Interrupt_t;



/**
 * @}
 */

/**
 * @defgroup TM_LIB_Functions
 * @brief    Library Functions
 * @{
 */

TM_MPU9250_Result_t TM_MPU9250_Init(TM_MPU9250_t* MPU9250, TM_MPU9250_Device_t dev);

TM_MPU9250_Result_t TM_MPU9250_ReadAcce(TM_MPU9250_t* MPU9250);
TM_MPU9250_Result_t TM_MPU9250_ReadGyro(TM_MPU9250_t* MPU9250);
TM_MPU9250_Result_t TM_MPU9250_ReadMag(TM_MPU9250_t* MPU9250);
TM_MPU9250_Result_t TM_MPU9250_DataReady(TM_MPU9250_t* MPU9250);


TM_MPU9250_Result_t TM_MPU9250_SetGyroscope(TM_MPU9250_t* MPU9250, TM_MPU9250_GyroSens_t GyroscopeSensitivity) ;
TM_MPU9250_Result_t TM_MPU9250_SetAccelerometer(TM_MPU9250_t* MPU9250, TM_MPU9250_AcceSens_t AccelerometerSensitivity) ;
TM_MPU9250_Result_t TM_MPU9250_SetDataRate(TM_MPU9250_t* MPU9250, uint8_t rate) ;
TM_MPU9250_Result_t TM_MPU9250_SetDLPFBandwidth(TM_MPU9250_t* MPU9250, DLPFBandwidth bandwidth);
TM_MPU9250_Result_t TM_MPU9250_EnableInterrupts(TM_MPU9250_t* MPU9250);
TM_MPU9250_Result_t TM_MPU9250_DisableInterrupts(TM_MPU9250_t* MPU9250);
TM_MPU9250_Result_t TM_MPU9250_ReadInterrupts(TM_MPU9250_t* MPU9250, TM_MPU9250_Interrupt_t* InterruptsStruct) ;
/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
