/**
 ******************************************************************************
 * @file    LSM6DS3Sensor.h
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Abstract Class of an LSM6DS3 Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DS3Sensor_H__
#define __LSM6DS3Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "hal.h"
#include "LSM6DS3_ACC_GYRO_driver.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_2G   0.061  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G   0.122  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_8G   0.244  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DS3_ACC_SENSITIVITY_FOR_FS_16G  0.488  /**< Sensitivity value for 16 g full scale [mg/LSB] */

#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_125DPS   04.375  /**< Sensitivity value for 125 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_245DPS   08.750  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS   17.500  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_1000DPS  35.000  /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */
#define LSM6DS3_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.000  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */

#define LSM6DS3_PEDOMETER_THRESHOLD_LOW       0x00  /**< Lowest  value of pedometer threshold */
#define LSM6DS3_PEDOMETER_THRESHOLD_MID_LOW   0x07
#define LSM6DS3_PEDOMETER_THRESHOLD_MID       0x0F
#define LSM6DS3_PEDOMETER_THRESHOLD_MID_HIGH  0x17
#define LSM6DS3_PEDOMETER_THRESHOLD_HIGH      0x1F  /**< Highest value of pedometer threshold */

#define LSM6DS3_WAKE_UP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DS3_WAKE_UP_THRESHOLD_MID_LOW   0x0F
#define LSM6DS3_WAKE_UP_THRESHOLD_MID       0x1F
#define LSM6DS3_WAKE_UP_THRESHOLD_MID_HIGH  0x2F
#define LSM6DS3_WAKE_UP_THRESHOLD_HIGH      0x3F  /**< Highest value of wake up threshold */

#define LSM6DS3_TAP_THRESHOLD_LOW       0x01  /**< Lowest  value of wake up threshold */
#define LSM6DS3_TAP_THRESHOLD_MID_LOW   0x08
#define LSM6DS3_TAP_THRESHOLD_MID       0x10
#define LSM6DS3_TAP_THRESHOLD_MID_HIGH  0x18
#define LSM6DS3_TAP_THRESHOLD_HIGH      0x1F  /**< Highest value of wake up threshold */

#define LSM6DS3_TAP_SHOCK_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DS3_TAP_SHOCK_TIME_MID_LOW   0x01
#define LSM6DS3_TAP_SHOCK_TIME_MID_HIGH  0x02
#define LSM6DS3_TAP_SHOCK_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

#define LSM6DS3_TAP_QUIET_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DS3_TAP_QUIET_TIME_MID_LOW   0x01
#define LSM6DS3_TAP_QUIET_TIME_MID_HIGH  0x02
#define LSM6DS3_TAP_QUIET_TIME_HIGH      0x03  /**< Highest value of wake up threshold */

#define LSM6DS3_TAP_DURATION_TIME_LOW       0x00  /**< Lowest  value of wake up threshold */
#define LSM6DS3_TAP_DURATION_TIME_MID_LOW   0x04
#define LSM6DS3_TAP_DURATION_TIME_MID       0x08
#define LSM6DS3_TAP_DURATION_TIME_MID_HIGH  0x0C
#define LSM6DS3_TAP_DURATION_TIME_HIGH      0x0F  /**< Highest value of wake up threshold */

namespace motor_driver {

/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  LSM6DS3_STATUS_OK = 0,
  LSM6DS3_STATUS_ERROR,
  LSM6DS3_STATUS_TIMEOUT,
  LSM6DS3_STATUS_NOT_IMPLEMENTED
} LSM6DS3StatusTypeDef;

typedef enum
{
  LSM6DS3_INT1_PIN,
  LSM6DS3_INT2_PIN
} LSM6DS3_Interrupt_Pin_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
} LSM6DS3_Event_Status_t;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LSM6DS3 Inertial Measurement Unit (IMU) 6 axes
 * sensor.
 */
class LSM6DS3Sensor
{
  public:
    LSM6DS3Sensor                                     (I2CDriver *i2c);
    void                 start                        ();
    LSM6DS3StatusTypeDef Enable_X                     (void);
    LSM6DS3StatusTypeDef Enable_G                     (void);
    LSM6DS3StatusTypeDef Disable_X                    (void);
    LSM6DS3StatusTypeDef Disable_G                    (void);
    LSM6DS3StatusTypeDef ReadID                       (uint8_t *p_id);
    LSM6DS3StatusTypeDef Get_Acc_X                    (int32_t *pData);
    LSM6DS3StatusTypeDef Get_Acc_Y                    (int32_t *pData);
    LSM6DS3StatusTypeDef Get_Acc_Z                    (int32_t *pData);
    LSM6DS3StatusTypeDef Get_Acc                      (int32_t *pData);
    LSM6DS3StatusTypeDef Get_X_Axes                   (int32_t *pData);
    LSM6DS3StatusTypeDef Get_G_Axes                   (int32_t *pData);
    LSM6DS3StatusTypeDef Get_X_Sensitivity            (float *pfData);
    LSM6DS3StatusTypeDef Get_G_Sensitivity            (float *pfData);
    LSM6DS3StatusTypeDef Get_X_AxesRaw                (int16_t *pData);
    LSM6DS3StatusTypeDef Get_G_AxesRaw                (int16_t *pData);
    LSM6DS3StatusTypeDef Get_X_ODR                    (float *odr);
    LSM6DS3StatusTypeDef Get_G_ODR                    (float *odr);
    LSM6DS3StatusTypeDef Set_X_ODR                    (float odr);
    LSM6DS3StatusTypeDef Set_G_ODR                    (float odr);
    LSM6DS3StatusTypeDef Get_X_FS                     (float *fullScale);
    LSM6DS3StatusTypeDef Get_G_FS                     (float *fullScale);
    LSM6DS3StatusTypeDef Set_X_FS                     (float fullScale);
    LSM6DS3StatusTypeDef Set_G_FS                     (float fullScale);
    LSM6DS3StatusTypeDef Enable_Free_Fall_Detection   (void);
    LSM6DS3StatusTypeDef Enable_Free_Fall_Detection   (LSM6DS3_Interrupt_Pin_t int_pin);
    LSM6DS3StatusTypeDef Disable_Free_Fall_Detection  (void);
    LSM6DS3StatusTypeDef Set_Free_Fall_Threshold      (uint8_t thr);
    LSM6DS3StatusTypeDef Enable_Pedometer             (void);
    LSM6DS3StatusTypeDef Disable_Pedometer            (void);
    LSM6DS3StatusTypeDef Get_Step_Counter             (uint16_t *step_count);
    LSM6DS3StatusTypeDef Reset_Step_Counter           (void);
    LSM6DS3StatusTypeDef Set_Pedometer_Threshold      (uint8_t thr);
    LSM6DS3StatusTypeDef Enable_Tilt_Detection        (void);
    LSM6DS3StatusTypeDef Enable_Tilt_Detection        (LSM6DS3_Interrupt_Pin_t int_pin);
    LSM6DS3StatusTypeDef Disable_Tilt_Detection       (void);
    LSM6DS3StatusTypeDef Enable_Wake_Up_Detection     (void);
    LSM6DS3StatusTypeDef Enable_Wake_Up_Detection     (LSM6DS3_Interrupt_Pin_t int_pin);
    LSM6DS3StatusTypeDef Disable_Wake_Up_Detection    (void);
    LSM6DS3StatusTypeDef Set_Wake_Up_Threshold        (uint8_t thr);
    LSM6DS3StatusTypeDef Enable_Single_Tap_Detection  (void);
    LSM6DS3StatusTypeDef Enable_Single_Tap_Detection  (LSM6DS3_Interrupt_Pin_t int_pin);
    LSM6DS3StatusTypeDef Disable_Single_Tap_Detection (void);
    LSM6DS3StatusTypeDef Enable_Double_Tap_Detection  (void);
    LSM6DS3StatusTypeDef Enable_Double_Tap_Detection  (LSM6DS3_Interrupt_Pin_t int_pin);
    LSM6DS3StatusTypeDef Disable_Double_Tap_Detection (void);
    LSM6DS3StatusTypeDef Set_Tap_Threshold            (uint8_t thr);
    LSM6DS3StatusTypeDef Set_Tap_Shock_Time           (uint8_t time);
    LSM6DS3StatusTypeDef Set_Tap_Quiet_Time           (uint8_t time);
    LSM6DS3StatusTypeDef Set_Tap_Duration_Time        (uint8_t time);
    LSM6DS3StatusTypeDef Enable_6D_Orientation        (void);
    LSM6DS3StatusTypeDef Enable_6D_Orientation        (LSM6DS3_Interrupt_Pin_t int_pin);
    LSM6DS3StatusTypeDef Disable_6D_Orientation       (void);
    LSM6DS3StatusTypeDef Get_6D_Orientation_XL        (uint8_t *xl);
    LSM6DS3StatusTypeDef Get_6D_Orientation_XH        (uint8_t *xh);
    LSM6DS3StatusTypeDef Get_6D_Orientation_YL        (uint8_t *yl);
    LSM6DS3StatusTypeDef Get_6D_Orientation_YH        (uint8_t *yh);
    LSM6DS3StatusTypeDef Get_6D_Orientation_ZL        (uint8_t *zl);
    LSM6DS3StatusTypeDef Get_6D_Orientation_ZH        (uint8_t *zh);
	LSM6DS3StatusTypeDef Get_Event_Status             (LSM6DS3_Event_Status_t *status);
    LSM6DS3StatusTypeDef ReadReg                      (uint8_t reg, uint8_t *data);
    LSM6DS3StatusTypeDef WriteReg                     (uint8_t reg, uint8_t data);
	
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      uint8_t add = (uint8_t) ((address >> 1) & 0x7F);
      systime_t tmo = MS2ST(3);
      msg_t status = RDY_OK;
      status = i2cMasterTransmitTimeout(dev_i2c, add, &RegisterAddr, 1, pBuffer, NumByteToRead, tmo);
      if (status == RDY_OK) {
        return 0;
      } else {
        return status;
      }
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      uint8_t add = (uint8_t) ((address >> 1) & 0x7F);
      uint8_t txbuf[NumByteToWrite + 1];
      txbuf[0] = RegisterAddr;
      for (int i = 0; i < NumByteToWrite; i++) {
        txbuf[i + 1] = pBuffer[i];
      }
      systime_t tmo = MS2ST(3);
      msg_t status = RDY_OK;
      status = i2cMasterTransmitTimeout(dev_i2c, add, txbuf, NumByteToWrite + 1, pBuffer, 0, tmo);
      if (status == RDY_OK) {
        return 0;
      } else {
        return status;
      }
    }

  private:
    LSM6DS3StatusTypeDef Set_X_ODR_When_Enabled(float odr);
    LSM6DS3StatusTypeDef Set_G_ODR_When_Enabled(float odr);
    LSM6DS3StatusTypeDef Set_X_ODR_When_Disabled(float odr);
    LSM6DS3StatusTypeDef Set_G_ODR_When_Disabled(float odr);

    /* Helper classes. */
    I2CDriver *dev_i2c;

    /* Configuration */
    uint8_t address;
    float acc_sensitivity;
	
    uint8_t X_isEnabled;
    float X_Last_ODR;
    uint8_t G_isEnabled;
    float G_Last_ODR;
};


#ifdef __cplusplus
extern "C" {
#endif
uint8_t LSM6DS3_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t LSM6DS3_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
}
#endif
} // namespace motor_driver

#endif
