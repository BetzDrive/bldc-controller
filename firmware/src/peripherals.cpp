#include "peripherals.h"

#include "constants.h"
#include "usbcfg.h"
#include "state.h"

SerialUSBDriver SDU1;

namespace motor_driver {

void resumeInnerControlLoop();

/**
 * Called at the start of every motor PWM cycle
 */
static void motorPWMPeriodicCallback(PWMDriver *pwmp) {
  (void)pwmp;

  bool timer_counting_up = ((PWMD1.tim->CR1 & TIM_CR1_DIR) == 0);

  /*
   * Run the inner control loop every other PWM cycle, due to center-aligned PWM
   */
  if (timer_counting_up) {
    resumeInnerControlLoop();
  }
}

PWMConfig motor_pwm_config = {
  motor_pwm_clock_freq,                         // PWM clock frequency
  motor_pwm_clock_freq / motor_pwm_cycle_freq, 	// PWM period (ticks)
  motorPWMPeriodicCallback,                		  // PWM callback
  {
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0,                                // CR2 (select enable signal as TRGO output)
  0,                                            // BDTR
  0
};

DRV8312 gate_driver(
  PWMD1,
  2,
  1,
  0,
  {GPIOB, GPIOB_MDRV_RST_A},
  {GPIOB, GPIOB_MDRV_RST_B},
  {GPIOB, GPIOB_MDRV_RST_C},
  {GPIOC, GPIOC_MDRV_EN},
  {GPIOC, GPIOC_MDRV_NFAULT},
  {GPIOC, GPIOC_MDRV_NOCTW}
);

constexpr unsigned int led_pwm_clock_freq = 84000000; // Hz
constexpr unsigned int led_pwm_period = 52500; // clock cycles

const PWMConfig led_pwm_config = {
  led_pwm_clock_freq,
  led_pwm_period,
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_ACTIVE_LOW, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
  0,
  0
};

AS5047D encoder(
  SPID3,
  {GPIOA, GPIOA_ENC_CSN}
);

BinarySemaphore ivsense_adc_samples_bsem;

volatile adcsample_t *ivsense_adc_samples_ptr = nullptr;

volatile size_t ivsense_adc_samples_count;

adcsample_t ivsense_sample_buf[ivsense_channel_count * ivsense_sample_buf_depth];

static void ivsenseADCEndCallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;

  chSysLockFromIsr();

  ivsense_adc_samples_ptr = buffer;
  ivsense_adc_samples_count = n;
  chBSemSignalI(&ivsense_adc_samples_bsem); // Signal that new ADC samples are available

  chSysUnlockFromIsr();
}

static void ivsenseADCErrorCallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;

  // TODO: display error
}

static const ADCConversionGroup ivsense_adc_group = {
  true,                                     // Use circular buffer
  ivsense_channel_count,
  ivsenseADCEndCallback,
  ivsenseADCErrorCallback,
  0,                                        // CR1
  ADC_CR2_EXTSEL_3 | ADC_CR2_EXTEN_0,       // CR2 (begin conversion on rising edge of TIM3 TRGO)
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_15) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15) | ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15)
      | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_15) | ADC_SMPR1_SMP_AN14(ADC_SAMPLE_15) | ADC_SMPR1_SMP_AN15(ADC_SAMPLE_15), // SMPR1
  ADC_SMPR2_SMP_AN8(ADC_SAMPLE_15),         // SMPR2
  ADC_SQR1_NUM_CH(ivsense_channel_count),   // SQR1
  ADC_SQR2_SQ7_N(ADC_CHANNEL_IN13),         // SQR2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN14) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN15) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN8)
      | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN10) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN11) | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN12) // SQR3
};

static const PWMConfig adc_trigger_pwm_config = {
  adc_pwm_cycle_freq,                                     // PWM clock frequency
  2,                                                      // PWM period (ticks)
  NULL,                                                   // PWM callback
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  0, 
  0,                                                      // BDTR
  0
};

MCP9808 temp_sensor(I2CD1);

LSM6DS3Sensor acc_gyr(&I2CD1);

void initPeripherals() {
  chBSemInit(&ivsense_adc_samples_bsem, true);
}

void startPeripherals() {
  // Start USB serial
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serial_usb_config);

  usbDisconnectBus(serial_usb_config.usbp);
  chThdSleepMilliseconds(100);
  usbStart(serial_usb_config.usbp, &usb_config);
  usbConnectBus(serial_usb_config.usbp);

  // Start LED PWM timer
  pwmStart(&PWMD5, &led_pwm_config);

  // Configure motor PWM timer and pause it
  pwmStart(&PWMD1, &motor_pwm_config);
  PWMD1.tim->CR1 &= ~TIM_CR1_CEN;
  PWMD1.tim->CR1 = (PWMD1.tim->CR1 & ~TIM_CR1_CMS) 
                   | TIM_CR1_CMS_0 | TIM_CR1_CMS_1 // (TIM1_CMS = 11) Enable center-aligned PWM
                   ;

  // Start gate driver
  gate_driver.start();

  // Start encoder
  //startEncoder();
  encoder.start();

  // Start temperature sensor
  temp_sensor.start();

  // Start accelerometer
  acc_gyr.start();
  acc_gyr.Enable_X();

  // Start ADC
  adcStart(&ADCD1, NULL);
  adcStartConversion(&ADCD1, &ivsense_adc_group, ivsense_sample_buf, ivsense_sample_buf_depth);
  // Configure ADC trigger timer and pause it
  // Note: no PWM outputs are generated, this is just a convenient way to configure a timer
  pwmStart(&PWMD3, &adc_trigger_pwm_config);
  PWMD3.tim->CR1 &= ~TIM_CR1_CEN;

  // From section 18.3.15 of the STM32f405 reference manual
  // Set up Timer 1 (Motor PWM Output) as a master for Timer 3 (ADC Sampler)
  PWMD1.tim->CR2  = TIM_CR2_MMS_1;                                       // (TIM1_MMS = 010) Set Timer 1 to send trigger on count
  PWMD3.tim->CR2  = TIM_CR2_MMS_1;                                       // (TIM3_MMS = 010) Set Update signal as TRGO Output
  PWMD3.tim->SMCR = (PWMD3.tim->SMCR & ~TIM_SMCR_TS & ~TIM_SMCR_SMS)     // (TIM3_TS = 000) Enable counter on TIM1 (motor PWM) TRGO rising edge
                    | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0   // (TIM3_SMS = 111) External clock mode
                    ;

  // Reset timer counters
  PWMD1.tim->CNT = 0;
  PWMD3.tim->CNT = 0;

  // Start motor PWM timer, which also starts the ADC trigger timer
  PWMD1.tim->CR1 |= TIM_CR1_CEN;
  PWMD3.tim->CR1 |= TIM_CR1_CEN;
}

static uint16_t ledPWMPulseWidthFromIntensity(uint8_t intensity) {
  return led_gamma_table[intensity];
}

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue) {
  pwmEnableChannel(&PWMD5, 0, ledPWMPulseWidthFromIntensity(red));
  pwmEnableChannel(&PWMD5, 2, ledPWMPulseWidthFromIntensity(green));
  pwmEnableChannel(&PWMD5, 1, ledPWMPulseWidthFromIntensity(blue));
}

void setStatusLEDColor(uint32_t color) {
  setStatusLEDColor(color >> 16, color >> 8, color);
}

void setCommsActivityLED(bool on) {
  palWritePad(GPIOA, GPIOA_LED_Y, !on);
}

void setRS485TransmitMode(bool transmit) {
  palWritePad(GPIOD, GPIOD_RS485_DIR, transmit);
}

} // namespace motor_driver
