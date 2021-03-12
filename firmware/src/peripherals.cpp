#include "peripherals.h"

#include "constants.h"
#include "hw_conf.h"
#include "state.h"
#include "stm32f4xx_flash.h"

namespace motor_driver {
namespace peripherals {

void resumeInnerControlLoop();

MUTEX_DECL(var_access_mutex);

/**
 * Called at the start of every motor PWM cycle
 */
static void motorPWMPeriodicCallback(PWMDriver *pwmp) {
  (void)pwmp;
  static uint8_t events = 0;
  /*
   * Run the inner control loop at the control frequency given
   */
  events = (events + 1) % consts::current_control_count_per_motor_cycle;
  if (events == 0) {
    controller::resumeInnerControlLoop();
  }
}

PWMConfig motor_pwm_config = {
    // PWM clock frequency.
    consts::motor_pwm_clock_freq,
    // PWM period (ticks).
    consts::motor_pwm_clock_freq / consts::motor_pwm_cycle_freq,
    // PWM callback.
    motorPWMPeriodicCallback,
    {{PWM_OUTPUT_ACTIVE_LOW, NULL},
     {PWM_OUTPUT_ACTIVE_LOW, NULL},
     {PWM_OUTPUT_ACTIVE_LOW, NULL},
     {PWM_OUTPUT_DISABLED, NULL}},
    // CR2.
    0,
    // BDTR.
    0,
    // DIER.
    0,
};

DRV8312 gate_driver(PWMD1, 2, 1, 0, {GPIOC, GPIOC_MDRV_RST_A},
                    {GPIOC, GPIOC_MDRV_RST_B}, {GPIOC, GPIOC_MDRV_RST_C},
                    {GPIOC, GPIOC_MDRV_NFAULT}, {GPIOB, GPIOB_MDRV_NOCTW});

static // Hz.
    constexpr unsigned int led_pwm_clock_freq = 84000000;
static // clock cycles.
    constexpr unsigned int led_pwm_period = 52500;

const PWMConfig led_pwm_config = {
    led_pwm_clock_freq,
    led_pwm_period,
    NULL,
    {{PWM_OUTPUT_ACTIVE_LOW, NULL},
     {PWM_OUTPUT_ACTIVE_LOW, NULL},
     {PWM_OUTPUT_ACTIVE_LOW, NULL},
     {PWM_OUTPUT_DISABLED, NULL}},
    // CR2.
    0,
    // BDTR.
    0,
    // DIER.
    0,
};

AS5047D encoder(SPID3, {GPIOB, GPIOB_ENC_CSN});

BinarySemaphore ivsense_adc_samples_bsem;

volatile adcsample_t *curra_adc_samples_ptr = nullptr;
volatile adcsample_t *currb_adc_samples_ptr = nullptr;
volatile adcsample_t *currc_adc_samples_ptr = nullptr;
volatile adcsample_t *vsense_adc_samples_ptr = nullptr;

// volatile size_t ivsense_adc_samples_count;

adcsample_t ivsense_sample_buf[consts::ivsense_channel_count *
                               consts::ivsense_sample_buf_depth];

static void ivsenseADCEndCallback(ADCDriver *adcp, adcsample_t *buffer,
                                  size_t n) {
  (void)adcp;

  chSysLockFromIsr();
  bool is_odd_sample = (buffer - ivsense_sample_buf) >> 1;
  curra_adc_samples_ptr = buffer;
  vsense_adc_samples_ptr = buffer + 1;
  currb_adc_samples_ptr = ivsense_sample_buf + 4 + is_odd_sample;
  currc_adc_samples_ptr = ivsense_sample_buf + 6 + is_odd_sample;
  // ivsense_adc_samples_count = n;
  // Signal that new ADC samples are available.
  chBSemSignalI(&ivsense_adc_samples_bsem);

  chSysUnlockFromIsr();
}

static void ivsenseADCErrorCallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;

  // TODO: display error
}

static const ADCConversionGroup currc_adc_group = {
    // Use circular buffer.
    true, 1, NULL, NULL,
    // CR1.
    0,
    // CR2 (begin conversion on rising edge of TIM3 TRGO).
    ADC_CR2_EXTSEL_3 | ADC_CR2_EXTEN_0,
    // SMPR1.
    ADC_SMPR1_SMP_AN10(ADC_SAMPLE_15),
    // SMPR2.
    0,
    // SQR1.
    ADC_SQR1_NUM_CH(1),
    // SQR2.
    0,
    // SQR3
    ADC_SQR3_SQ1_N(CURR_C_CHANNEL)};

static const ADCConversionGroup currb_adc_group = {
    // Use circular buffer.
    true,
    // Channels.
    1, NULL, NULL,
    // CR1.
    0,
    // CR2 (begin conversion on rising edge of TIM3 TRGO).
    ADC_CR2_EXTSEL_3 | ADC_CR2_EXTEN_0,
    // SMPR1.
    ADC_SMPR1_SMP_AN11(ADC_SAMPLE_15),
    // SMPR2.
    0,
    // SQR1.
    ADC_SQR1_NUM_CH(1),
    // SQR2.
    0,
    // SQR3.
    ADC_SQR3_SQ1_N(CURR_B_CHANNEL)};

static const ADCConversionGroup ivsense_adc_group = {
    // Use circular buffer.
    true,
    // Channels.
    2, ivsenseADCEndCallback, ivsenseADCErrorCallback,
    // CR1.
    0,
    // CR2 (begin conversion on rising edge of TIM3 TRGO).
    ADC_CR2_EXTSEL_3 | ADC_CR2_EXTEN_0,
    // SMPR1.
    ADC_SMPR1_SMP_AN12(ADC_SAMPLE_15) | ADC_SMPR1_SMP_AN13(ADC_SAMPLE_15),
    // SMPR2.
    0,
    // SQR1.
    ADC_SQR1_NUM_CH(2),
    // SQR2.
    0,
    // SQR3.
    ADC_SQR3_SQ1_N(CURR_A_CHANNEL) | ADC_SQR3_SQ2_N(VBUS_CHANNEL)};

static const PWMConfig adc_trigger_pwm_config = {
    consts::adc_pwm_cycle_freq,
    // Period. This becomes the delay between the wrapping of the PWM clock
    // and the start of the ADC sampling seq.
    5,
    // Callback.
    NULL,
    {{PWM_OUTPUT_DISABLED, NULL},
     {PWM_OUTPUT_DISABLED, NULL},
     {PWM_OUTPUT_DISABLED, NULL},
     {PWM_OUTPUT_DISABLED, NULL}},
    // CR2.
    0,
    // BDTR.
    0,
    // DIER.
    0,
};

MCP9808 temp_sensor(I2CD2);

IIS328DQ acc(I2CD2);

void initPeripherals() { chBSemInit(&ivsense_adc_samples_bsem, true); }

void startPeripherals() {
  // Set Brownout Threshold to 2.7V
  FLASH_OB_BORConfig(OB_BOR_LEVEL3);

  // Start LED PWM timer
  pwmStart(&PWMD5, &led_pwm_config);

  // Configure motor PWM timer and pause it
  pwmStart(&PWMD1, &motor_pwm_config);
  PWMD1.tim->CR1 &= ~TIM_CR1_CEN;
  PWMD1.tim->CR1 = (PWMD1.tim->CR1 & ~TIM_CR1_CMS);
  // Center aligned
  PWMD1.tim->CR1 |= TIM_CR1_CMS_1;
  // Halve the update interrupt rate
  PWMD1.tim->RCR = 1;

  // Start gate driver
  gate_driver.start();

  // Start encoder
  encoder.start();

  // Start temperature sensor
  temp_sensor.start();

  // Start accelerometer
  acc.start();

  // Start ADC
  adcStart(&ADCD1, NULL);
  adcStart(&ADCD2, NULL);
  adcStart(&ADCD3, NULL);

  adcStartConversion(&ADCD1, &ivsense_adc_group, ivsense_sample_buf,
                     consts::ivsense_sample_buf_depth);
  adcStartConversion(&ADCD2, &currb_adc_group,
                     ivsense_sample_buf +
                         (2 * consts::ivsense_sample_buf_depth),
                     consts::ivsense_sample_buf_depth);
  adcStartConversion(&ADCD3, &currc_adc_group,
                     ivsense_sample_buf +
                         (3 * consts::ivsense_sample_buf_depth),
                     consts::ivsense_sample_buf_depth);
  // Configure ADC trigger timer and pause it.
  // Note: no PWM outputs are generated, this is just a convenient way to
  // configure a timer.
  pwmStart(&PWMD3, &adc_trigger_pwm_config);

  // Turn off TIM3.
  PWMD3.tim->CR1 &= ~TIM_CR1_CEN;
  // Set TIM3 One Pulse Mode.
  PWMD3.tim->CR1 |= TIM_CR1_OPM;

  // From section 18.3.15 of the STM32f405 reference manual.
  // Set up Timer 1 (Motor PWM Output) as a master for
  // Timer 3 (ADC Sampler).
  // (TIM1_MMS = 010) Set Timer 1 to send trigger on count.
  PWMD1.tim->CR2 = TIM_CR2_MMS_1;
  // (TIM3_MMS = 010) Set Update signal as TRGO Output.
  PWMD3.tim->CR2 = TIM_CR2_MMS_1;
  // (TIM3_TS = 000) Enable counter on TIM1 (motor PWM) TRGO rising edge.
  // (TIM3_SMS = 110) Trigger mode: update event from TIM1 restarts the
  //  timer.
  PWMD3.tim->SMCR = (PWMD3.tim->SMCR & ~TIM_SMCR_TS & ~TIM_SMCR_SMS) |
                    TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;

  // Reset timer counters.
  PWMD1.tim->CNT = 0;
  PWMD3.tim->CNT = 0;

  // Start motor PWM timer, which also starts the ADC trigger timer.
  PWMD3.tim->CR1 |= TIM_CR1_CEN;
  PWMD1.tim->CR1 |= TIM_CR1_CEN;
}

static uint16_t ledPWMPulseWidthFromIntensity(uint8_t intensity) {
  return consts::led_gamma_table[intensity];
}

void setStatusLEDColor(uint8_t red, uint8_t green, uint8_t blue) {
  pwmEnableChannel(&PWMD5, 2, ledPWMPulseWidthFromIntensity(red));
  pwmEnableChannel(&PWMD5, 0, ledPWMPulseWidthFromIntensity(green));
  pwmEnableChannel(&PWMD5, 1, ledPWMPulseWidthFromIntensity(blue));
}

void setStatusLEDColor(uint32_t color) {
  setStatusLEDColor(color >> 16, color >> 8, color);
}

void setCommsActivityLED(bool on) { palWritePad(GPIOB, GPIOB_LED_Y, !on); }

void setRS485TransmitMode(bool transmit) {
  palWritePad(GPIOD, GPIOD_RS485_DIR, transmit);
}
} // namespace peripherals
} // namespace motor_driver
