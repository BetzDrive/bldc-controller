#include "peripherals.h"

#include "constants.h"
#include "state.h"

namespace motor_driver {

void resumeInnerControlLoop();

static volatile uint16_t ADC_Value[3]; 
constexpr unsigned int led_pwm_clock_freq = 84000000; // Hz
constexpr unsigned int led_pwm_period = 52500; // clock cycles

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

MCP9808 temp_sensor(I2CD1);

LSM6DS3Sensor acc_gyr(&I2CD1);

/* Configuring DMA/ADC to use Tim1 to trigger injected conversions on all 3 ADCs */
void configureIVSense() {
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;

  // Clock
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

  // DMA for the ADC
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &DMA_InitStructure);

  // DMA2_Stream0 enable
  DMA_Cmd(DMA2_Stream4, ENABLE);

  // ADC Common Init
  // Note that the ADC is running at 42MHz, which is higher than the
  // specified 36MHz in the data sheet, but it works.
  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_InjecSimult;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  // Channel-specific settings
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  // ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
  // ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = 0;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;

  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_Init(ADC3, &ADC_InitStructure);

  // Enable DMA request after last transfer (Multi-ADC mode)
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

  // Injected channels for current measurement at end of cycle
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
  // ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_CC4);
  ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);
  // ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Falling);
  ADC_InjectedSequencerLengthConfig(ADC1, 3);
  ADC_InjectedSequencerLengthConfig(ADC2, 2);
  ADC_InjectedSequencerLengthConfig(ADC3, 2);

  // ADC1 regular channels
  ADC_InjectedChannelConfig(ADC1, CURR_A_CHANNEL, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC1, VSENSE_A_CHANNEL, 2, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC1, VBUS_CHANNEL, 3, ADC_SampleTime_15Cycles);

  // ADC2 regular channels
  ADC_InjectedChannelConfig(ADC2, CURR_B_CHANNEL, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC2, VSENSE_B_CHANNEL, 2, ADC_SampleTime_15Cycles);

  // ADC3 regular channels
  ADC_InjectedChannelConfig(ADC3, CURR_C_CHANNEL, 1, ADC_SampleTime_15Cycles);
  ADC_InjectedChannelConfig(ADC3, VSENSE_C_CHANNEL, 2, ADC_SampleTime_15Cycles);

  // Interrupt
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
  nvicEnableVector(ADC_IRQn, 4);

  // Enable ADC1
  ADC_Cmd(ADC1, ENABLE);

  // Enable ADC2
  ADC_Cmd(ADC2, ENABLE);

  // Enable ADC3
  ADC_Cmd(ADC3, ENABLE);
}   /* End of ADC Configure */

void initPeripherals() {
  /* Configuring/Starting Internal Peripherals */
  // Start LED PWM timer
  pwmStart(&PWMD5, &led_pwm_config);

  // Initializing ADC/DMA for Current/Voltage Readings
  configureIVSense();

  // Configure motor PWM timer and pause it
  pwmStart(&PWMD1, &motor_pwm_config);
  PWMD1.tim->CR1 &= ~TIM_CR1_CEN;
  PWMD1.tim->CR1 = (PWMD1.tim->CR1 & ~TIM_CR1_CMS) 
                   | TIM_CR1_CMS_0 | TIM_CR1_CMS_1 // (TIM1_CMS = 11) Enable center-aligned PWM
                   ;
  PWMD1.tim->CR2 |= TIM_CR2_MMS_1 | TIM_CR2_CCDS;
  /* End of Config for Internal Peripherals */

  /* Initializing External Device Drivers */

  // Start gate driver
  gate_driver.start();

  // Start encoder
  encoder.start();

  // Start temperature sensor
  temp_sensor.start();

  // Start accelerometer
  acc_gyr.start();
  acc_gyr.Enable_X();

  /* End of External Device Driver Init */

  // Reset timer counter
  PWMD1.tim->CNT = 0;

  // Start motor PWM timer
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
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
