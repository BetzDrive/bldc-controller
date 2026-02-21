#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the FOC control loop.
 * Must be called after state_load_calibration(). */
void control_init(void);

/* Single control step, called from TIM1 update ISR at 20kHz.
 * Internally divides down to 10kHz for actual control. */
void control_step(void);

/* Set all phases to zero and switch to raw PWM mode. */
void control_brake(void);

/* Reset the communication timeout timer. */
void control_reset_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
