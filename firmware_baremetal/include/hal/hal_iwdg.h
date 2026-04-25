#ifndef HAL_IWDG_H
#define HAL_IWDG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Abstract independent watchdog interface. */

void hal_iwdg_init(void);
void hal_iwdg_kick(void);

/* Pause/resume for long flash operations. */
void hal_iwdg_pause(void);
void hal_iwdg_resume(void);

/* Clear hardware watchdog-reset flag (RCC_CSR WDGRSTF on STM32). */
void hal_iwdg_clear_reset_flag(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_IWDG_H */
