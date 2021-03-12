
#include "helper.h"
#include "flash.h"
#include "stm32f4xx_iwdg.h"
#include <ch.h>
#include <hal.h>

void flashJumpApplication(uint32_t address) {
  typedef void (*funcPtr)(void);

  u32 jumpAddr = *(vu32 *)(address + 0x04); /* reset ptr in vector table */
  funcPtr usrMain = (funcPtr)jumpAddr;

  /* Reset all interrupts to default */
  chSysDisable();

  /* Clear pending interrupts just to be on the save side */
  SCB_ICSR = ICSR_PENDSVCLR;

  /* Disable all interrupts */
  int i;
  for (i = 0; i < 8; ++i)
    NVIC->ICER[i] = NVIC->IABR[i];

  /* Set stack pointer as in application's vector table */
  __set_MSP(*(vu32 *)address);
  usrMain();
}

struct IWDG_Values pauseIWDG() {
  // Save current configs.
  struct IWDG_Values save;
  save.prescaler = IWDG->PR;
  save.reload = IWDG->RLR;

  // Set config to very large wait time.
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  IWDG_SetReload(5000);
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
  IWDG_ReloadCounter();

  // Return the save to be re-loaded later.
  return save;
}

void resumeIWDG(struct IWDG_Values save) {
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(save.prescaler);
  IWDG_SetReload(save.reload);
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
  IWDG_ReloadCounter();
}
