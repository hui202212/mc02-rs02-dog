/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    arm_lib_stubs.c
  * @brief   ARM C library stubs for bare-metal builds.
  ******************************************************************************
  */
/* USER CODE END Header */

#if defined(__ARMCC_VERSION)
#if (__ARMCC_VERSION >= 6000000)
/* Older ArmClang -O0 builds may not emit this marker from main(void). */
#ifndef __MICROLIB
__asm(".weak __ARM_use_no_argv\n"
      "__ARM_use_no_argv:\n");
#endif
__asm(".global __use_no_semihosting\n");
#else
#pragma import(__use_no_semihosting)
#endif
#endif

#include <rt_sys.h>

void _ttywrch(int ch)
{
  (void)ch;
}

char *_sys_command_string(char *cmd, int len)
{
  (void)cmd;
  (void)len;

  return (char *)0;
}

__attribute__((noreturn)) void _sys_exit(int return_code)
{
  (void)return_code;

  for (;;)
  {
  }
}
