#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus

extern "C"
{
#endif
	
void control_task_init();
void control_task();
extern volatile uint32_t imu_poll_count_watch;
#ifdef __cplusplus
};

#endif

#endif
