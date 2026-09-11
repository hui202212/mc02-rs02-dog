#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* The only robot-specific interface used by the real-time scheduler. */
void robot_app_init(void);
void robot_app_step_1ms(void);

#ifdef __cplusplus
}
#endif

#endif
