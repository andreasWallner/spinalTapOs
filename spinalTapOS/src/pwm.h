#ifndef spinaltap_pwm_h
#define spinaltap_pwm_h

#include <stdint.h>

struct pwm_reg_t {
	uint32_t ctrl;
	uint32_t prescaler;
	uint32_t max;
	uint32_t level[3];
};

#define PWM ((struct pwm_reg_t *)0x43c00100)

#define PWM_CTRL_RUN UINT32_C(1)

#endif
