#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#include "stdint.h"
#include "stdbool.h"

/**
 * \def TIME_OVER
 * \brief check if time over in max 24.8 days acc.to ms
 *
 */
#define TIME_OVER(target, time) (uint32_t)((time) - (target) < 0x80000000UL)

typedef struct
{
	_Bool oIn;
	_Bool oAux;
	uint32_t dwInterval;
	uint32_t dwSince;
} softTimer_t;

typedef struct
{
	_Bool oAux;
	uint32_t dwSince;
} ton_t;

void timerCheck(softTimer_t *s, const uint32_t *pdwNow, void(*cb)(void));
void timerStart(softTimer_t *s);
void timerStop(softTimer_t *s);
void timerSetInterval(softTimer_t *s, uint32_t dwinterval);
_Bool TON(ton_t *s, _Bool oIn, const uint32_t *pdwNow, uint32_t dwPresetTime);


#endif /* SOFT_TIMER_H */
