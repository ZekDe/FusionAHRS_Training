#include "soft_timer.h"

#define TIME_PASSED(now, since)	(now >= since) ? (now - since) : (now + (UINT32_MAX - since))

/**
 * \fn void timeoutCheck(timeout_t *s, const uint32_t *pdwNow, void(*cb)(void))
 * \brief Start a periodic timer with a specified duration .
 *
 * \param t - variables are stored which function needs
 * \param now - system tick continuously running
 * \param cb - callback function is called when the timer expired
 */
void timerCheck(softTimer_t *s, const uint32_t *pdwNow, void(*cb)(void))
{
	if(s->oIn)
	{
		if(!s->oAux)
 		{
 			s->dwSince = *pdwNow;
 			s->oAux = true;
 		}
		else if(TIME_OVER(s->dwSince + s->dwInterval, *pdwNow))
		{
				s->dwSince = *pdwNow;
				(*cb)();
		}
		else
		{}
	}
	else
	{
		s->oAux = false;
	}
}

/**
 * \fn void timeoutStart(timeout_t*, uint32_t)
 * \brief
 * \param s
 * \param interval
 */
void timerStart(softTimer_t *s)
{
	s->oIn = true;
}

/**
 * \fn void timeoutStop(timeout_t*)
 * \brief
 * \param s
 */
void timerStop(softTimer_t *s)
{
	s->oIn = false;
}

/**
 * \fn void timeoutSetInterval(timeout_t*, uint32_t)
 * \brief
 * \param s
 * \param dwinterval
 */
void timerSetInterval(softTimer_t *s, uint32_t dwinterval)
{
	s->dwInterval = dwinterval;
}

/**
 * \fn uint8_t TON(ton_t *s, _Bool oIn, const uint32_t *pdwNow, uint32_t dwPresetTime)
 * \brief Start a timer with a specified duration as on-delay.
 * \param t - variables are stored which function needs
 * \param in - timer is executed when the "in" state changes from 0 to 1
 * \param now - system tick continuously running
 * \param presetTime - timer is started for the time stored in
 * \return if time is over , return value is 1
 */
_Bool TON(ton_t *s, _Bool oIn, const uint32_t *pdwNow, uint32_t dwPresetTime)
{
	if(oIn)
	{
		if(!s->oAux)
 		{
			s->dwSince = *pdwNow;
			s->oAux = true;
 		}
		else if(TIME_OVER(s->dwSince + dwPresetTime, *pdwNow))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		s->oAux = false;
	}

	return false;
}
