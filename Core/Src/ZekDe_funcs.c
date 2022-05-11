#include "ZekDe_defs.h"
#include "ZekDe_funcs.h"


/**
 * \def make_sumFunc makes functions of type t
 * \brief example usage: make_func(int) creates a
 * int sum_int(int a, int b) function
 *
 *
 */
#define tFuncTemplate(t)	t func_##t(t a, t b) \
{\
	return (t)(a + b); \
}


/**
 * \fn void risingEdgeDetection_withCB(uint8_t, void(*)(void))
 * \brief - state of 'catch' changes from 0 to 1
 * callback function is called
 * \param catch - related value to catch
 * \param cb - callback function
 */
void vRisingEdgeDetection(rising_edge_detection_t *s, _Bool oCatch, void(*cb)(void))
{
	if (oCatch)
	{
		if (!s->o_aux)
		{
			s->o_aux = true;
			(*cb)();
		}
	}
	else
	{
		s->o_aux = false;
	}
}

/**
 * \fn uint8_t risingEdgeDetection(uint8_t)
 * \brief - state of 'catch' changes from 0 to 1
 * and return value remains 1 until which is read
 * \param catch - related value to catch
 * \return - 1 is caught
 */
_Bool oRisingEdgeDetection(rising_edge_detection_t *s, _Bool oCatch)
{
	if (oCatch)
	{
		if (!s->o_aux)
		{
			s->o_aux = true;
			return true;
		}
	}
	else
	{
		s->o_aux = false;
	}
	return false;
}

/**
 * @brief to activate the seal, "breakTheSeal" should be 0 and after the "seal" is 1, "out" is going to be 1.
 * To break the seal, "breakTheSeal" should be 1
 * @param seal - variable for sealing
 * @param breakTheSeal - variable that break the sealing
 * @param out - result is 1 or 0
 */
void seal(_Bool oSeal, _Bool oBreakTheSeal, _Bool *poOut)
{
	*poOut = (((oSeal) || (*poOut)) && !oBreakTheSeal);
}

/**
 * @brief SR FlipFlop Q(t+1) = S + R'Q(t) characteristic equation, reset priority
 * if S and R are 1, Q is 0
 * @param S
 * @param R
 * @param Q
 */
void SR(_Bool oS, _Bool oR, _Bool *poQ)
{
	(oS && oR) ? *poQ = false : (*poQ = oS || (!oR && *poQ));
}

float map(float fx, float fInMin, float fInMax,
			   float fOutMin, float fOutMax)
{
	return (fx - fInMin) * (fOutMax - fOutMin) / (fInMax - fInMin) + fOutMin;
}

uint8_t getNumOfBitsSet(uint32_t dwVal)
{
   uint8_t i = 0;
   for (i = 0; dwVal; dwVal >>= 1)
   {
      i += dwVal & 1;
   }
   return i;
}
uint8_t getNumOfBitsSet_32bits(uint32_t dwVal)	
{
   dwVal = dwVal - ((dwVal >> 1) & 0x55555555);
   dwVal = (dwVal & 0x33333333) + ((dwVal >> 2) & 0x33333333);
   return ((dwVal + (dwVal >> 4) & 0xF0F0F0F) * 0x1010101) >> 24;
}
float norm(const float *vec, uint32_t size)
{
	float X;
	float scale;
	float absxk;
	float t;

	X = 0.0;
	scale = 1.175494e-38;

	for (uint32_t k = 0; k < size; k++)
	{
		absxk = fabs(vec[k]);
		if (absxk > scale)
		{
			t = scale / absxk;
			X = X * t * t + 1.0;
			scale = absxk;
		}
		else
		{
			t = absxk / scale;
			X += t * t;
		}
	}

	return scale * sqrt(X);
}

char* toString(enum colors value)
{
    switch (value) {
        #define X(color) \
            case color:  \
                return #color;
                COLORS;
        #undef X
    }
}
