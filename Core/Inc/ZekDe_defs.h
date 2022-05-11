#ifndef ZEKDE_DEFS_H
#define ZEKDE_DEFS_H

#include "stdint.h"

#define SET_BIT_POS(reg, pos)		((reg) |= 1UL << (pos))
#define RESET_BIT_POS(reg, pos)	((reg) &= ~(1UL << (pos)))
#define READ_BIT_POS(reg, pos)		(((reg) >> (pos)) & 1UL)
#define TOGGLE_BIT_POS(reg, pos)	((reg) ^= 1UL << (pos))
#define MODIFY_BIT_POS(reg, pos, _0_1)	((reg) ^= (-_0_1 ^ (reg)) & (1UL << (pos)))
//#define MODIFY_BIT_POS(reg, pos, _0_1)	((reg) & ~(1UL << (pos)) | (_0_1 << (pos)))

#define PERCENT_ERROR(goal, actual)		(100 * ((goal) - (actual)) / (goal))
/**
 * \def __SIGNUM
 * \brief
 * if x < 0 -> -1
 * if x = 0 -> 0
 * if x > 0 -> 1
 * \return -1, 0, 1
 */
#define SIGNUM(x)				((x) > 0) - ((x) < 0)
#define IS_ODD_NUMBER(x)		(((x) & 1) != 0)
#define IS_POWEROF2(x)			(x) && !((x) & ((x)-1))

/**
 * \def MIN(x, y) and MAX(x, y
 * \brief get the minimum of int x and y
 * 
 */
#define MIN(x, y)			(y) ^ (((x) ^ (y)) & -((x) < (y)))
#define MAX(x, y)			(x) ^ (((x) ^ (y)) & -((x) < (y)))

#define SWAP_BYTE(x)			(x) = ( (((x) & 0x0F) << 4) | (((x) & 0xF0) >> 4) )

/**
 * \def MOVE(en, out, in)
 * \brief To transfer the content of the in to the out in accordance the en
 * en state '1' is active
 */
#define MOVE(en, out, in)		(en) ? ((out) = (in)): 0

/**
 * \def F2S
 * \brief float to single precision
 *
 */
#define F2S(x)	*((uint32_t*)(&(x)))

/**
 * \def S2F
 * \brief single precision to float
 *
 */
#define S2F(x)	*((float*)&(x))

/**
 * \def IS_LITTLE_ENDIAN
 * \brief check the system if it is little endian
 *
 */
#define IS_LITTLE_ENDIAN() (1 == *(uint8_t *)&(const int){1})

/**
 * \def HIGH8
 * \brief high 8 bit of an 16 bit for little endian
 * \example
 * uint16_t num = 0xFFFA;
 * uint8_t high = 0;
 * high = HIGH8(num);
 */
#define HIGH8(x)	*(((uint8_t*)&(x)) + 1)
/**
 * \def LOW8
 * \brief low 8 bit of 16 bit
 * \example
 * uint8_t low = 0;
 * uint16_t num = 0xFFFA;
 * low = LOW8(num);
 *
 */
#define LOW8(x)		*((uint8_t*)&(x))

/**
 * \def PARSE_BYTES
 * \brief
 * \example
 *
 *  uint8_t *pb = NULL;
 *  float *pfVal = NULL;
 *  float fVal = 1.235;
 *  PARSE_BYTES(pb, fVal);
 *  MERGE_BYTES(pfVal, pb, float);
 *
 */
#define PARSE_BYTES(p, val)	(p) = (uint8_t*)&(val)
#define MERGE_BYTES(val, x, type)	(val) = (type*)(x)

#define SIZEX(x)	(sizeof((x)) / sizeof(*(x)))

#define CLEAR_STRUCT(s)	memset(&s, 0, sizeof(s));

typedef struct
{
	uint32_t bit0:1U;
	uint32_t bit1:1U;
	uint32_t bit2:1U;
	uint32_t bit3:1U;
	uint32_t bit4:1U;
	uint32_t bit5:1U;
	uint32_t bit6:1U;
	uint32_t bit7:1U;
}byte_t;


#endif /* MYDEF_H */
