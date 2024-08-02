#ifndef INC_ADF41020_H_
#define INC_ADF41020_H_

#include "main.h"
#include <stdint.h>

/*** Redefine if necessary ***/
#define ADF41020_SPI_PORT 		hspi1
extern SPI_HandleTypeDef 		ADF41020_SPI_PORT;

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Channels */
#define ADF41020_RX_CHANNEL	0
#define ADF41020_TX_CHANNEL	1


/* COUNTER LATCH */
#define ADF41020_R_COUNTER	0
#define ADF41020_N_COUNTER	1
#define ADF41020_F_LATCH	2

/* R Bit Definitions */
#define ADF41020_R_MOD(x)					(((x) & 0xFFF) << 3)
#define ADF41020_R_PHASE(x)					(((x) & 0xFFF) << 15)
#define ADF41020_R_PRESCALER				(1 << 27)

/*F_LATCH*/
#define ADF41020_F_LATCH_COUNTER_RESET_EN			(1 << 2)
#define ADF41020_F_LATCH_POWER_DOWN_1_EN			(1 << 3)
#define ADF41020_F_LATCH_MUXOUT(x)					((x) << 4)
#define ADF41020_F_LATCH_PD_POLARITY_POS			(1 << 7)
#define ADF41020_F_LATCH_CP_THREESTATE_EN			(1 << 8)
#define ADF41020_F_LATCH_FAST_LOCK_EN				(1 << 9)
#define ADF41020_F_LATCH_FAST_LOCK_MODE				(1 << 10)
#define ADF41020_F_LATCH_FAST_TIMER_CNT_CNTR(x)		((x) << 11)
#define ADF41020_F_LATCH_FAST_CURRENT_SETTING_1(x)	((x) << 15)
#define ADF41020_F_LATCH_FAST_CURRENT_SETTING_2(x)	((x) << 18)
#define ADF41020_F_LATCH_POWER_DOWN_2_EN			(1 << 21)
#define ADF41020_F_LATCH_PRESCALLER_VALUE(x)		((x) << 22)

/* Specifications */
#define ADF41020_MAX_OUT_FREQ		3500000000ULL /* Hz */
#define ADF41020_MIN_OUT_FREQ		34375000 /* Hz */
#define ADF41020_MIN_VCO_FREQ		2200000000ULL /* Hz */
#define ADF41020_MAX_FREQ_45_PRESC	3000000000ULL /* Hz */
#define ADF41020_MAX_FREQ_PFD		32000000 /* Hz */
#define ADF41020_MAX_BANDSEL_CLK		125000 /* Hz */
#define ADF41020_MAX_FREQ_REFIN		250000000 /* Hz */
#define ADF41020_MAX_MODULUS			4095
#define ADF41020_MAX_R_CNT			1023

/******************************************************************************/
/************************ Types Definitions ***********************************/
/******************************************************************************/
struct adf41020_platform_data
{
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint64_t	power_up_frequency;

	uint16_t	ref_div_factor; /* 10-bit R counter */
	uint8_t	    ref_doubler_en;
	uint8_t	    ref_div2_en;

	int32_t	    gpio_lock_detect;
};

typedef struct
{
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint32_t	power_up_frequency;
	uint32_t	reference_div_factor;
	uint8_t		reference_doubler_enable;
	uint8_t		reference_div2_enable;

}adf41020_init_param;

void SPI_Write(uint8_t data);

int32_t adf41020_write(uint32_t data);

int32_t adf41020_out_altvoltage0_powerdown(int32_t pwd);

int32_t adf41020_setup(adf41020_init_param init_param);


#endif /* INC_ADF41020_H_ */
