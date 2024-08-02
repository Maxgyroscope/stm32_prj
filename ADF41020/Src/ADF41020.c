/*
 * ADF41020.c
 *
 *  Created on: Jul 14, 2024
 *      Author: MaksimS
 */

#include "ADF41020.h"
#include "stm32f1xx_hal.h"

struct adf41020_platform_data platdata;

/************************
 *  @brief write 8 bits of data on SPI interface MOSI pin..
 */
void SPI_Write(uint8_t data) {
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
}

/******************************************************************************/
/************************ Local variables and types ***************************/
/******************************************************************************/
static struct adf41020_state
{
	struct adf41020_platform_data	*pdata;
	uint32_t	clkin;
	uint32_t	chspc;	/* Channel Spacing */
	uint32_t	fpfd;	/* Phase Frequency Detector */
	uint32_t	regs[6];
	uint32_t	regs_hw[6];
	uint32_t 	val;
} adf41020_st;


/***************************************************************************//**
 * @brief Writes 4 bytes of data to ADF41020.
 *
 * @param data - Data value to write.
 *
 * @return Returns 0 in case of success or negative error code..
*******************************************************************************/
int32_t adf41020_write(uint32_t data)
{
	uint8_t txData[4];
	txData[0] = (data & 0xFF000000) >> 24;
	txData[1] = (data & 0x00FF0000) >> 16;
	txData[2] = (data & 0x0000FF00) >> 8;
	txData[3] = (data & 0x000000FF) >> 0;

	SPI_Write(txData[0]);
	SPI_Write(txData[1]);
	SPI_Write(txData[2]);
	SPI_Write(txData[3]);

	return 0;
}

/***************************************************************************//**
 * @brief Updates the registers values.
 *
 * @param st - The selected structure.
 *
 * @return Returns 0 in case of success or negative error code.
*******************************************************************************/
int32_t adf41020_sync_config(struct adf41020_state *st)
{
	int32_t ret, i, doublebuf = 0;

	for (i = ADF41020_R_COUNTER; i >= ADF41020_R_COUNTER; i--)
	{
		if ((st->regs_hw[i] != st->regs[i]) ||
			((i == ADF41020_R_COUNTER) && doublebuf))
		{
			switch (i)
			{
				case ADF41020_R_COUNTER:
				case ADF41020_N_COUNTER:
					doublebuf = 1;
					break;
			}

			st->val = (st->regs[i] | i);
			ret = adf41020_write(st->val);
			if (ret < 0)
				return ret;
			st->regs_hw[i] = st->regs[i];
		}
	}

	return 0;
}


/***************************************************************************//**
 * @brief Powers down the PLL.
 *
 * @param pwd - Power option.
 *				Example: 0 - Power up the PLL.
 *						 1 - Power down the PLL.
 *
 * @return Returns the PLL's power status.
*******************************************************************************/
int32_t adf41020_out_altvoltage0_powerdown(int32_t pwd)
{
	struct adf41020_state *st = &adf41020_st;

	if(pwd == 1)
	{
		st->regs[ADF41020_F_LATCH] |= ADF41020_F_LATCH_POWER_DOWN_1_EN;
		adf41020_sync_config(st);
	}
	if(pwd == 0)
	{
		st->regs[ADF41020_F_LATCH] &= ~ADF41020_F_LATCH_POWER_DOWN_1_EN;
		adf41020_sync_config(st);
	}


	return (st->regs[ADF41020_F_LATCH] & ADF41020_F_LATCH_POWER_DOWN_1_EN);
}

/***************************************************************************//**
 * @brief Sets the ADF41020 frequency.
 *
 * @param st   - The selected structure.
 * @param freq - The desired frequency value.
 *
 * @return calculatedFrequency - The actual frequency value that was set.
*******************************************************************************/
int64_t adf41020_set_freq(struct adf41020_state *st, uint64_t freq)
{
	struct adf41020_platform_data *pdata = st->pdata;
	uint64_t tmp;
	uint32_t div_gcd, prescaler, chspc;
	uint16_t mdiv, r_cnt = 0;
	uint8_t band_sel_div;
	int32_t ret;

	if ((freq > ADF41020_MAX_OUT_FREQ) || (freq < ADF41020_MIN_OUT_FREQ))
		return -1;

	if (freq > ADF41020_MAX_FREQ_45_PRESC) {
		prescaler = ADF41020_R_PRESCALER;
		mdiv = 75;
	}
	else
	{
		prescaler = 0;
		mdiv = 23;
	}


	if (pdata->ref_div_factor)
		r_cnt = pdata->ref_div_factor - 1;

	chspc = st->chspc;


	band_sel_div = st->fpfd % ADF41020_MAX_BANDSEL_CLK > ADF41020_MAX_BANDSEL_CLK / 2 ?
					st->fpfd / ADF41020_MAX_BANDSEL_CLK + 1 :
					st->fpfd / ADF41020_MAX_BANDSEL_CLK;

	st->regs[ADF41020_F_LATCH] =
		ADF41020_F_LATCH_PD_POLARITY_POS |
		ADF41020_F_LATCH_MUXOUT(0x7);

	ret = adf41020_sync_config(st);
}

int32_t adf41020_setup(adf41020_init_param init_param)
{
	struct adf41020_state *st = &adf41020_st;

	st->pdata = &platdata; // static assign memory.

	if (!st->pdata)
		return -1;

	st->pdata->clkin = init_param.clkin;

	st->pdata->channel_spacing = init_param.channel_spacing;

	st->pdata->power_up_frequency = init_param.power_up_frequency;
	st->pdata->ref_div_factor = init_param.reference_div_factor;
	st->pdata->ref_doubler_en = init_param.reference_doubler_enable;
	st->pdata->ref_div2_en = init_param.reference_div2_enable;


}
