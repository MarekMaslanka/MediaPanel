#include "vs1003.h"

extern SPI_HandleTypeDef hspi1;

uint8_t SPIPutChar(uint8_t outB)
{
	HAL_SPI_TransmitReceive(&hspi1, &outB, &outB, 1, 100);
	return outB;
}

void ControlReset(uint8_t State)
{
	HAL_GPIO_WritePin(XRESET_PORT, XRESET_PIN, !State);
}

void SCI_ChipSelect(uint8_t State)
{
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, !State);
}

void SDI_ChipSelect(uint8_t State)
{
	HAL_GPIO_WritePin(XDCS_PORT, XDCS_PIN, !State);
}

void WriteRegister(uint8_t addressbyte, uint8_t highbyte, uint8_t lowbyte)
{
	SDI_ChipSelect(RESET);
	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;
	SCI_ChipSelect(SET);
	SPIPutChar(VS_WRITE_COMMAND);
	SPIPutChar(addressbyte);
	SPIPutChar(highbyte);
	SPIPutChar(lowbyte);
	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;
	SCI_ChipSelect(RESET);
}

uint16_t ReadRegister(uint8_t addressbyte)
{
	uint16_t result;
	SDI_ChipSelect(RESET);
	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;
	SCI_ChipSelect(SET);
	SPIPutChar(VS_READ_COMMAND);
	SPIPutChar(addressbyte);
	result = SPIPutChar(0) << 8;
	result |= SPIPutChar(0);
	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;
	SCI_ChipSelect(RESET);
	return result;
}

void ResetChip()
{
	ControlReset(SET);
	osDelay(10);
	SPIPutChar(0xff);
	SCI_ChipSelect(RESET);
	SDI_ChipSelect(RESET);
	ControlReset(RESET);
	osDelay(10);

	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;
	osDelay(10);
}

uint16_t MaskAndShiftRight(uint16_t Source, uint16_t Mask, uint16_t Shift)
{
	return ((Source & Mask) >> Shift);
}

void VS1003_Start()
{
	ControlReset(SET);
	osDelay(100);
	SPIPutChar(0xFF);
	SCI_ChipSelect(RESET);
	SDI_ChipSelect(RESET);
	ControlReset(RESET);
	VS1003_SoftwareReset();
	osDelay(100);

	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;

	WriteRegister(SPI_MODE, 0x08, 0x00);
	WriteRegister(SPI_CLOCKF, 0x98, 0x00);
	WriteRegister(SPI_AUDATA, 0xAC, 0x45);
	WriteRegister(SPI_BASS, 0x08, 0x00);
	WriteRegister(SPI_VOL, 0x0B, 0x0B);
	WriteRegister(SPI_STATUS, 0, 0b00110011);

	while (HAL_GPIO_ReadPin(DREQ_PORT, DREQ_PIN) == 0)
		;
}

void VS1003_SoftwareReset()
{
	WriteRegister(SPI_MODE, 0x00, 0x04);
}

uint8_t VS1003_GetVolume()
{
	return (ReadRegister(SPI_VOL) & 0x00FF);
}
/**
 * Function sets the same volume level to both channels.
 * @param xMinusHalfdB describes damping level as a multiple
 * 		of 0.5dB. Maximum volume is 0 and silence is 0xFEFE.
 */
void VS1003_SetVolume(uint8_t xMinusHalfdB)
{
	WriteRegister(SPI_VOL, xMinusHalfdB, xMinusHalfdB);
}

/**
 * Function increases volume level for both channels.
 * If it is impossible to increase volume as much as
 * xHalfdB, volume is set to maximum.
 * @param xHalfdB multiple of 0.5dB describing how
 * 		much volume should be turned up.
 */
void VS1003_VolumeUp(uint8_t xHalfdB)
{
	uint8_t currentVol = VS1003_GetVolume();

	//if it is impossible to turn volume up as we want
	if ((uint8_t) (currentVol - xHalfdB) > currentVol)
		VS1003_SetVolume(0);
	else
		VS1003_SetVolume(currentVol - xHalfdB);
}

/**
 * Function decreases volume level for both channels.
 * @note If it is impossible to decrease volume as much as
 * xHalfdB, volume is muted.
 * @param xHalfdB multiple of 0.5dB describing how
 * 		much volume should be turned down.
 */
void VS1003_VolumeDown(uint8_t xHalfdB)
{
	uint8_t currentVol = VS1003_GetVolume();

	//if it is impossible to turn volume down as we want
	if (currentVol + xHalfdB < currentVol || currentVol + xHalfdB == 255)
		VS1003_SetVolume(0xFE);
	else
		VS1003_SetVolume(currentVol + xHalfdB);
}

/**
 * Functions returns level of treble enhancement.
 * @return Returned value describes enhancement in multiplies
 * 		of 1.5dB. 0 value means no enhancement, 8 max (12dB).
 */
uint8_t VS1003_GetTreble()
{
	return ((ReadRegister(SPI_BASS) & 0xF000) >> 12);
}

/**
 * Sets treble level.
 * @note If xOneAndHalfdB is greater than max value, sets treble
 * 		to maximum.
 * @param xOneAndHalfdB describes level of enhancement. It is a multiplier
 * 		of 1.5dB. 0 - no enhancement, 8 - maximum, 12dB.
 * @return void
 */
void VS1003_SetTreble(uint8_t xOneAndHalfdB)
{
	uint16_t bassReg = ReadRegister(SPI_BASS);
	if (xOneAndHalfdB <= 8)
		WriteRegister( SPI_BASS,
				MaskAndShiftRight(bassReg, 0x0F00, 8) | (xOneAndHalfdB << 4),
				bassReg & 0x00FF);
	else
		WriteRegister( SPI_BASS,
				MaskAndShiftRight(bassReg, 0x0F00, 8) | 0b10000000,
				bassReg & 0x00FF);
}

/**
 * Turns up treble.
 * @note If xOneAndHalfdB is greater than max value, sets treble
 * 		to maximum.
 * @param xOneAndHalfdB describes how many dBs add to current treble level.
 *  	It is a multiplier of 1.5dB.
 * @return void
 */
void VS1003_TrebleUp(uint8_t xOneAndHalfdB)
{
	uint8_t currentTreble = VS1003_GetTreble();

	if ((uint8_t) (currentTreble - xOneAndHalfdB) > currentTreble)
		VS1003_SetTreble(0);
	else
		VS1003_SetTreble(currentTreble - xOneAndHalfdB);
}

/**
 * Turns down treble.
 * @note If it is impossible to decrease by xdB, the minimum value is set (off).
 * @param xOneAndHalfdB describes how many dBs subtract from current treble level.
 *  	It is a multiplier of 1.5dB.
 * @return void
 */
void VS1003_TrebleDown(uint8_t xOneAndHalfdB)
{
	uint8_t currentTreble = VS1003_GetTreble();

	if (currentTreble + xOneAndHalfdB >= 8)
		VS1003_SetTreble(8);
	else
		VS1003_SetTreble(currentTreble + xOneAndHalfdB);
}
/**
 * Sets low limit frequency of treble enhancer.
 * @note new frequency is set only if argument is valid.
 * @param xkHz The lowest frequency enhanced by treble enhancer.
 * 		Values from 0 to 15 (in kHz)
 * @return void
 */
void VS1003_SetTrebleFreq(uint8_t xkHz)
{
	uint16_t bassReg = ReadRegister(SPI_BASS);
	if (xkHz <= 15)
		WriteRegister( SPI_BASS, MaskAndShiftRight(bassReg, 0xF000, 8) | xkHz,
				bassReg & 0x00FF);
}

/**
 * Returns level of bass boost in dB.
 * @return Value of bass enhancement from 0 (off) to 15(dB).
 */
uint8_t VS1003_GetBass()
{
	return ((ReadRegister(SPI_BASS) & 0x00F0) >> 4);
}

/**
 * Sets bass enhancement level (in dB).
 * @note If xdB is greater than max value, bass enhancement is set to its max (15dB).
 * @param xdB Value of bass enhancement from 0 (off) to 15(dB).
 * @return void
 */
void VS1003_SetBass(uint8_t xdB)
{
	uint16_t bassReg = ReadRegister(SPI_BASS);
	if (xdB <= 15)
		WriteRegister(SPI_BASS, (bassReg & 0xFF00) >> 8,
				(bassReg & 0x000F) | (xdB << 4));
	else
		WriteRegister(SPI_BASS, (bassReg & 0xFF00) >> 8,
				(bassReg & 0x000F) | 0xF0);
}

/**
 * Increases level of bass enhancement.
 * @note If it is impossible to increase by xdB, the maximum value is set.
 * @param xdB Value of bass enhancement from 0 (off) to 15(dB).
 */
void VS1003_BassUp(uint8_t xdB)
{
	uint8_t currentBass = VS1003_GetBass();

	if (currentBass + xdB >= 15)
		VS1003_SetBass(15);
	else
		VS1003_SetBass(currentBass + xdB);
}

/**
 * Decreases level of bass enhancement.
 * @note If it is impossible to decrease by xdB, the minimum value is set.
 * @param xdB Value of bass enhancement from 0 (off) to 15(dB).
 */
void VS1003_BassDown(uint8_t xdB)
{
	uint8_t currentBass = VS1003_GetBass();
	if (currentBass - xdB > currentBass)
		VS1003_SetBass(0);
	else
		VS1003_SetBass(currentBass - xdB);
}

/**
 * Sets low limit frequency of bass enhancer.
 * @note new frequency is set only if argument is valid.
 * @param xTenHz The lowest frequency enhanced by bass enhancer.
 * 		Values from 2 to 15 ( equal to 20 - 150 Hz).
 * @return void
 */
void VS1003_SetBassFreq(uint8_t xTenHz)
{
	uint16_t bassReg = ReadRegister(SPI_BASS);
	if (xTenHz >= 2 && xTenHz <= 15)
		WriteRegister(SPI_BASS, MaskAndShiftRight(bassReg, 0xFF00, 8),
				(bassReg & 0x00F0) | xTenHz);
}

uint16_t VS1003_GetDecodeTime()
{
	return ReadRegister(SPI_DECODE_TIME);
}

uint16_t VS1003_GetBitrate()
{
	uint16_t bitrate = (ReadRegister(SPI_HDAT0) & 0b1111000000000000) >> 12;
	uint8_t ID = (ReadRegister(SPI_HDAT1) & 0b0000000000011000) >> 3;
	uint16_t res;
	if (ID == 3)
	{
		res = 32;
		while (bitrate > 13)
		{
			res += 64;
			bitrate--;
		}
		while (bitrate > 9)
		{
			res += 32;
			bitrate--;
		}
		while (bitrate > 5)
		{
			res += 16;
			bitrate--;
		}
		while (bitrate > 1)
		{
			res += 8;
			bitrate--;
		}
	}
	else
	{
		res = 8;

		while (bitrate > 8)
		{
			res += 16;
			bitrate--;
		}
		while (bitrate > 1)
		{
			res += 8;
			bitrate--;
		}
	}
	return res;
}

uint16_t VS1003_GetSampleRate()
{
	return (ReadRegister(SPI_AUDATA) & 0xFFFE);
}
