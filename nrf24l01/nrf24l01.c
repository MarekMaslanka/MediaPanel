#include "nrf24l01.h"

/* NRF24L01+ registers*/
#define NRF24L01_REG_CONFIG			0x00	//Configuration Register
#define NRF24L01_REG_EN_AA			0x01	//Enable Auto Acknowledgment Function
#define NRF24L01_REG_EN_RXADDR		0x02	//Enabled RX Addresses
#define NRF24L01_REG_SETUP_AW		0x03	//Setup of Address Widths (common for all data pipes)
#define NRF24L01_REG_SETUP_RETR		0x04	//Setup of Automatic Retransmission
#define NRF24L01_REG_RF_CH			0x05	//RF Channel
#define NRF24L01_REG_RF_SETUP		0x06	//RF Setup Register
#define NRF24L01_REG_STATUS			0x07	//Status Register
#define NRF24L01_REG_OBSERVE_TX		0x08	//Transmit observe register
#define NRF24L01_REG_RPD			0x09
#define NRF24L01_REG_RX_ADDR_P0		0x0A	//Receive address data pipe 0. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P1		0x0B	//Receive address data pipe 1. 5 Bytes maximum length.
#define NRF24L01_REG_RX_ADDR_P2		0x0C	//Receive address data pipe 2. Only LSB
#define NRF24L01_REG_RX_ADDR_P3		0x0D	//Receive address data pipe 3. Only LSB
#define NRF24L01_REG_RX_ADDR_P4		0x0E	//Receive address data pipe 4. Only LSB
#define NRF24L01_REG_RX_ADDR_P5		0x0F	//Receive address data pipe 5. Only LSB
#define NRF24L01_REG_TX_ADDR		0x10	//Transmit address. Used for a PTX device only
#define NRF24L01_REG_RX_PW_P0		0x11
#define NRF24L01_REG_RX_PW_P1		0x12
#define NRF24L01_REG_RX_PW_P2		0x13
#define NRF24L01_REG_RX_PW_P3		0x14
#define NRF24L01_REG_RX_PW_P4		0x15
#define NRF24L01_REG_RX_PW_P5		0x16
#define NRF24L01_REG_FIFO_STATUS	0x17	//FIFO Status Register
#define NRF24L01_REG_DYNPD			0x1C	//Enable dynamic payload length
#define NRF24L01_REG_FEATURE		0x1D

/* Registers default values */
#define NRF24L01_REG_DEFAULT_VAL_CONFIG			0x08
#define NRF24L01_REG_DEFAULT_VAL_EN_AA			0x3F
#define NRF24L01_REG_DEFAULT_VAL_EN_RXADDR		0x03
#define NRF24L01_REG_DEFAULT_VAL_SETUP_AW		0x03
#define NRF24L01_REG_DEFAULT_VAL_SETUP_RETR		0x03
#define NRF24L01_REG_DEFAULT_VAL_RF_CH			0x02
#define NRF24L01_REG_DEFAULT_VAL_RF_SETUP		0x0E
#define NRF24L01_REG_DEFAULT_VAL_STATUS			0x0E
#define NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX		0x00
#define NRF24L01_REG_DEFAULT_VAL_RPD			0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4	0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4	0xC2
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2		0xC3
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3		0xC4
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4		0xC5
#define NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5		0xC6
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3		0xE7
#define NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4		0xE7
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P0		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P1		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P2		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P3		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P4		0x00
#define NRF24L01_REG_DEFAULT_VAL_RX_PW_P5		0x00
#define NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS	0x11
#define NRF24L01_REG_DEFAULT_VAL_DYNPD			0x00
#define NRF24L01_REG_DEFAULT_VAL_FEATURE		0x00

/* Configuration register*/
#define NRF24L01_MASK_RX_DR		6
#define NRF24L01_MASK_TX_DS		5
#define NRF24L01_MASK_MAX_RT	4
#define NRF24L01_EN_CRC			3
#define NRF24L01_CRCO			2
#define NRF24L01_PWR_UP			1
#define NRF24L01_PRIM_RX		0

/* Enable auto acknowledgment*/
#define NRF24L01_ENAA_P5		5
#define NRF24L01_ENAA_P4		4
#define NRF24L01_ENAA_P3		3
#define NRF24L01_ENAA_P2		2
#define NRF24L01_ENAA_P1		1
#define NRF24L01_ENAA_P0		0

/* Enable rx addresses */
#define NRF24L01_ERX_P5			5
#define NRF24L01_ERX_P4			4
#define NRF24L01_ERX_P3			3
#define NRF24L01_ERX_P2			2
#define NRF24L01_ERX_P1			1
#define NRF24L01_ERX_P0			0

/* Setup of address width */
#define NRF24L01_AW				0 //2 bits

/* Setup of auto re-transmission*/
#define NRF24L01_ARD			4 //4 bits
#define NRF24L01_ARC			0 //4 bits

/* RF setup register*/
#define NRF24L01_PLL_LOCK		4
#define NRF24L01_RF_DR_LOW		5
#define NRF24L01_RF_DR_HIGH		3
#define NRF24L01_RF_DR			3
#define NRF24L01_RF_PWR			1 //2 bits

/* General status register */
#define NRF24L01_RX_DR			6
#define NRF24L01_TX_DS			5
#define NRF24L01_MAX_RT			4
#define NRF24L01_RX_P_NO		1 //3 bits
#define NRF24L01_TX_FULL		0

/* Transmit observe register */
#define NRF24L01_PLOS_CNT		4 //4 bits
#define NRF24L01_ARC_CNT		0 //4 bits

/* FIFO status*/
#define NRF24L01_TX_REUSE		6
#define NRF24L01_FIFO_FULL		5
#define NRF24L01_TX_EMPTY		4
#define NRF24L01_RX_FULL		1
#define NRF24L01_RX_EMPTY		0

//Dynamic length
#define NRF24L01_DPL_P0			0
#define NRF24L01_DPL_P1			1
#define NRF24L01_DPL_P2			2
#define NRF24L01_DPL_P3			3
#define NRF24L01_DPL_P4			4
#define NRF24L01_DPL_P5			5

/* Transmitter power*/
#define NRF24L01_M18DBM			0 //-18 dBm
#define NRF24L01_M12DBM			1 //-12 dBm
#define NRF24L01_M6DBM			2 //-6 dBm
#define NRF24L01_0DBM			3 //0 dBm

/* Data rates */
#define NRF24L01_2MBPS			0
#define NRF24L01_1MBPS			1
#define NRF24L01_250KBPS		2

/* Configuration */
#define NRF24L01_CONFIG			((1 << NRF24L01_MASK_TX_DS) | (1 << NRF24L01_MASK_MAX_RT) | (1 << NRF24L01_EN_CRC) | (0 << NRF24L01_CRCO))

/* Instruction Mnemonics */
#define NRF24L01_REGISTER_MASK				0x1F

#define NRF24L01_READ_REGISTER_MASK(reg)	(0x00 | (NRF24L01_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address
#define NRF24L01_WRITE_REGISTER_MASK(reg)	(0x20 | (NRF24L01_REGISTER_MASK & reg)) //Last 5 bits will indicate reg. address
#define NRF24L01_R_RX_PAYLOAD_MASK			0x61
#define NRF24L01_W_TX_PAYLOAD_MASK			0xA0
#define NRF24L01_FLUSH_TX_MASK				0xE1
#define NRF24L01_FLUSH_RX_MASK				0xE2
#define NRF24L01_REUSE_TX_PL_MASK			0xE3
#define NRF24L01_ACTIVATE_MASK				0x50
#define NRF24L01_R_RX_PL_WID_MASK			0x60
#define NRF24L01_NOP_MASK					0xFF

extern SPI_HandleTypeDef hspi3;

/* Flush FIFOs */
#define NRF24L01_FLUSH_TX					do { uint8_t data=NRF24L01_FLUSH_TX_MASK; NRF24L01_CSN_LOW; HAL_SPI_Transmit(&hspi3, &data, 1, 100);  NRF24L01_CSN_HIGH; } while (0)
#define NRF24L01_FLUSH_RX					do { uint8_t data=NRF24L01_FLUSH_RX_MASK; NRF24L01_CSN_LOW; HAL_SPI_Transmit(&hspi3, &data, 1, 100);  NRF24L01_CSN_HIGH; } while (0)

#define NRF24L01_TRANSMISSON_OK 			0
#define NRF24L01_MESSAGE_LOST   			1

#define NRF24L01_CHECK_BIT(reg, bit)       (reg & (1 << bit))

#define SPI_OPERATION_WAIT 100

typedef struct
{
	uint8_t payloadSize;
	uint8_t channel;
	nrf24l01OutputPower_t outPower;
	nrf24l01DataRate_t dataRate;
} Nrf20l01Config_t;

/* Private functions */
void nrf24l01InitPins(void);
void nrf24l01WriteBit(uint8_t reg, uint8_t bit, uint8_t value);
uint8_t nrf24l01ReadBit(uint8_t reg, uint8_t bit);
uint8_t nrf24l01ReadRegister(uint8_t reg);
void nrf24l01WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count);
void nrf24l01SoftwareReset(void);
uint8_t nrf24l01RxFifoEmpty(void);

/* NRF structure */
static Nrf20l01Config_t Nrf20l01Config;

void nrf24l01InitPins(void)
{
	/* CSN high = disable SPI */
	NRF24L01_CSN_HIGH;
	/* CE low = disable TX/RX */
	NRF24L01_CE_LOW;
}

uint8_t nrf24l01Init(uint8_t channel, uint8_t payloadSize)
{
	nrf24l01InitPins();
	if (payloadSize > 32)
	{
		payloadSize = 32;
	}
	Nrf20l01Config.channel = !channel;
	Nrf20l01Config.payloadSize = payloadSize;
	Nrf20l01Config.outPower = nrf24l01OutputPower0dBm;
	Nrf20l01Config.dataRate = nrf24l01DataRate250k;
	nrf24l01SoftwareReset();
	nrf24l01SetChannel(channel);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P0, Nrf20l01Config.payloadSize); // Auto-ACK pipe
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P1, Nrf20l01Config.payloadSize); // Data payload pipe
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P2, Nrf20l01Config.payloadSize);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P3, Nrf20l01Config.payloadSize);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P4, Nrf20l01Config.payloadSize);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P5, Nrf20l01Config.payloadSize);

	/* Set RF settings */
	nrf24l01SetRF(Nrf20l01Config.dataRate, Nrf20l01Config.outPower);

	/* Config register */
	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG);

	/* Enable auto-acknowledgment for all pipes */
	nrf24l01WriteRegister(NRF24L01_REG_EN_AA, 0x3F);

	/* Enable RX addresses */
	nrf24l01WriteRegister(NRF24L01_REG_EN_RXADDR, 0x3F);

	/* Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials */
	nrf24l01WriteRegister(NRF24L01_REG_SETUP_RETR, 0x4F);

	/* Dynamic length configurations: No dynamic length */
	nrf24l01WriteRegister(NRF24L01_REG_DYNPD,
			(0 << NRF24L01_DPL_P0) | (0 << NRF24L01_DPL_P1) | (0 << NRF24L01_DPL_P2) | (0 << NRF24L01_DPL_P3) | (0 << NRF24L01_DPL_P4)
					| (0 << NRF24L01_DPL_P5));

	/* Clear FIFOs */
	NRF24L01_FLUSH_TX;
	NRF24L01_FLUSH_RX;

	/* Clear interrupts */
	NRF24L01_CLEAR_INTERRUPTS;

	/* Go to RX mode */
	nrf24l01PowerUpRx();

	/* Return OK */
	return 1;
}

void nrf24l01SetMyAddress(uint8_t *adr)
{
	NRF24L01_CE_LOW;
	nrf24l01WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, adr, 5);
	NRF24L01_CE_HIGH;
}

void nrf24l01SetTxAddress(uint8_t *adr)
{
	nrf24l01WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, adr, 5);
	nrf24l01WriteRegisterMulti(NRF24L01_REG_TX_ADDR, adr, 5);
}

void nrf24l01WriteBit(uint8_t reg, uint8_t bit, uint8_t value)
{
	uint8_t tmp;
	/* Read register */
	tmp = nrf24l01ReadRegister(reg);
	/* Make operation */
	if (value)
	{
		tmp |= 1 << bit;
	}
	else
	{
		tmp &= ~(1 << bit);
	}
	/* Write back */
	nrf24l01WriteRegister(reg, tmp);
}

uint8_t nrf24l01ReadBit(uint8_t reg, uint8_t bit)
{
	uint8_t tmp;
	tmp = nrf24l01ReadRegister(reg);
	if (!NRF24L01_CHECK_BIT(tmp, bit))
	{
		return 0;
	}
	return 1;
}

uint8_t nrf24l01ReadRegister(uint8_t reg)
{
	uint8_t value[] = {NRF24L01_READ_REGISTER_MASK(reg), NRF24L01_NOP_MASK};
	NRF24L01_CSN_LOW;
	HAL_SPI_TransmitReceive(&hspi3, value, value, 2, SPI_OPERATION_WAIT);
	NRF24L01_CSN_HIGH;

	return value[1];
}

void nrf24l01WriteRegister(uint8_t reg, uint8_t value)
{
	uint8_t r[] = {NRF24L01_WRITE_REGISTER_MASK(reg), value};
	NRF24L01_CSN_LOW;
	HAL_SPI_Transmit(&hspi3, r, 2, SPI_OPERATION_WAIT);
	NRF24L01_CSN_HIGH;
}

void nrf24l01WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t count)
{
	uint8_t r = NRF24L01_WRITE_REGISTER_MASK(reg);
	NRF24L01_CSN_LOW;
	HAL_SPI_Transmit(&hspi3, &r, 1, SPI_OPERATION_WAIT);
	HAL_SPI_Transmit(&hspi3, data, count, SPI_OPERATION_WAIT);
	NRF24L01_CSN_HIGH;
}

void nrf24l01PowerUpTx(void)
{
	NRF24L01_CLEAR_INTERRUPTS;
	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | (0 << NRF24L01_PRIM_RX) | (1 << NRF24L01_PWR_UP));
}

void nrf24l01PowerUpRx(void)
{
	/* Disable RX/TX mode */
	NRF24L01_CE_LOW;
	/* Clear RX buffer */
	NRF24L01_FLUSH_RX;
	/* Clear interrupts */
	NRF24L01_CLEAR_INTERRUPTS;
	/* Setup RX mode */
	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_CONFIG | 1 << NRF24L01_PWR_UP | 1 << NRF24L01_PRIM_RX);
	/* Start listening */
	NRF24L01_CE_HIGH;
}

void nrf24l01PowerDown(void)
{
	NRF24L01_CE_LOW;
	nrf24l01WriteBit(NRF24L01_REG_CONFIG, NRF24L01_PWR_UP, GPIO_PIN_RESET);
}

void nrf24l01Transmit(uint8_t *data)
{
	uint8_t reg = NRF24L01_W_TX_PAYLOAD_MASK;

	/* Chip enable put to low, disable it */
	NRF24L01_CE_LOW;

	/* Go to power up tx mode */
	nrf24l01PowerUpTx();

	/* Clear TX FIFO from NRF24L01+ */
	NRF24L01_FLUSH_TX;

	/* Send payload to nRF24L01+ */
	NRF24L01_CSN_LOW;
	/* Send write payload command */
	HAL_SPI_Transmit(&hspi3, &reg, 1, SPI_OPERATION_WAIT);
	/* Fill payload with data*/
	HAL_SPI_Transmit(&hspi3, data, Nrf20l01Config.payloadSize, SPI_OPERATION_WAIT);
	/* Disable SPI */
	NRF24L01_CSN_HIGH;

	/* Send data! */
	NRF24L01_CE_HIGH;
}

void nrf24l01GetData(uint8_t* data)
{
	uint8_t reg = NRF24L01_R_RX_PAYLOAD_MASK;
	/* Pull down chip select */
	NRF24L01_CSN_LOW;
	/* Send read payload command*/
	HAL_SPI_Transmit(&hspi3, &reg, 1, SPI_OPERATION_WAIT);
	/* Read payload */
	HAL_SPI_Transmit(&hspi3, data, Nrf20l01Config.payloadSize, SPI_OPERATION_WAIT);
	/* Pull up chip select */
	NRF24L01_CSN_HIGH;

	/* Reset status register, clear RX_DR interrupt flag */
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, (1 << NRF24L01_RX_DR));
}

uint8_t nrf24l01DataReady(void)
{
	uint8_t status = nrf24l01GetStatus();

	if (NRF24L01_CHECK_BIT(status, NRF24L01_RX_DR))
	{
		return 1;
	}
	return !nrf24l01RxFifoEmpty();
}

uint8_t nrf24l01RxFifoEmpty(void)
{
	uint8_t reg = nrf24l01ReadRegister(NRF24L01_REG_FIFO_STATUS);
	return NRF24L01_CHECK_BIT(reg, NRF24L01_RX_EMPTY);
}

uint8_t nrf24l01GetStatus(void)
{
	uint8_t status;
	uint8_t reg = NRF24L01_NOP_MASK;

	NRF24L01_CSN_LOW;
	/* First received byte is always status register */
	HAL_SPI_TransmitReceive(&hspi3, &reg, &status, 1, SPI_OPERATION_WAIT);
	/* Pull up chip select */
	NRF24L01_CSN_HIGH;

	return status;
}

nrf24l01TransmitStatus_t nrf24l01GetTransmissionStatus(void)
{
	uint8_t status = nrf24l01GetStatus();
	if (NRF24L01_CHECK_BIT(status, NRF24L01_TX_DS))
	{
		/* Successfully sent */
		return nrf24l01TransmitStatusOk;
	}
	else if (NRF24L01_CHECK_BIT(status, NRF24L01_MAX_RT))
	{
		/* Message lost */
		return nrf24l01TransmitStatusLost;
	}

	/* Still sending */
	return nrf24l01TransmitStatusSending;
}

void nrf24l01SoftwareReset(void)
{
	uint8_t data[5];

	nrf24l01WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_REG_DEFAULT_VAL_CONFIG);
	nrf24l01WriteRegister(NRF24L01_REG_EN_AA, NRF24L01_REG_DEFAULT_VAL_EN_AA);
	nrf24l01WriteRegister(NRF24L01_REG_EN_RXADDR, NRF24L01_REG_DEFAULT_VAL_EN_RXADDR);
	nrf24l01WriteRegister(NRF24L01_REG_SETUP_AW, NRF24L01_REG_DEFAULT_VAL_SETUP_AW);
	nrf24l01WriteRegister(NRF24L01_REG_SETUP_RETR, NRF24L01_REG_DEFAULT_VAL_SETUP_RETR);
	nrf24l01WriteRegister(NRF24L01_REG_RF_CH, NRF24L01_REG_DEFAULT_VAL_RF_CH);
	nrf24l01WriteRegister(NRF24L01_REG_RF_SETUP, NRF24L01_REG_DEFAULT_VAL_RF_SETUP);
	nrf24l01WriteRegister(NRF24L01_REG_STATUS, NRF24L01_REG_DEFAULT_VAL_STATUS);
	nrf24l01WriteRegister(NRF24L01_REG_OBSERVE_TX, NRF24L01_REG_DEFAULT_VAL_OBSERVE_TX);
	nrf24l01WriteRegister(NRF24L01_REG_RPD, NRF24L01_REG_DEFAULT_VAL_RPD);

	//P0
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P0_4;
	nrf24l01WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, data, 5);

	//P1
	data[0] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P1_4;
	nrf24l01WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, data, 5);

	nrf24l01WriteRegister(NRF24L01_REG_RX_ADDR_P2, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P2);
	nrf24l01WriteRegister(NRF24L01_REG_RX_ADDR_P3, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P3);
	nrf24l01WriteRegister(NRF24L01_REG_RX_ADDR_P4, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P4);
	nrf24l01WriteRegister(NRF24L01_REG_RX_ADDR_P5, NRF24L01_REG_DEFAULT_VAL_RX_ADDR_P5);

	//TX
	data[0] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_0;
	data[1] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_1;
	data[2] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_2;
	data[3] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_3;
	data[4] = NRF24L01_REG_DEFAULT_VAL_TX_ADDR_4;
	nrf24l01WriteRegisterMulti(NRF24L01_REG_TX_ADDR, data, 5);

	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P0, NRF24L01_REG_DEFAULT_VAL_RX_PW_P0);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P1, NRF24L01_REG_DEFAULT_VAL_RX_PW_P1);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P2, NRF24L01_REG_DEFAULT_VAL_RX_PW_P2);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P3, NRF24L01_REG_DEFAULT_VAL_RX_PW_P3);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P4, NRF24L01_REG_DEFAULT_VAL_RX_PW_P4);
	nrf24l01WriteRegister(NRF24L01_REG_RX_PW_P5, NRF24L01_REG_DEFAULT_VAL_RX_PW_P5);
	nrf24l01WriteRegister(NRF24L01_REG_FIFO_STATUS, NRF24L01_REG_DEFAULT_VAL_FIFO_STATUS);
	nrf24l01WriteRegister(NRF24L01_REG_DYNPD, NRF24L01_REG_DEFAULT_VAL_DYNPD);
	nrf24l01WriteRegister(NRF24L01_REG_FEATURE, NRF24L01_REG_DEFAULT_VAL_FEATURE);
}

uint8_t nrf24l01GetRetransmissionsCount(void)
{
	/* Low 4 bits */
	return nrf24l01ReadRegister(NRF24L01_REG_OBSERVE_TX) & 0x0F;
}

void nrf24l01SetChannel(uint8_t channel)
{
	if (channel <= 125 && channel != Nrf20l01Config.channel)
	{
		/* Store new channel setting */
		Nrf20l01Config.channel = channel;
		/* Write channel */
		nrf24l01WriteRegister(NRF24L01_REG_RF_CH, channel);
	}
}

void nrf24l01SetRF(nrf24l01DataRate_t dataRate, nrf24l01OutputPower_t outPwr)
{
	uint8_t tmp = 0;
	Nrf20l01Config.dataRate = dataRate;
	Nrf20l01Config.outPower = outPwr;

	if (dataRate == nrf24l01DataRate2M)
	{
		tmp |= 1 << NRF24L01_RF_DR_HIGH;
	}
	else if (dataRate == nrf24l01DataRate250k)
	{
		tmp |= 1 << NRF24L01_RF_DR_LOW;
	}
	/* If 1Mbps, all bits set to 0 */

	if (outPwr == nrf24l01OutputPower0dBm)
	{
		tmp |= 3 << NRF24L01_RF_PWR;
	}
	else if (outPwr == nrf24l01OutputPowerM6dBm)
	{
		tmp |= 2 << NRF24L01_RF_PWR;
	}
	else if (outPwr == nrf24l01OutputPowerM12dBm)
	{
		tmp |= 1 << NRF24L01_RF_PWR;
	}

	nrf24l01WriteRegister(NRF24L01_REG_RF_SETUP, tmp);
}
