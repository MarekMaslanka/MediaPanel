#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <nrf24l01_hwconfig.h>

typedef enum {
	nrf24l01TransmitStatusLost = 0x00,   /*!< Message is lost, reached maximum number of retransmissions */
	nrf24l01TransmitStatusOk = 0x01,     /*!< Message sent successfully */
	nrf24l01TransmitStatusSending = 0xFF /*!< Message is still sending */
} nrf24l01TransmitStatus_t;

typedef enum {
	nrf24l01DataRate2M,  /*!< Data rate set to 2Mbps */
	nrf24l01DataRate1M,  /*!< Data rate set to 1Mbps */
	nrf24l01DataRate250k /*!< Data rate set to 250kbps */
} nrf24l01DataRate_t;

typedef enum {
	nrf24l01OutputPowerM18dBm,	/*!< Output power set to -18dBm */
	nrf24l01OutputPowerM12dBm, /*!< Output power set to -12dBm */
	nrf24l01OutputPowerM6dBm,  /*!< Output power set to -6dBm */
	nrf24l01OutputPower0dBm    /*!< Output power set to 0dBm */
} nrf24l01OutputPower_t;

#define NRF24L01_CLEAR_INTERRUPTS   do { nrf24l01WriteRegister(0x07, 0x70); } while (0)

#define NRF24L01_GET_INTERRUPTS     nrf24l01GetStatus()

/* Interrupt masks */
#define NRF24L01_IRQ_DATA_READY     0x40 /*!< Data ready for receive */
#define NRF24L01_IRQ_TRAN_OK        0x20 /*!< Transmission went OK */
#define NRF24L01_IRQ_MAX_RT         0x10 /*!< Max retransmissions reached, last transmission failed */

uint8_t nrf24l01Init(uint8_t channel, uint8_t payloadSize);
void nrf24l01SetMyAddress(uint8_t* adr);
void nrf24l01SetTxAddress(uint8_t* adr);
uint8_t nrf24l01GetRetransmissionsCount(void);
void nrf24l01PowerUpTx(void);
void nrf24l01PowerUpRx(void);
void nrf24l01PowerDown(void);
nrf24l01TransmitStatus_t nrf24l01GetTransmissionStatus(void);
void nrf24l01Transmit(uint8_t *data);
uint8_t nrf24l01DataReady(void);
void nrf24l01GetData(uint8_t *data);
void nrf24l01SetChannel(uint8_t channel);
void nrf24l01SetRF(nrf24l01DataRate_t dataRate, nrf24l01OutputPower_t outPwr);

uint8_t nrf24l01GetStatus(void);
void nrf24l01WriteRegister(uint8_t reg, uint8_t value);
uint8_t nRF24_Check(void);

void sendByte(uint8_t data, uint8_t *result);

#endif /* NRF24L01_H_ */
