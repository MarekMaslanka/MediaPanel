#ifndef NRFCONFIG_H
#define NRFCONFIG_H

#include <stdint.h>

typedef enum
{
    NRF_SPEED_250K = 0,
    NRF_SPEED_1M,
    NRF_SPEED_2M
} nrfSpeed_t;

typedef enum
{
    NRF_POWER_M18DBM = 0,
    NRF_POWER_M12DBM,
    NRF_POWER_M6DBM,
    NRF_POWER_0DBM
} nrfOutputPower_t;

typedef enum
{
    NRF_CRC8,
    NRF_CRC16
} nrfCrc_t;

typedef struct
{
    uint8_t channel;
    uint8_t payloadSize;
    nrfSpeed_t speed:8;
    nrfOutputPower_t outPower:8;
    uint8_t autoAck;
    uint8_t retriesCount;
    uint32_t retriesDelay;
    uint8_t addressWidth;
    uint8_t address[5];
    uint8_t enableCrc;
    nrfCrc_t crc:8;
} nrfConfig_t;

#ifdef __cplusplus
class NrfConfig
{
public:
    NrfConfig();
};

#endif

#endif // NRFCONFIG_H
