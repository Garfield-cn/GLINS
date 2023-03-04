#pragma once

#include "nmea_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

// Transform position from IMU to antenna
void transformToAntenna(IlrStat_t* ilrstat);

#ifdef __cplusplus
}
#endif
