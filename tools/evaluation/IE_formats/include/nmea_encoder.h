#pragma once

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

// Encode GNGGA message
int encodeGGA(const sol_t* sol, uint8_t* buf);

// Encode GNRMC message
int encodeRMC(const sol_t* sol, uint8_t* buf);


#ifdef __cplusplus
}
#endif
