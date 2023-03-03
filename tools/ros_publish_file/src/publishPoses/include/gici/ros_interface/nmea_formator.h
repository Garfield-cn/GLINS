#pragma once

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

// Decode GNRMC message
int decodeRMC(char *buff, sol_t *sol);

// Decode GNGGA message
int decodeGGA(char *buff, sol_t *sol);

// ESA informations
typedef struct {
  gtime_t time;
  double vel[3];
  double att[3];
} esa_t;

// Decode GNESA message
int decodeESA(char *buff, sol_t *sol, esa_t *esa);

// Encode GNRMC message
int encodeRMC(const sol_t *sol, char *buff);

// Encode GNGGA message
int encodeGGA(const sol_t *sol, char *buff);

#ifdef __cplusplus
}
#endif
