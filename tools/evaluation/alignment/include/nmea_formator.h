/**
* @Function: Decode and encode GICI NMEA messages
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

#undef lock
#undef unlock

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

// Encode GNESA (self-defined Extended Speed and Attitude) message
// Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
int encodeESA(const sol_t *sol, const esa_t *esa, uint8_t* buf);

#ifdef __cplusplus
}

// One epoch NMEA message
typedef struct {
  sol_t sol;
  esa_t esa;
} NmeaEpoch;

// Load and decode NMEA file
bool loadNmeaFile(char *path, std::vector<NmeaEpoch>& epochs);

// Encode and write NMEA file
bool writeNmeaFile(const std::vector<NmeaEpoch>& epochs, char *path);

#endif
