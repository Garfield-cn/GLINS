#pragma once

#include "rtklib.h"

typedef struct IlrStat_t_tag {
	int           gpsweek;
	double        gpstow;    //GPS system Time Of Week, unit: ms
	double        att[3];    //attitude 3D, unit: rad, |rolling[0]|pitch[1]|heading[2]|
	double        pos[3];    //position of GNSS output in ECEF frame, unit: m, |X[0]|Y[1]|Z[2]|
} IlrStat_t;

// Encode GNGGA message
int encodeGGA(const sol_t* sol, uint8_t* buf);

// Encode GNRMC message
int encodeRMC(const sol_t* sol, uint8_t* buf);
