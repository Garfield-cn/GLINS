#include "nmea_encoder.h"

#define NMEA_TID   "GP"         /* NMEA talker ID for RMC and GGA sentences */
#define MAXFIELD   64           /* max number of fields in a record */
#define MAXNMEA    256          /* max length of nmea sentence */
#define KNOT2M     0.514444444  /* m/knot */
static const int nmea_solq[]={  /* NMEA GPS quality indicator */
    /* 0=Fix not available or invalidi */
    /* 1=GPS SPS Mode, fix valid */
    /* 2=Differential GPS, SPS Mode, fix valid */
    /* 3=GPS PPS Mode, fix valid */
    /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
    /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
    /* 6=Estimated (dead reckoning) Mode */
    /* 7=Manual Input Mode */
    /* 8=Simulation Mode */
    SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
    SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
};

// Encode GNGGA message
int encodeGGA(const sol_t* sol, uint8_t* buf)
{
  gtime_t time;
  double h,ep[6],pos[3],dms1[3],dms2[3],dop=1.0;
  int solq,refid=0;
  char *p=(char *)buf,*q,sum;
  
  if (sol->stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sGGA,,,,,,,,,,,,,,",NMEA_TID);
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  for (solq=0;solq<8;solq++) if (nmea_solq[solq]==sol->stat) break;
  if (solq>=8) solq=0;
  time=gpst2utc(sol->time);
  time2epoch(time,ep);
  ecef2pos(sol->rr,pos);
  h=geoidh(pos);
  deg2dms(fabs(pos[0])*R2D,dms1,7);
  deg2dms(fabs(pos[1])*R2D,dms2,7);
  p+=sprintf(p,"$%sGGA,%02.0f%02.0f%06.3f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,"
              "%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%04d",
              NMEA_TID,ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,
              pos[0]>=0?"N":"S",dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",
              solq,sol->ns,dop,pos[2]-h,h,sol->age,refid);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}

// Encode GNRMC message
int encodeRMC(const sol_t* sol, uint8_t* buf)
{
  static double dirp=0.0;
  gtime_t time;
  double ep[6],pos[3],enuv[3],dms1[3],dms2[3],vel,dir,amag=0.0;
  char *p=(char *)buf,*q,sum;
  const char *emag="E",*mode="A",*status="V";
  
  trace(3,"outnmea_rmc:\n");
  
  if (sol->stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sRMC,,,,,,,,,,,,,",NMEA_TID);
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  time=gpst2utc(sol->time);
  time2epoch(time,ep);
  ecef2pos(sol->rr,pos);
  ecef2enu(pos,sol->rr+3,enuv);
  vel=norm(enuv,3);
  if (vel>=1.0) {
    dir=atan2(enuv[0],enuv[1])*R2D;
    if (dir<0.0) dir+=360.0;
    dirp=dir;
  }
  else {
    dir=dirp;
  }
  if      (sol->stat==SOLQ_DGPS ||sol->stat==SOLQ_SBAS) mode="D";
  else if (sol->stat==SOLQ_FLOAT||sol->stat==SOLQ_FIX ) mode="R";
  else if (sol->stat==SOLQ_PPP) mode="P";
  deg2dms(fabs(pos[0])*R2D,dms1,7);
  deg2dms(fabs(pos[1])*R2D,dms2,7);
  p+=sprintf(p,"$%sRMC,%02.0f%02.0f%06.3f,A,%02.0f%010.7f,%s,%03.0f%010.7f,"
              "%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s,%s",
              NMEA_TID,ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,
              pos[0]>=0?"N":"S",dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",
              vel/KNOT2M,dir,ep[2],ep[1],(int)ep[0]%100,amag,emag,mode,status);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}
