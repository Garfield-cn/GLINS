/**
* @Function: Single Point positioning implementation
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
*
* Copyright (C) 2023 by Cheng Chi, All rights reserved.
**/
#include "gici/gnss/spp_estimator.h"

#include "gici/gnss/gnss_parameter_blocks.h"
#include "gici/gnss/gnss_common.h"

// ESA informations
typedef struct {
  gtime_t time;
  double vel[3];
  double att[3];
} esa_t;

// ESD informations
typedef struct {
  gtime_t time;
  double std_pos[3];
  double std_vel[3];
  double std_att[3];
} esd_t;

// One epoch NMEA message
typedef struct {
  sol_t sol;
  esa_t esa;
  esd_t esd;
} NmeaEpoch;

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

/* convert ddmm.mm in nmea format to deg -------------------------------------*/
static double dmm2deg(double dmm)
{
    return floor(dmm/100.0)+fmod(dmm,100.0)/60.0;
}
/* convert time in nmea format to time ---------------------------------------*/
static void septime(double t, double *t1, double *t2, double *t3)
{
    *t1=floor(t/10000.0);
    t-=*t1*10000.0;
    *t2=floor(t/100.0);
    *t3=t-*t2*100.0;
}

// Decode GNRMC message
int decodeRMC(char *buff, sol_t *sol)
{
    double tod=0.0,lat=0.0,lon=0.0,vel=0.0,dir=0.0,date=0.0,ang=0.0,ep[6];
    double pos[3]={0};
    char act=' ',ns='N',ew='E',mew='E',mode='A';
    int i;
    char *p,*q,*val[MAXFIELD],**v;
    int n=0;
    
    trace(4,"decode_nmearmc: buff=%s\n",buff);

    /* parse fields */
    for (p=buff;*p&&n<MAXFIELD;p=q+1) {
        if ((q=strchr(p,','))||(q=strchr(p,'*'))) {
            val[n++]=p; *q='\0';
        }
        else break;
    }
    if (n<1) {
        return 0;
    }
    
    v = val + 1;
    for (i=0;i<n;i++) {
        switch (i) {
            case  0: tod =atof(v[i]); break; /* time in utc (hhmmss) */
            case  1: act =*v[i];      break; /* A=active,V=void */
            case  2: lat =atof(v[i]); break; /* latitude (ddmm.mmm) */
            case  3: ns  =*v[i];      break; /* N=north,S=south */
            case  4: lon =atof(v[i]); break; /* longitude (dddmm.mmm) */
            case  5: ew  =*v[i];      break; /* E=east,W=west */
            case  6: vel =atof(v[i]); break; /* speed (knots) */
            case  7: dir =atof(v[i]); break; /* track angle (deg) */
            case  8: date=atof(v[i]); break; /* date (ddmmyy) */
            case  9: ang =atof(v[i]); break; /* magnetic variation */
            case 10: mew =*v[i];      break; /* E=east,W=west */
            case 11: mode=*v[i];      break; /* mode indicator (>nmea 2) */
                                      /* A=autonomous,D=differential */
                                      /* E=estimated,N=not valid,S=simulator */
        }
    }
    if ((act!='A'&&act!='V')||(ns!='N'&&ns!='S')||(ew!='E'&&ew!='W')) {
        trace(3,"invalid nmea rmc format\n");
        return 0;
    }
    pos[0]=(ns=='S'?-1.0:1.0)*dmm2deg(lat)*D2R;
    pos[1]=(ew=='W'?-1.0:1.0)*dmm2deg(lon)*D2R;
    septime(date,ep+2,ep+1,ep);
    septime(tod,ep+3,ep+4,ep+5);
    ep[0]+=ep[0]<80.0?2000.0:1900.0;
    sol->time=utc2gpst(epoch2time(ep));
    // pos2ecef(pos,sol->rr);
    // sol->stat=mode=='D'?SOLQ_DGPS:SOLQ_SINGLE;
    // sol->ns=0;
    
    // sol->type=0; /* postion type = xyz */
    
    trace(5,"decode_nmearmc: %s rr=%.3f %.3f %.3f stat=%d ns=%d vel=%.2f dir=%.0f ang=%.0f mew=%c mode=%c\n",
          time_str(sol->time,0),sol->rr[0],sol->rr[1],sol->rr[2],sol->stat,sol->ns,
          vel,dir,ang,mew,mode);
    
    return 2; /* update time */
}

// Decode GNGGA message
int decodeGGA(char *buff, sol_t *sol)
{
    gtime_t time;
    double tod=0.0,lat=0.0,lon=0.0,hdop=0.0,alt=0.0,msl=0.0,ep[6],tt;
    double pos[3]={0},age=0.0;
    char ns='N',ew='E',ua=' ',um=' ';
    int i,solq=0,nrcv=0;
    char *p,*q,*val[MAXFIELD],**v;
    int n=0;
    
    trace(4,"decode_nmea: buff=%s\n",buff);
    
    /* parse fields */
    for (p=buff;*p&&n<MAXFIELD;p=q+1) {
        if ((q=strchr(p,','))||(q=strchr(p,'*'))) {
            val[n++]=p; *q='\0';
        }
        else break;
    }
    if (n<1) {
        return 0;
    }
    
    v = val + 1;
    for (i=0;i<n;i++) {
        switch (i) {
            case  0: tod =atof(v[i]); break; /* UTC of position (hhmmss) */
            case  1: lat =atof(v[i]); break; /* Latitude (ddmm.mmm) */
            case  2: ns  =*v[i];      break; /* N=north,S=south */
            case  3: lon =atof(v[i]); break; /* Longitude (dddmm.mmm) */
            case  4: ew  =*v[i];      break; /* E=east,W=west */
            case  5: solq=atoi(v[i]); break; /* GPS quality indicator */
            case  6: nrcv=atoi(v[i]); break; /* # of satellites in use */
            case  7: hdop=atof(v[i]); break; /* HDOP */
            case  8: alt =atof(v[i]); break; /* Altitude MSL */
            case  9: ua  =*v[i];      break; /* unit (M) */
            case 10: msl =atof(v[i]); break; /* Geoid separation */
            case 11: um  =*v[i];      break; /* unit (M) */
            case 12: age =atof(v[i]); break; /* Age of differential */
        }
    }
    if ((ns!='N'&&ns!='S')||(ew!='E'&&ew!='W')) {
        trace(3,"invalid nmea gga format\n");
        return 0;
    }
    if (sol->time.time==0) {
        trace(3,"no date info for nmea gga\n");
        return 0;
    }
    pos[0]=(ns=='N'?1.0:-1.0)*dmm2deg(lat)*D2R;
    pos[1]=(ew=='E'?1.0:-1.0)*dmm2deg(lon)*D2R;
    pos[2]=alt+msl;
    
    time2epoch(sol->time,ep);
    septime(tod,ep+3,ep+4,ep+5);
    time=utc2gpst(epoch2time(ep));
    tt=timediff(time,sol->time);
    if      (tt<-43200.0) sol->time=timeadd(time, 86400.0);
    else if (tt> 43200.0) sol->time=timeadd(time,-86400.0);
    else sol->time=time;
    pos2ecef(pos,sol->rr);
    sol->stat=0<=solq&&solq<=8?nmea_solq[solq]:SOLQ_NONE;
    sol->ns=nrcv;
    sol->age=(float)age;
    
    sol->type=0; /* postion type = xyz */
    
    trace(5,"decode_nmeagga: %s rr=%.3f %.3f %.3f stat=%d ns=%d hdop=%.1f ua=%c um=%c\n",
          time_str(sol->time,0),sol->rr[0],sol->rr[1],sol->rr[2],sol->stat,sol->ns,
          hdop,ua,um);
    
    return 1;
}

// Decode GNESA message
int decodeESA(char *buff, sol_t *sol, esa_t *esa)
{
    gtime_t time;
    double tod=0.0,ep[6],tt;
    char *p,*q,*val[MAXFIELD],**v;
    int n=0, i=0;
    
    trace(4,"decode_nmea: buff=%s\n",buff);
    
    /* parse fields */
    for (p=buff;*p&&n<MAXFIELD;p=q+1) {
        if ((q=strchr(p,','))||(q=strchr(p,'*'))) {
            val[n++]=p; *q='\0';
        }
        else break;
    }
    if (n<1) {
        return 0;
    }
    
    v = val + 1;
    for (i=0;i<n;i++) {
        switch (i) {
            case  0: tod =atof(v[i]); break; /* UTC of position (hhmmss) */
            case  1: esa->vel[0] = atof(v[i]); break;
            case  2: esa->vel[1] = atof(v[i]); break;
            case  3: esa->vel[2] = atof(v[i]); break;
            case  4: esa->att[0] = atof(v[i]) * D2R; break;
            case  5: esa->att[1] = atof(v[i]) * D2R; break;
            case  6: esa->att[2] = atof(v[i]) * D2R; break;
        }
    }
    if (sol->time.time==0) {
        trace(3,"no date info for nmea gga\n");
        return 0;
    }

    time2epoch(sol->time,ep);
    septime(tod,ep+3,ep+4,ep+5);
    time=utc2gpst(epoch2time(ep));
    tt=timediff(time,sol->time);
    if      (tt<-43200.0) esa->time=timeadd(time, 86400.0);
    else if (tt> 43200.0) esa->time=timeadd(time,-86400.0);
    else esa->time=time;

    return 1;
}

// Decode GNESD message
int decodeESD(char *buff, sol_t *sol, esd_t *esd)
{
    gtime_t time;
    double tod=0.0,ep[6],tt;
    char *p,*q,*val[MAXFIELD],**v;
    int n=0, i=0;
    
    trace(4,"decode_nmea: buff=%s\n",buff);
    
    /* parse fields */
    for (p=buff;*p&&n<MAXFIELD;p=q+1) {
        if ((q=strchr(p,','))||(q=strchr(p,'*'))) {
            val[n++]=p; *q='\0';
        }
        else break;
    }
    if (n<1) {
        return 0;
    }
    
    v = val + 1;
    for (i=0;i<n;i++) {
        switch (i) {
            case  0: tod =atof(v[i]); break; /* UTC of position (hhmmss) */
            case  1: esd->std_pos[0] = atof(v[i]); break;
            case  2: esd->std_pos[1] = atof(v[i]); break;
            case  3: esd->std_pos[2] = atof(v[i]); break;
            case  4: esd->std_vel[0] = atof(v[i]); break;
            case  5: esd->std_vel[1] = atof(v[i]); break;
            case  6: esd->std_vel[2] = atof(v[i]); break;
            case  7: esd->std_att[0] = atof(v[i]) * D2R; break;
            case  8: esd->std_att[1] = atof(v[i]) * D2R; break;
            case  9: esd->std_att[2] = atof(v[i]) * D2R; break;
        }
    }
    if (sol->time.time==0) {
        trace(3,"no date info for nmea gga\n");
        return 0;
    }

    time2epoch(sol->time,ep);
    septime(tod,ep+3,ep+4,ep+5);
    time=utc2gpst(epoch2time(ep));
    tt=timediff(time,sol->time);
    if      (tt<-43200.0) esd->time=timeadd(time, 86400.0);
    else if (tt> 43200.0) esd->time=timeadd(time,-86400.0);
    else esd->time=time;

    return 1;
}

// Encode GNRMC message
int encodeRMC(const sol_t *sol, char *buff)
{
    static double dirp=0.0;
    gtime_t time;
    double ep[6],pos[3],enuv[3],dms1[3],dms2[3],vel,dir,amag=0.0;
    char *p=(char *)buff,*q,sum;
    const char *emag="E",*mode="A",*status="V";

    trace(3,"outnmea_rmc:\n");

    if (sol->stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sRMC,,,,,,,,,,,,,",NMEA_TID);
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buff;
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
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
    p+=sprintf(p,"*%02X\r\n",sum);
    return p-(char *)buff;
}

// Encode GNGGA message
int encodeGGA(const sol_t *sol, char *buff)
{
    gtime_t time;
    double h,ep[6],pos[3],dms1[3],dms2[3],dop=1.0;
    int solq,refid=0;
    char *p=(char *)buff,*q,sum;

    if (sol->stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sGGA,,,,,,,,,,,,,,",NMEA_TID);
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buff;
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
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
    p+=sprintf(p,"*%02X\r\n",sum);
    return p-(char *)buff;
}

// Encode GNESA (self-defined Extended Speed and Attitude) message
// Format: $GNESA,tod,Ve,Vn,Vu,Ar,Ap,Ay*checksum
int encodeESA(const sol_t *sol, const esa_t *esa, char* buf)
{
  gtime_t time;
  double ep[6];
  int solq,refid=0;
  char *p=(char *)buf,*q,sum;
  
  if (sol->stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sESA,,,,,,,",NMEA_TID);
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  time=gpst2utc(sol->time);
  time2epoch(time,ep);
  p+=sprintf(p,"$%sESA,%02.0f%02.0f%06.3f,%+.3f,%+.3f,%+.3f,"
             "%+.3f,%+.3f,%+.3f",
             NMEA_TID,ep[3],ep[4],ep[5],sol->rr[3],sol->rr[4],sol->rr[5],
             esa->att[0]*R2D,esa->att[1]*R2D,esa->att[2]*R2D);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}

// Encode GNESD (self-defined Extended Speed and Attitude) message
// Format: $GNESD,tod,STD_Pe,STD_Pn,STD_Pu,STD_Ve,STD_Vn,STD_Vu,
//         STD_Ar,STD_Ap,STD_Py*checksum
int encodeESD(const sol_t *sol, const esd_t *esd, char* buf)
{
  gtime_t time;
  double ep[6], std_p[3], std_v[3], std_a[3];
  int solq,refid=0;
  char *p=(char *)buf,*q,sum;

  for (size_t i = 0; i < 3; i++) {
    std_p[i] = esd->std_pos[i];
    std_v[i] = esd->std_vel[i];
    std_a[i] = esd->std_att[i] * R2D;
  }

  if (sol->stat<=SOLQ_NONE) {
    p+=sprintf(p,"$%sESD,,,,,,,",NMEA_TID);
    for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q;
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buf;
  }
  time=gpst2utc(sol->time);
  time2epoch(time,ep);
  p+=sprintf(p,"$%sESD,%02.0f%02.0f%06.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             NMEA_TID,ep[3],ep[4],ep[5],std_p[0],std_p[1],std_p[2],
             std_v[0],std_v[1],std_v[2],std_a[0],std_a[1],std_a[2]);
  for (q=(char *)buf+1,sum=0;*q;q++) sum^=*q; /* check-sum */
  p+=sprintf(p,"*%02X\r\n",sum);
  return p-(char *)buf;
}

// Load and decode NMEA file
bool loadNmeaFile(char *path, std::vector<NmeaEpoch>& epochs)
{
    FILE *fp_nmea = fopen(path, "r");
    if (fp_nmea == NULL) return false;

    char buf[1024];
    sol_t sol = {0};
    esa_t esa = {0};
    esd_t esd = {0};
    std::vector<sol_t> sols;
    std::vector<esa_t> esas;
    std::vector<esd_t> esds;
    while (!(fgets(buf, 1024 * sizeof(char), fp_nmea) == NULL))
    {
        std::vector<std::string> strs;
        strs.push_back("");
        for (int i = 0; i < strlen(buf); i++) {
            if (buf[i] == ',') {
            if (strs.back().size() > 0) strs.push_back("");
            continue;
            }
            if (buf[i] == '\r' || buf[i] == '\n') break;
            strs.back().push_back(buf[i]);
        }
        if (strs.size() < 2) continue;
        if (strs[0].size() < 6) continue;

        if (strs[0].substr(3, 3) == "GGA") {
            if (decodeGGA(buf, &sol)) sols.push_back(sol);
        }
        else if (strs[0].substr(3, 3) == "RMC") {
            decodeRMC(buf, &sol);
        }
        else if (strs[0].substr(3, 3) == "ESA") {
            if (decodeESA(buf, &sol, &esa)) esas.push_back(esa);
        }
        else if (strs[0].substr(3, 3) == "ESD") {
            if (decodeESD(buf, &sol, &esd)) esds.push_back(esd);
        }
    }
    fclose(fp_nmea);

    int esa_index = 0, esd_index = 0;
    for (int i = 0; i < sols.size(); i++) {
        epochs.push_back(NmeaEpoch());
        epochs.back().sol = sols[i];
        for (int j = esa_index; j < esas.size(); j++) {
            double dt = timediff(esas[j].time, sols[i].time);
            if (dt == 0.0) {
                epochs.back().esa = esas[j];
                esa_index = j;
                break;
            }
            if (dt > 0.0) break;
        }
        for (int j = esd_index; j < esds.size(); j++) {
            double dt = timediff(esds[j].time, sols[i].time);
            if (dt == 0.0) {
                epochs.back().esd = esds[j];
                esd_index = j;
                break;
            }
            if (dt > 0.0) break;
        }
    }

    return true;
}

namespace gici {

static std::vector<NmeaEpoch> epochs_;
static int epochs_idx_ = 0;

// The default constructor
SppEstimator::SppEstimator(const SppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options, 
               const EstimatorBaseOptions& base_options) :
  spp_options_(options), 
  GnssEstimatorBase(gnss_base_options, base_options),
  EstimatorBase(base_options)
{
  type_ = EstimatorType::Spp;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());
  can_compute_covariance_ = true;

  loadNmeaFile("/media/cc/T7/ubuntu/datasets/urbannav/mediumurban/gnss/20210517.light-urban.tste.ublox.f9p.splitter.nmea", epochs_);
}

SppEstimator::SppEstimator(const SppEstimatorOptions& options, 
               const GnssEstimatorBaseOptions& gnss_base_options) :
  spp_options_(options), 
  GnssEstimatorBase(gnss_base_options, EstimatorBaseOptions()),
  EstimatorBase(EstimatorBaseOptions())
{
  base_options_.max_iteration = 10;
  base_options_.max_solver_time = 0.05;
  base_options_.solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  base_options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  base_options_.verbose_output = false;

  type_ = EstimatorType::Spp;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());

  loadNmeaFile("/media/cc/T7/ubuntu/datasets/urbannav/mediumurban/gnss/20210517.light-urban.tste.ublox.f9p.splitter.nmea", epochs_);
}

SppEstimator::SppEstimator(const GnssEstimatorBaseOptions& gnss_base_options) :
  spp_options_(SppEstimatorOptions()), 
  GnssEstimatorBase(gnss_base_options, EstimatorBaseOptions()),
  EstimatorBase(EstimatorBaseOptions())
{
  base_options_.max_iteration = 10;
  base_options_.max_solver_time = 0.05;
  base_options_.solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  base_options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  base_options_.verbose_output = false;

  type_ = EstimatorType::Spp;
  states_.push_back(State());
  gnss_measurements_.push_back(GnssMeasurement());

  loadNmeaFile("/media/cc/T7/ubuntu/datasets/urbannav/mediumurban/gnss/20210517.light-urban.tste.ublox.f9p.splitter.nmea", epochs_);
}

// The default destructor
SppEstimator::~SppEstimator()
{}

// Add measurement
bool SppEstimator::addMeasurement(const EstimatorDataCluster& measurement)
{
  if (measurement.gnss && measurement.gnss_role == GnssRole::Rover) {
    return addGnssMeasurementAndState(*measurement.gnss);
  }

  return false;
}

// Add GNSS measurements and state
bool SppEstimator::addGnssMeasurementAndState(
    const GnssMeasurement& measurement)
{
  // Get prior position
  Eigen::Vector3d position_prior = Eigen::Vector3d::Zero();
  if (!isFirstEpoch()) {
    position_prior = getPositionEstimate();
  }
  else if (coordinate_) {
    position_prior = coordinate_->getZero(GeoType::ECEF);
  }
  else {
    // compute from ephemerides
    int num_sum_satellites = 0;
    Eigen::Vector3d sum_satellite_position = Eigen::Vector3d::Zero();
    for (auto& sat : measurement.satellites) {
      auto& satellite = sat.second;
      if (checkZero(satellite.sat_position)) continue;
      sum_satellite_position += satellite.sat_position;
      num_sum_satellites++;
    }
    Eigen::Vector3d avg_satellite_position = sum_satellite_position / 
      static_cast<double>(num_sum_satellites);
    Eigen::Vector3d avg_lla;
    ecef2pos(avg_satellite_position.data(), avg_lla.data());
    avg_lla(2) = 0.0;
    pos2ecef(avg_lla.data(), position_prior.data());
  }

  // Set to local measurement handle
  curGnss() = measurement;
  curGnss().position = position_prior;

  // Erase non-base-frequency measurements and non-dual-frequency satellites
  if (spp_options_.use_dual_frequency) arrangeDualFrequency(curGnss());
  // Correct code bias for single frequency mode
  else correctCodeBias(curGnss());

  // Compute ionosphere delays
  computeIonosphereDelay(curGnss(), !spp_options_.use_dual_frequency);

  // Add parameter blocks
  double timestamp = measurement.timestamp;
  curState().timestamp = timestamp;


  bool found = false;
  // for (; epochs_idx_ < epochs_.size(); epochs_idx_++) {
  //   double timestamp_epoch = gnss_common::gtimeToDouble(gpst2utc(epochs_[epochs_idx_].sol.time));
  //   if (checkEqual(timestamp_epoch, timestamp, 1.0e-2)) {
  //     position_prior = Eigen::Map<Eigen::Vector3d>(epochs_[epochs_idx_].sol.rr);
  //     found = true;
  //     break;
  //   }
  //   while (timestamp_epoch > timestamp) {
  //     epochs_idx_--; 
  //     timestamp_epoch = gnss_common::gtimeToDouble(gpst2utc(epochs_[epochs_idx_].sol.time));
  //     LOG(INFO) << "epochs_idx_: " << epochs_idx_ << ", " << "time: " << std::fixed << timestamp_epoch;
  //   }
  // }
  // position block
  BackendId position_id = addGnssPositionParameterBlock(curGnss().id, position_prior);
  curState().id = position_id;
  curState().id_in_graph = position_id;
  if (found) {
    LOG(INFO) << "Found truth at " << std::fixed << timestamp << ": " << std::fixed << position_prior.transpose();
    graph_->setParameterBlockConstant(position_id.asInteger());
  }


  // clock block
  int num_valid_system = 0;
  addClockParameterBlocks(curGnss(), curGnss().id, num_valid_system);
  
  // Add pseudorange residual blocks
  int num_valid_satellite = 0;
  addPseudorangeResidualBlocks(
    curGnss(), curState(), num_valid_satellite, true);

  // Check if insufficient satellites
  if (!checkSufficientSatellite(num_valid_satellite, num_valid_system)) {
    return false;
  }

  num_satellites_ = num_valid_satellite;

  // Erase all parameters in previous states
  Graph::ParameterBlockCollection parameters = graph_->parameters();
  for (auto parameter : parameters) {
    if (BackendId(parameter.first).bundleId() == curState().id.bundleId()) continue;
    graph_->removeParameterBlock(parameter.first);
  }

  return true;
}

// Solve current graph
bool SppEstimator::estimate()
{
  status_ = EstimatorStatus::Converged;

  // Optimize with FDE
  if (gnss_base_options_.use_outlier_rejection)
  while (1)
  {
    optimize();
    // reject outlier
    if (!rejectPseudorangeOutlier(curState(), 
        gnss_base_options_.reject_one_outlier_once)) break;
  }
  // Optimize without FDE
  else {
    optimize();
  }

  // for (size_t i = 0; i < getGnssSystemList().size(); i++) 
  // {
  //   const char system = getGnssSystemList()[i];
  //   BackendId clock_id = createGnssClockId(system, curState().id_in_graph.bundleId());
  //   if (gnss_common::useSystem(gnss_base_options_.common, system) && 
  //       graph_->parameterBlockExists(clock_id.asInteger())) 
  //   {
  //     auto clock_parameter = graph_->parameterBlockPtr(clock_id.asInteger());
  //     LOG(INFO) << "Clock " << system << ": " << clock_parameter->parameters()[0];
  //   }
  // }

  // {
  //   std::ofstream outfile;
  //   outfile.open("/home/cc/Work/Data/tmp/log.txt", std::ios::out | std::ios::trunc);
  //   outfile << graph_->summary.BriefReport() << std::endl;

  //   for (auto residual_map : graph_->residual_block_id_to_parameter_block_collection_map_) {
  //     auto residual = residual_map.first;
  //     auto error_interface_ptr = graph_->residual_block_id_to_residual_block_spec_map_.at(residual_map.first).error_interface_ptr;
  //     size_t size = error_interface_ptr->residualDim();
  //     size_t para_size = residual_map.second.size();

  //     Eigen::VectorXd Residuals = Eigen::VectorXd::Zero(size);
  //     graph_->problem_->EvaluateResidualBlock(
  //           residual, false, nullptr, Residuals.data(), nullptr);
  //     error_interface_ptr->deNormalizeResidual(Residuals.data());
  //     double norm_residual = Residuals.norm();
  //     if (norm_residual < 10.0) {
  //           outfile << "Residual " << kErrorToStr.at(error_interface_ptr->typeInfo()) << ": " 
  //                 << residual << ": " << Residuals.transpose() << std::endl;
  //     }
  //     else {
  //           outfile << "*Residual " << kErrorToStr.at(error_interface_ptr->typeInfo()) << ": " 
  //                 << residual << ": " << Residuals.transpose() << std::endl;
  //     }
  //     if (error_interface_ptr->typeInfo() == ErrorType::kReprojectionError) {
  //           auto parameters = graph_->parameters(residual);
  //           outfile << BackendId(parameters[0].first).bundleId() << std::endl;
  //     }
  //   }
  //   outfile.close();
  // }

  // Check DOP
  bool dop_valid = true;
  curGnss().position = getPositionEstimate(curState());
  if (!checkZero(curGnss().position)) {
    updateGdop(curGnss());
    if (gdop_ > gnss_base_options_.common.max_gdop) {
      LOG(INFO) << "High GDOP! Our tolerant is " 
        << gnss_base_options_.common.max_gdop << " in maximum, but current GDOP is "
        << gdop_ << "!";
      // erase parameters
      Graph::ParameterBlockCollection parameters = graph_->parameters();
      for (auto parameter : parameters) graph_->removeParameterBlock(parameter.first);
      states_.clear(); 
      dop_valid = false;
    }
  }

  // Check chi-square
  int n_residual = 0;
  int n_parameter = graph_->parameters().size();
  double chi_square = 0.0;
  auto residual_blocks = graph_->residuals();
  for (auto residual_block : residual_blocks) {
    std::shared_ptr<ErrorInterface> interface = residual_block.error_interface_ptr;
    ErrorType type = interface->typeInfo();
    if (!(type == ErrorType::kPseudorangeError)) continue;
    double residual[1];
    n_residual++;
    graph_->problem()->EvaluateResidualBlock(residual_block.residual_block_id, 
      false, nullptr, residual, nullptr);
    chi_square += square(residual[0]);
  }
  int chisqr_degree = n_residual - n_parameter - 1;
  if (chisqr_degree > 0 && chi_square > chisqr[n_residual - n_parameter - 1]) {
    LOG(INFO) << "High chi-square! Chi-square is " << chi_square
      << ". Chi-square threshold is " << chisqr[n_residual - n_parameter - 1] << ".";
  }

  // Log information
  if (base_options_.verbose_output) {
    LOG(INFO) << estimatorTypeToString(type_) << ": " 
      << "Iterations: " << graph_->summary.iterations.size() << ", "
      << std::scientific << std::setprecision(3) 
      << "Initial cost: " << graph_->summary.initial_cost << ", "
      << "Final cost: " << graph_->summary.final_cost
      << ", Sat number: " << std::setw(2) << num_satellites_      
      << ", GDOP: " << std::setprecision(1) << std::fixed << gdop_;
  }

  // Estimate velocity
  if (spp_options_.estimate_velocity && dop_valid) 
  {
    // Add parameter blocks
    int num_valid_doppler_system = 0;
    // velocity block
    addGnssVelocityParameterBlock(curGnss().id);
    // frequency block
    addFrequencyParameterBlocks(curGnss(), curGnss().id, num_valid_doppler_system);

    // Add doppler residual blocks
    int num_valid_doppler_satellite = 0;
    addDopplerResidualBlocks(curGnss(), curState(), num_valid_doppler_satellite, true);
    if (!checkSufficientSatellite(
        num_valid_doppler_satellite, num_valid_doppler_system, false)) {
      eraseGnssVelocityParameterBlock(curState());
      eraseFrequencyParameterBlocks(curState());
      has_velocity_estimate_ = false;
    }
    else {
      // To speed-up the optimization, we do the following things:
      // erase all pseudorange residuals
      erasePseudorangeResidualBlocks(curState());
      // set position and clock parameters as constant
      graph_->setParameterBlockConstant(curState().id.asInteger());
      for (auto system : getGnssSystemList()) {
        BackendId clock_id = changeIdType(curState().id, IdType::gClock, system);
        if (graph_->parameterBlockExists(clock_id.asInteger())) {
          graph_->setParameterBlockConstant(clock_id.asInteger());
        }
      }

      optimize();
      has_velocity_estimate_ = true;
    }
  }

  // Shift memory
  states_.push_back(State());
  while (states_.size() > 2) states_.pop_front();

  if (!dop_valid) status_ = EstimatorStatus::InvalidEpoch;
  return dop_valid;
}

// Erase non-base-frequency measurements and non-dual-frequency satellites
void SppEstimator::arrangeDualFrequency(GnssMeasurement& measurement)
{
  CodeBias::BaseFrequencies bases = measurement.code_bias->getBase();
  for (auto it_sat = measurement.satellites.begin(); 
       it_sat != measurement.satellites.end(); ) {
    char system = it_sat->first[0];
    std::pair<int, int> base_pair = bases.at(system);
    int phase_id_base_lhs = 
      gnss_common::getPhaseID(system, base_pair.first);
    int phase_id_base_rhs = 
      gnss_common::getPhaseID(system, base_pair.second);
    int valid_cnt = 0;
    for (auto it_obs = it_sat->second.observations.begin(); 
         it_obs != it_sat->second.observations.end(); ) {
      int phase_id = gnss_common::getPhaseID(system, it_obs->first);
      if ((phase_id != phase_id_base_lhs && phase_id != phase_id_base_rhs) || 
          !checkObservationValid(measurement, 
          GnssMeasurementIndex(it_sat->first, it_obs->first))) {
        it_obs = it_sat->second.observations.erase(it_obs);
      }
      else { valid_cnt++; it_obs++; }
    }
    if (valid_cnt != 2) {
      it_sat = measurement.satellites.erase(it_sat);
    }
    else it_sat++;
  }
}

// Get position estimate in ECEF
Eigen::Vector3d SppEstimator::getPositionEstimate(const State& state)
{
  std::shared_ptr<PositionParameterBlock> block_ptr =
      std::static_pointer_cast<PositionParameterBlock>(
        graph_->parameterBlockPtr(state.id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Get clock estimate
std::map<char, double> SppEstimator::getClockEstimate(const State& state)
{
  std::map<char, double> system_to_clock;
  for (auto system : getGnssSystemList()) {
    BackendId clock_id = changeIdType(state.id, IdType::gClock, system);
    if (!graph_->parameterBlockExists(clock_id.asInteger())) continue;
    std::shared_ptr<ClockParameterBlock> block_ptr =
        std::static_pointer_cast<ClockParameterBlock>(
          graph_->parameterBlockPtr(clock_id.asInteger()));
    CHECK(block_ptr != nullptr);
    system_to_clock.insert(std::make_pair(system, block_ptr->estimate()[0]));
  }
  return system_to_clock;
}

// Get velocity estimate in ECEF
Eigen::Vector3d SppEstimator::getVelocityEstimate(const State& state)
{
  if (!spp_options_.estimate_velocity) {
    return Eigen::Vector3d::Zero();
  }

  BackendId velocity_id = changeIdType(state.id, IdType::gVelocity);
  if (!graph_->parameterBlockExists(velocity_id.asInteger())) {
    return Eigen::Vector3d::Zero();
  }
  std::shared_ptr<VelocityParameterBlock> block_ptr =
      std::static_pointer_cast<VelocityParameterBlock>(
        graph_->parameterBlockPtr(velocity_id.asInteger()));
  CHECK(block_ptr != nullptr);
  return block_ptr->estimate();
}

// Get frequency estimate
std::map<char, double> SppEstimator::getFrequencyEstimate(const State& state)
{
  std::map<char, double> system_to_freq;
  if (!spp_options_.estimate_velocity) {
    return system_to_freq;
  }

  for (auto system : getGnssSystemList()) {
    BackendId freq_id = changeIdType(state.id, IdType::gFrequency, system);
    if (!graph_->parameterBlockExists(freq_id.asInteger())) continue;
    std::shared_ptr<FrequencyParameterBlock> block_ptr =
        std::static_pointer_cast<FrequencyParameterBlock>(
          graph_->parameterBlockPtr(freq_id.asInteger()));
    CHECK(block_ptr != nullptr);
    system_to_freq.insert(std::make_pair(system, block_ptr->estimate()[0]));
  }
  return system_to_freq;
}

// Get velocity covariance in ECEF
Eigen::Matrix3d SppEstimator::getVelocityCovariance(const State& state)
{
  if (!spp_options_.estimate_velocity) {
    return Eigen::Matrix3d::Zero();
  }

  BackendId velocity_id = changeIdType(state.id, IdType::gVelocity);
  if (!graph_->parameterBlockExists(velocity_id.asInteger())) {
    return Eigen::Matrix3d::Zero();
  }
  std::vector<size_t> parameter_block_ids;
  parameter_block_ids.push_back(velocity_id.asInteger());

  // compute covariance
  Eigen::MatrixXd covariance;
  CHECK(graph_->computeCovariance(parameter_block_ids, covariance));
  CHECK_EQ(covariance.cols(), 3);
  return covariance;
}

};