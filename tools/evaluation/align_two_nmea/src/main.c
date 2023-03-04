#include "rtklib.h"
#include "nmea_encoder.h"

// Align NMEA file with high rate to the low rate file
int main(int argc, char ** argv)
{
  char nmea_high_rate[1024], nmea_low_rate[1024];
	if (argc < 3) {
		return -1;
	} else if (argc == 3) {
		strcpy(nmea_high_rate, argv[1]);
    strcpy(nmea_low_rate, argv[2]);
	}

  char buf[1034];
  sprintf(buf, "%s.aligned", nmea_high_rate);
  FILE *fp_high_aligned = fopen(buf, "w");
  
  // Load
  char *paths[1];
  solbuf_t sols_high = {0}, sols_low = {0};
  gtime_t ts = {0}, te = {0};
  paths[0] = nmea_high_rate;
  readsolt(paths, 1, ts, te, 0, 0, &sols_high);
  paths[0] = nmea_low_rate;
  readsolt(paths, 1, ts, te, 0, 0, &sols_low);

  if (sols_low.n > sols_high.n) {
    printf("The number of message contained in the first file should be larger than the sencond one!\r\n");
    return -1;
  }

  // Interpolate
  solbuf_t sols_high_aligned = {0};
  initsolbuf(&sols_high_aligned, 0, 0);
  sols_high_aligned.nmax = sols_low.n;
  if (!(sols_high_aligned.data=malloc(sizeof(sol_t)*sols_high_aligned.nmax))) {
      return -1;
  }
  int i = 0;
  for (int k = 0; k < sols_low.n; k++) {
    double time, time_next;
    time = (double)sols_high.data[i].time.time + sols_high.data[i].time.sec;
    if (i + 1 >= sols_high.n) {
      continue;
    }
    else {
      time_next = (double)sols_high.data[i + 1].time.time + sols_high.data[i + 1].time.sec;
    }
    double require_time = (double)sols_low.data[k].time.time + sols_low.data[k].time.sec;
    if (time > require_time) {
      continue;
    }
    else if (time <= require_time && time_next > require_time) {
      sol_t *sol = &sols_high_aligned.data[sols_high_aligned.n];
      *sol = sols_high.data[i];
      sol->time.time = (time_t)floor(require_time);
      sol->time.sec = require_time - floor(require_time);
      double interval = time_next - time;
      double dt = require_time - time;
      const double r = dt / interval;
      for (int m = 0; m < 6; m++) {
        sol->rr[m] = (1.0 - r) * sols_high.data[i].rr[m] + r * sols_high.data[i + 1].rr[m];
      }
      sols_high_aligned.n++;
    }
    else {
      i++; k--;
    }
  }

  // Output
  for (int i = 0; i < sols_high_aligned.n; i++) {
    uint8_t *p = (uint8_t *)buf;
    p += encodeGGA(sols_high_aligned.data + i, p);
    p += encodeRMC(sols_high_aligned.data + i, p);
    fwrite(buf, strlen(buf), 1, fp_high_aligned);
  }

  fclose(fp_high_aligned);

  return 0;
}