/**
* @Function: Convert file from GICI IMU pack to IE IMR
*
* @Author  : Cheng Chi
* @Email   : chichengcn@sjtu.edu.cn
**/
#include "gici/stream/format_imu.h"

#include <iostream>
#include <string>
#include <vector>

#define FREQ 400

int main(int argc, char ** argv)
{
  char imu_pack_path[1024];
	if (argc < 2) {
		return -1;
	} else if (argc == 2) {
		strcpy(imu_pack_path, argv[1]);
	}

  FILE *fp_imu_pack = fopen(imu_pack_path, "r");
  char buf[1034];
  sprintf(buf, "%s.imr", imu_pack_path);
  FILE *fp_imr = fopen(buf, "w");

  // IMR header 
  // https://docs.novatel.com/Waypoint/Content/Data_Formats/IMR_File.htm
  const double acc_encode_factor = 1.0e-6;
  const double gyro_encode_factor = 1.0e-5;
  uint8_t head_buf[512];
  double double_tmp;
  int32_t int32_tmp;
  int idx = 0;
  sprintf((char *)head_buf, "$IMURAW"); idx += 8;
  setbits(head_buf, idx*8, 1*8, 0); idx += 1;
  double_tmp = 8.8;
  memcpy(head_buf + idx, &double_tmp, 8); idx += 8;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  double_tmp = FREQ;
  memcpy(head_buf + idx, &double_tmp, 8); idx += 8;
  memcpy(head_buf + idx, &gyro_encode_factor, 8); idx += 8;
  memcpy(head_buf + idx, &acc_encode_factor, 8); idx += 8;
  int32_tmp = 1;
  memcpy(head_buf + idx, &int32_tmp, 4); idx += 4;  // use UTC time
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;  // Unknown, will default to corrected time
  double_tmp = 0.0;
  memcpy(head_buf + idx, &double_tmp, 8); idx += 8;
  sprintf((char *)(head_buf + idx), "gici-imu"); idx += 32;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  sprintf((char *)(head_buf + idx), "gici"); idx += 32;
  memset(head_buf + idx, 0, 12); idx += 12;
  setbits(head_buf, idx*8, 1*8, 0); idx += 1;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4;
  setbits(head_buf, idx*8, 4*8, 0); idx += 4; 
  memset(head_buf + idx, 0, 354); idx += 354;
  fwrite(head_buf, sizeof(uint8_t), 512, fp_imr);
  
  imu_t imu;
  init_imu(&imu);
  uint8_t body_buf[32];
  while (size_t n = fread(buf, 1, 1034, fp_imu_pack))
  {
    for (size_t i = 0; i < n; i++) {
      if (!(input_imu(&imu, buf[i]) == 1)) continue;
      idx = 0;
      double tow = time2gpst(imu.time, NULL);
      memcpy(body_buf + idx, &tow, 8); idx += 8;
      int32_tmp = round(imu.gyro[0] * R2D / gyro_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.gyro[1] * R2D / gyro_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.gyro[2] * R2D / gyro_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.acc[0] / acc_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.acc[1] / acc_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      int32_tmp = round(imu.acc[2] / acc_encode_factor);
      memcpy(body_buf + idx, &int32_tmp, 4); idx += 4;
      fwrite(body_buf, sizeof(uint8_t), 32, fp_imr);
    }
  }
  
  free_imu(&imu);
  fclose(fp_imu_pack);
  fclose(fp_imr);

  return 0;
}