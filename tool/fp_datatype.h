#include "cnl/scaled_integer.h"

#define fixp_t int_3_12
#define fixp_d int_3_12
#define fixp_d_d int_3_12
#define fixp_d_dd int_4_11
#define fixp_d_ddd int_7_8
#define fixp_s int_8_7
#define fixp_s_d int_5_10
#define fixp_s_dd int_5_10
#define fixp_s_ddd int_5_10
#define fixp_x int_8_7
#define fixp_y int_3_12
#define fixp_yaw int_1_14
#define fixp_ds int_3_12
#define fixp_c int_3_12

typedef cnl::scaled_integer<int16_t, cnl::power<-7>>  int_8_7;
typedef cnl::scaled_integer<int16_t, cnl::power<-8>>  int_7_8;
typedef cnl::scaled_integer<int16_t, cnl::power<-9>> int_6_9;
typedef cnl::scaled_integer<int16_t, cnl::power<-10>>  int_5_10;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;
typedef cnl::scaled_integer<int16_t, cnl::power<-11>> int_4_11;
typedef cnl::scaled_integer<int16_t, cnl::power<-12>> int_3_12;
typedef cnl::scaled_integer<int16_t, cnl::power<-14>> int_1_14;
typedef cnl::scaled_integer<uint16_t, cnl::power<-13>> uint_3_13;
typedef cnl::scaled_integer<uint16_t, cnl::power<-10>> uint_6_10;
typedef cnl::scaled_integer<uint16_t, cnl::power<-6>> uint_10_6;
