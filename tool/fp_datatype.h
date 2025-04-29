#ifndef FP_DATATYPE_H
#define FP_DATATYPE_H

#include "cnl/scaled_integer.h"

// //----------fot_wrapper.cpp-----------
#define fixp_wx fp_type
#define fixp_wy fp_type
#define fixp_o_llx fp_type
#define fixp_o_lly fp_type
#define fixp_o_urx fp_type
#define fixp_o_ury fp_type
#define fixp_s fp_type
#define fixp_x fp_type
#define fixp_y fp_type
//--------------------------------------

#define fixp_position_error fixp_x(0)
#define fixp_t fp_type
#define fixp_d fp_type
#define fixp_d_d fp_type
#define fixp_d_dd fp_type
#define fixp_d_ddd fp_type

#define fixp_s_d fp_type
#define fixp_s_dd fp_type
#define fixp_s_ddd fp_type
#define fixp_dt fp_type
#define fixp_intermediate fp_type
#define fixp_max_road_width fp_type
#define fixp_tv fp_type

#define fixp_yaw fp_type
#define fixp_ds fp_type
#define fixp_c fp_type
#define fixp_quarticpolynomial_A0 fp_type
#define fixp_quarticpolynomial_A1_A4 fp_type
#define fixp_quinticpolynomial_A0_A5 fp_type
// #define fixp_time_2 uint_54_10
// #define fixp_time_3 uint_54_10
// #define fixp_time_4 uint_54_10
// #define fixp_time_5 uint_54_10
#define fixp_cost fp_type
#define fixp_lateral_jerk fp_type
#define fixp_docProduct fp_type
#define fixp_sum_cost fp_type
#define fixp_inverse_distanceToObstacles fp_type
#define fixp_mincost fp_type
#define fixp_c_d fp_type
#define fixp_target_speed fp_type
#define fixp_lateral_acceleration fp_type
#define fixp_lateral_deviation fp_type
#define fixp_lateral_velocity fp_type
#define fixp_longitudinal_acceleration fp_type
#define fixp_longitudinal_jerk fp_type
#define fixp_end_speed_deviation fp_type
#define fixp_inv_dist_to_obstacles fp_type
#define fixp_c_lateral fp_type
#define fixp_longitudinal fp_type
#define fixp_cf fp_type
#define fixp_quarticpolynomial_K1 fp_type
#define fixp_quarticpolynomial_K2 fp_type
#define fixp_quinticpolynomial_K  fp_type
#define fixp_quinticpolynomial_K0 fp_type
#define fixp_quinticpolynomial_K1 fp_type
#define fixp_quinticpolynomial_K2 fp_type
#define fixp_cum_sum fp_type
#define fixp_dx fp_type
#define fixp_dy fp_type
#define fixp_cubicspline_b fp_type
#define fixp_cubicspline_c fp_type
#define fixp_cubicspline_d fp_type
#define fixp_cubicspline_m fp_type
#define fixp_TM_a fp_type
#define fixp_TM_b fp_type
#define fixp_TM_c fp_type
#define fixp_TM_d fp_type
#define fixp_obstacle_clearance fp_type
#define fixp_vx fp_type
#define fixp_speeds fp_type
#define fixp_30_33 fp_type

#define Dummy 20
#define Trigonometric_Precision Dummy
#define Time_Precision Dummy
#define Standard_precision Dummy
typedef cnl::scaled_integer<int64_t, cnl::power<-Trigonometric_Precision>>  Trignometric;
typedef cnl::scaled_integer<int64_t, cnl::power<-Standard_precision>>  fp_type;
typedef cnl::scaled_integer<int64_t, cnl::power<-Time_Precision>>  fp_time;
 
typedef cnl::scaled_integer<int16_t, cnl::power<-14>> int_1_14;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;
typedef cnl::scaled_integer<int16_t, cnl::power<-12>> int_3_12;
typedef cnl::scaled_integer<int16_t, cnl::power<-11>> int_4_11;
typedef cnl::scaled_integer<int16_t, cnl::power<-10>>  int_5_10;
typedef cnl::scaled_integer<int16_t, cnl::power<-9>> int_6_9;
typedef cnl::scaled_integer<int16_t, cnl::power<-8>>  int_7_8;
typedef cnl::scaled_integer<int16_t, cnl::power<-7>>  int_8_7;
typedef cnl::scaled_integer<int16_t, cnl::power<-6>> int_9_6;
typedef cnl::scaled_integer<int16_t, cnl::power<-5>> int_10_5;
typedef cnl::scaled_integer<int16_t, cnl::power<-4>> int_11_4;
typedef cnl::scaled_integer<int16_t, cnl::power<-3>> int_12_3;
typedef cnl::scaled_integer<int16_t, cnl::power<-2>> int_13_2;
typedef cnl::scaled_integer<int32_t, cnl::power<-12>> int_19_12;
typedef cnl::scaled_integer<int64_t, cnl::power<-10>> int_53_10;
typedef cnl::scaled_integer<int64_t, cnl::power<-33>> int_30_33;

typedef cnl::scaled_integer<uint16_t, cnl::power<-14>> uint_2_14;
typedef cnl::scaled_integer<uint16_t, cnl::power<-13>> uint_3_13;
typedef cnl::scaled_integer<uint16_t, cnl::power<-12>> uint_4_12;
typedef cnl::scaled_integer<uint16_t, cnl::power<-11>> uint_5_11;
typedef cnl::scaled_integer<uint16_t, cnl::power<-10>> uint_6_10;
typedef cnl::scaled_integer<uint16_t, cnl::power<-9>> uint_7_9;
typedef cnl::scaled_integer<uint16_t, cnl::power<-8>> uint_8_8;
typedef cnl::scaled_integer<uint16_t, cnl::power<-7>> uint_9_7;
typedef cnl::scaled_integer<uint16_t, cnl::power<-6>> uint_10_6;
typedef cnl::scaled_integer<uint16_t, cnl::power<-5>> uint_11_5;
typedef cnl::scaled_integer<uint16_t, cnl::power<-4>> uint_12_4;
typedef cnl::scaled_integer<uint32_t, cnl::power<-12>> uint_20_12;
typedef cnl::scaled_integer<uint64_t, cnl::power<-10>> uint_54_10;
#endif //FP_DATATYPE_H