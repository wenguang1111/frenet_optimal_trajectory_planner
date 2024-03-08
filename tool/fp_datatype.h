#include "cnl/scaled_integer.h"

typedef cnl::scaled_integer<int16_t, cnl::power<-9>> int_6_9;
typedef cnl::scaled_integer<int16_t, cnl::power<-13>> int_2_13;
typedef cnl::scaled_integer<int16_t, cnl::power<-11>> int_4_11;
typedef cnl::scaled_integer<int16_t, cnl::power<-12>> int_3_12;
typedef cnl::scaled_integer<int16_t, cnl::power<-14>> int_1_14;
typedef cnl::scaled_integer<uint16_t, cnl::power<-13>> uint_3_13;
typedef cnl::scaled_integer<uint16_t, cnl::power<-10>> uint_6_10;
typedef cnl::scaled_integer<uint16_t, cnl::power<-6>> uint_10_6;
