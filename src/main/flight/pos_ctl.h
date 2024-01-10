
#include "common/maths.h"
#include "common/time.h"

#define MAX_ACC_Z_NEG -20.f // around 2.0g

#ifndef MAX_ACC_Z_POS
#define MAX_ACC_Z_POS +9.5f // exactly 1g, so maximum commanded descend is falling
//#define MAX_ACC_Z_POS +20.f // haha, inverted go brr
#endif

//extern t_fp_vector posSpNed;
//extern t_fp_vector velSpNed;
extern t_fp_vector accSpNed;
extern float yawRateSpFromOuter;

void updatePosCtl(timeUs_t current);
void getAccSpNed(timeUs_t current);
