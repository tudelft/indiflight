
#include "common/maths.h"

#define MAX_ACC_Z_NEG -20.f // around 2.0g
//#define MAX_ACC_Z_POS +9.5f // exactly 1g, for obvious reasons we cant do more
#define MAX_ACC_Z_POS +20.f // haha, inverted go brr

//extern t_fp_vector posSpNed;
//extern t_fp_vector velSpNed;
extern t_fp_vector accSpNed;
extern float yawRateSpFromOuter;

void updatePosCtl(timeUs_t current);
void getAccSpNed(timeUs_t current);
