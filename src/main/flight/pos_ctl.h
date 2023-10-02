
#include "common/maths.h"

#define MAX_ACC_Z_NEG -15.f // around 1.5g
#define MAX_ACC_Z_POS +9.81f // exactly 1g, for obvious reasons we cant do much more

//extern t_fp_vector posSpNed;
//extern t_fp_vector velSpNed;
extern t_fp_vector accSpNed;
extern float yawRateSpFromOuter;

void updatePosCtl(timeUs_t current);
void getAccSpNed(void);
