
#include "platform.h"
#include "common/time.h"
#include "flight/indi.h"
#include "flight/indi_init.h"
#include "fc/init.h"
#include <stdio.h>

timeUs_t current = 0;

static void run(void) {
    indiController(current);
    printf("%f", indiRun.d[0]);
}

int main(void) {
    init();
    run();
    return 0;
}