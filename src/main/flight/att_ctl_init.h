#pragma once

#include "att_ctl.h"

void resetIndiProfile(indiProfile_t *profile);
void changeIndiProfile(uint8_t profileIndex);

void initIndiRuntime(void);
void indiInit(const pidProfile_t * pidProfile);
