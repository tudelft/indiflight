/*
(C) tblaha 2023
 */

#pragma once

#include <string.h>

typedef struct hilInput_s {
    float gyro[3]; // deg/s
    float acc[3]; // g
    float rpm[4]; // rpm
} hilInput_t;

extern hilInput_t hilInput;
extern bool hilThrowDetected;

//typedef struct hilOutput_s {
//    float actuator_set[4];
//} hilOutput_t;

//extern hilOutput_t hilOutput;

void initHil(void);
void handleHil(void);
void checkHilState(void);

void freeHilPort(void);
void configureHilPort(void);

void hilSendActuators(void);
