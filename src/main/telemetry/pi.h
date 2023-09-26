/*
(C) tblaha 2023
 */

#pragma once

#include <string.h>

void initPiTelemetry(void);
void handlePiTelemetry(void);
void checkPiTelemetryState(void);

void freePiTelemetryPort(void);
void configurePiTelemetryPort(void);
