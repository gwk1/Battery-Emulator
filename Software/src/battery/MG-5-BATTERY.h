#ifndef MG_5_BATTERY_H
#define MG_5_BATTERY_H
#include <Arduino.h>
#include "../include.h"

#define BATTERY_SELECTED
#define MAX_CELL_DEVIATION_MV 150

void setup_battery(void);
void transmit_can(CAN_frame* tx_frame, int interface);

#endif
