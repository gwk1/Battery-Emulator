#ifndef BMW_PHEV_BATTERY_H
#define BMW_PHEV_BATTERY_H
#include "../include.h"

#define BATTERY_SELECTED
#define MAX_CELL_DEVIATION_MV 9999
#define CELLS_PER_MODULE 16

/** MODULES * STRINGS =< 14**/
#define MODULES_PER_STRING 6
#define PARALLEL_STRINGS 1

void setup_battery(void);
void transmit_can(CAN_frame* tx_frame, int interface);

#endif
