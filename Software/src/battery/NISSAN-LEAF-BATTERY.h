#ifndef NISSAN_LEAF_BATTERY_H
#define NISSAN_LEAF_BATTERY_H

#include "../include.h"

#define BATTERY_SELECTED
#define MAX_CELL_DEVIATION_MV 500

uint16_t Temp_fromRAW_to_F(uint16_t temperature);
bool is_message_corrupt(CAN_frame rx_frame);
void setup_battery(void);
void transmit_can(CAN_frame* tx_frame, int interface);

#endif
