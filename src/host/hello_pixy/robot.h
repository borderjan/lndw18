/************************
 * Robot Specific Stuff *
 ************************/
#ifndef ROBOT_H
#define ROBOT_H

#include "gpio.h"

static pinmask hs_rx, hs_tx;
static pinmask data_rx, data_tx;

extern int do_handshake_master();
extern int do_handshake_slave();

extern int write_data(int data);
extern int read_data(int *data);

#endif
