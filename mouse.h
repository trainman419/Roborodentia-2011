#include <stdint.h>

#ifndef MOUSE_H
#define MOUSE_H

void mouse_tx(uint8_t m, uint8_t b);
uint8_t mouse_rx(uint8_t m);
void mouse_init();

#endif

