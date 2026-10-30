#ifndef OSC_TRIGGER_H
#define OSC_TRIGGER_H
#include "main.h"

int16_t OSC_Rising_Edge_Trigger(uint32_t *data, int size, int16_t threshold, int16_t hysteresis);
int16_t OSC_Falling_Edge_Trigger(uint32_t *data, int size, int16_t threshold, int16_t hysteresis);
int16_t OSC_Trigger_Potential(uint32_t* data, int size, int16_t threshold);

#endif