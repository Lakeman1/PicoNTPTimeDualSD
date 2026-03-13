#pragma once
#include <Arduino.h>

void initNodeID();
uint8_t getNodeID();
bool setNodeID(uint8_t newID);