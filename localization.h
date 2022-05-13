#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "i2c_bus.h"
void localization_init(void);

const float* get_speed(void);
const float* get_position(void);
const float* get_orientation(void);
void reset_speed(void);
void reset_position(void);
void reset_orientation(void);

#endif //LOCALIZATION_H
