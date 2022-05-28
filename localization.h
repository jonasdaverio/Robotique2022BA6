#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <ch.h>

#define SPATIAL_DIMENSIONS 3
#define ANGLE_DIMENSION 3
#define ACC_BUFFER_SIZE 50

void localization_init(uint8_t mode);

const float* get_speed(void);
const float* get_position(void);
const float* get_orientation(void);
void reset_speed(void);
void reset_position(void);
void reset_orientation(void);

#endif //LOCALIZATION_H
