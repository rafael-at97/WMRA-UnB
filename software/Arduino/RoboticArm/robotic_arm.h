#ifndef ROBOTICARM_H
#define ROBOTICARM_H

#include <Arduino.h>
#include "motors.h"

#define INDEX_COMPAT(INDEX) (INDEX - 1)

#define DOF (6)

typedef struct link
{ 
    float alpha;
    float a;
    float d;
    float theta;

    Motor * motor;
} link;

class robotic_arm
{
    private:
        link links[DOF];

    public:
        robotic_arm();

        void set_link(uint8_t pos, float alpha, float a, float d, Motor * motor = NULL);

        float read_angle(uint8_t pos);

        void set_speed(uint8_t pos, float qdot);
};

#endif