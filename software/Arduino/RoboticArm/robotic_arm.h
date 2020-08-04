#ifndef ROBOTICARM_H
#define ROBOTICARM_H

#include <Arduino.h>

#define INDEX_COMPAT(INDEX) (INDEX - 1)

#define DOF (6)

typedef struct link
{ 
    float alpha;
    float a;
    float d;
    float theta;
    float qlim[2];

    uint8_t port;
    void (*setup)();
    float (*read)(uint8_t, float[2]);
} link;

class robotic_arm
{
    private:
        link links[DOF];

    public:
        robotic_arm();

        void set_link(uint8_t pos, float alpha, float a, float d);
        void set_link_limits(uint8_t pos, float min, float max);
        void set_link_callbacks(uint8_t pos, void (*setup)(), float (*read)(uint8_t, float[2]), uint8_t port);

        void setup_link(uint8_t pos);
        float read_angle(uint8_t pos);
};

#endif