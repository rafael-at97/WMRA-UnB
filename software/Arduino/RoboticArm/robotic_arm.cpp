#include "robotic_arm.h"

robotic_arm::robotic_arm()
{
    /* Set option for variable number of DOFs */
}

void robotic_arm::set_link(uint8_t pos, float alpha, float a, float d)
{
    pos = INDEX_COMPAT(pos);

    links[pos].alpha = alpha;
    links[pos].a = a;
    links[pos].d = d;
}

void robotic_arm::set_link_limits(uint8_t pos, float min, float max)
{
    pos = INDEX_COMPAT(pos);

    links[pos].qlim[0] = min;
    links[pos].qlim[1] = max;
}

void robotic_arm::set_link_callbacks(uint8_t pos, void (*setup)(), float (*read)(uint8_t, float[2]), uint8_t port)
{
    pos = INDEX_COMPAT(pos);

    links[pos].setup = setup;
    links[pos].read = read;
    links[pos].port = port;
}

void robotic_arm::setup_link(uint8_t pos)
{
    links[INDEX_COMPAT(pos)].setup();
}

float robotic_arm::read_angle(uint8_t pos)
{
    pos = INDEX_COMPAT(pos);
    
    links[pos].theta = links[pos].read(links[pos].port, links[pos].qlim);

    return links[pos].theta;
}
