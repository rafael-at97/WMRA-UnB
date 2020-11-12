#include "robotic_arm.h"

robotic_arm::robotic_arm()
{
    /* Set option for variable number of DOFs */
}

void robotic_arm::set_link(uint8_t pos, float alpha, float a, float d, Motor * motor = NULL)
{
    pos = INDEX_COMPAT(pos);

    links[pos].alpha = alpha;
    links[pos].a = a;
    links[pos].d = d;
    
    links[pos].motor = motor;
}

float robotic_arm::read_angle(uint8_t pos)
{
    pos = INDEX_COMPAT(pos);

    if(links[pos].motor != NULL)
        links[pos].theta = (links[pos].motor)->get_angle();
    else
        links[pos].theta = 0.0f;    

    return links[pos].theta;
}

void robotic_arm::set_speed(uint8_t pos, float qdot)
{
    pos = INDEX_COMPAT(pos);

    if(links[pos].motor != NULL)
        (links[pos].motor)->set_speed(qdot);    
}