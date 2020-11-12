#include "BTS7960.h"

BTS7960::BTS7960(uint8_t L_PWM, uint8_t R_PWM)
{
    this->L_PWM = L_PWM;
    this->R_PWM = R_PWM;
}

void BTS7960::setDirection(bool dir)
{
    this->dir = dir;
}

void BTS7960::setSpeed(uint8_t pwm)
{
    if(dir == BTS7960_CW)
    {
        
    }
}