#ifndef BTS7960_H
#define BTS7960_H

#include <Arduino.h>

#define BTS7960_CW  false
#define BTS7960_CCW true

class BTS7960
{
    private:
        uint8_t L_PWM;
        uint8_t R_PWM;

        bool dir;   /* 0 - Clockwise, 1 - CCW*/

    public:
        BTS7960(uint8_t L_PWM, uint8_t R_PWM);

        void setDirection(bool dir);
        void setSpeed(uint8_t pwm);
};

#endif