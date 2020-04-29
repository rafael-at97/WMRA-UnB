#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

class Joystick
{
    private:
        int Ax;
        int Ay;

    public:
        Joystick(int _Ax, int _Ay);

        int readx();

        int ready();
}

#endif