#include "joystick.h"

Joystick::Joystick(int _Ax, int _Ay)
{
    Ax = _Ax;
    Ay = _Ay;
}

Joystick::readx()
{
    return analogRead(Ax);
}

Joystick::ready()
{
    return analogRead(Ay);
}