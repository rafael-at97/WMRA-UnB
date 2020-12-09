#include "joystick.h"

Joystick::Joystick(uint8_t btn, uint8_t x, uint8_t y)
{
    pin_btn = btn;
    pinMode(pin_btn, INPUT_PULLUP);

    pin_x = x;
    pin_y = y;

    x_min = 0;
    x_max = 1023;
    x_center = 512;

    y_min = 0;
    y_max = 1023;
    y_center = 512;
}

float Joystick::read(char dir)
{
    int raw;
    float out_min, out_max;
    float in_min, in_max;

    if( dir == 'x' || dir == 'X' )
    {
        raw = analogRead(pin_x);

        if( raw > (x_center - CALIBRATION_DEADZONE) && raw < (x_center + CALIBRATION_DEADZONE) )
            return 0;

        if( raw < (x_center - CALIBRATION_DEADZONE) )
        {
            out_min = x_min;
            out_max = ( x_max + x_min ) / 2;

            in_min = 0;
            in_max = x_center - CALIBRATION_DEADZONE;
        }
        else
        {
            out_min = ( x_max + x_min ) / 2;
            out_max = x_max;

            in_min = x_center + CALIBRATION_DEADZONE;
            in_max = 1023;
        }
        
        return (raw - in_min)*( ( out_max - out_min ) / (in_max - in_min) ) + out_min;
    }
    else if( dir == 'y' || dir == 'Y' )
    {
        raw = analogRead(pin_y);

        if( raw > (y_center - CALIBRATION_DEADZONE) && raw < (y_center + CALIBRATION_DEADZONE) )
            return 0;

        if( raw < (y_center - CALIBRATION_DEADZONE) )
        {
            out_min = y_min;
            out_max = ( y_max + y_min ) / 2;

            in_min = 0;
            in_max = y_center - CALIBRATION_DEADZONE;
        }
        else
        {
            out_min = ( y_max + y_min ) / 2;
            out_max = y_max;

            in_min = y_center + CALIBRATION_DEADZONE;
            in_max = 1023;
        }
            
        return (raw - in_min)*( ( out_max - out_min ) / (in_max - in_min) ) + out_min;
    }

    return 0;        
}

int Joystick::is_pressed()
{
    return !digitalRead(pin_btn);
}

int Joystick::set_ISR(void (*userFunc)(void))
{
    if(digitalPinToInterrupt(pin_btn) == NOT_AN_INTERRUPT)
        return false;

    attachInterrupt(digitalPinToInterrupt(pin_btn), userFunc, FALLING);

    return true;
}

void Joystick::set_limits(char* axis, float min, float max)
{
    uint8_t i = 0;

    if(axis == NULL)
        return;

    for( i=0 ; i<N_AXIS ; i++ )
    {
        if(axis[i] == '\0')
            break;
        
        if( axis[i] == 'x' || axis[i] == 'X' )
        {
            x_max = max;
            x_min = min;
        }
        else if( axis[i] == 'y' || axis[i] == 'Y')
        {
            y_max = max;
            y_min = min;
        }
    }
}

void Joystick::calibrate(void)
{
    uint8_t i;

    delay(500);

    /* Zero to avoid interfering with average */ 
    x_center = y_center = 0;

    /* Simple average */
    for( i=0 ; i < CALIBRATION_WINDOW ; i++ )
    {
        x_center += analogRead(pin_x);
        y_center += analogRead(pin_y);
    }
    x_center = x_center/CALIBRATION_WINDOW;
    y_center = y_center/CALIBRATION_WINDOW;
}