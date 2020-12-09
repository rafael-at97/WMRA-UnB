#include "joystick.h"
#include <math.h>

Joystick::Joystick(uint8_t btn, uint8_t x, uint8_t y)
{
    pin_btn = btn;
    pinMode(pin_btn, INPUT_PULLUP);

    pin_x = x;
    pin_y = y;

    x_center = 512;

    y_center = 512;
}

void Joystick::read(float xy[N_AXIS])
{
    int raw;
    float max, max_1, max_2, max_2pos, max_2neg, value; /* Values for linear mapping */
    float x, y, angle;

    /* X value */
    raw = analogRead(pin_x);
    if( raw > (x_center - CALIBRATION_DEADZONE) && raw < (x_center + CALIBRATION_DEADZONE) )
        x = x_center;
    else
        x = raw*1.0;

    /* Y value */
    raw = analogRead(pin_y);
    if( raw > (y_center - CALIBRATION_DEADZONE) && raw < (y_center + CALIBRATION_DEADZONE) )
        y = y_center;
    else
        y = raw*1.0;

    angle = atan2(y - y_center, x - x_center);
    value = sqrt(pow(x - x_center, 2) + pow(y - y_center, 2));

    /* Get maximum value possible for said angle */
    if(fabs(angle) < PI/4 || fabs(angle) > (3*PI)/4)
    {
        if(fabs(angle) < PI/4 )
            max_1 = (ADC_MAX-x_center);
        else
            max_1 = (-x_center);
        
        max_2 = max_1*tan(angle);

        max_2pos = (ADC_MAX-y_center);   /* Limit for positive values */
        max_2neg = -y_center;            /* Limit for negative values */
    }
    else
    {   
        if(angle > 0)
            max_1 = (ADC_MAX-y_center);
        else
            max_1 = -y_center;
        
        max_2 = max_1/tan(angle);

        max_2pos = (ADC_MAX-x_center);  /* Limit for positive values */
        max_2neg = -x_center;           /* Limit for negative values */
    }

    /* Apply limits */
    if(max_2 > max_2pos)
        max_2 = max_2pos;           /* Limit for positive values */
    else if(max_2 < max_2neg)
        max_2 = max_2neg;           /* Limit for negative values */    
    
    max = sqrt(pow(max_1, 2) + pow(max_2, 2));

    value = (value/max)*(out_max - out_min);

    xy[0] = value*cos(angle); /* x value */
    xy[1] = value*sin(angle); /* y value */      
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

void Joystick::set_limits(float min, float max)
{
    this->out_min = min;
    this->out_max = max;
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