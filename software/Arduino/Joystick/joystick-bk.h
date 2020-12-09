/*
 * This code deals with the readings for a joystick made of two perpendicular potentiometers
 * 
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

#define N_AXIS  (2)

#define ADC_MAX (1023)

#define CALIBRATION_WINDOW      (10)
#define CALIBRATION_DEADZONE    (20.0) /* Approximately 2% of 1024 */

class Joystick
{
    private:
        uint8_t pin_btn;
        uint8_t pin_x;
        uint8_t pin_y;

        float x_max;
        float x_min;
        float x_center;
        
        float y_max;
        float y_min;
        float y_center;

    public:
        Joystick(uint8_t btn, uint8_t x, uint8_t y);

        float read(char dir);
        int is_pressed();
        int set_ISR(void (*userFunc)(void));

        void set_limits(char* axis, float min, float max);
        void calibrate(void);
};

#endif