#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "BTS7960.h"

#define CW  false
#define CCW true

/** Defines for steppers **/
#define W_DOT       (0.4)
#define W_MIN       (0.015)

#define ERROR       (0.005)

#define CS_MASK 0x7
#define FREQUENCY   (16000000.0)

#define MAX_COUNT   (30)

class Motor
{
    public:
        virtual void set_speed(float) = 0;
        virtual void set_dir(bool) = 0;

        virtual float get_angle() = 0;

        virtual ~Motor() = default;
};

class DCMotor : public Motor
{
    private:
        float qlim[2] = {0.0f, 0.0f};
        
        union Driver
        {
            BTS7960 * btsdriver;
        } driver;
        
        uint8_t angle_pin;

        float (*angle_read)(uint8_t, float[2]);

    public:
        /** Constructor for using BTS7960
         */
        DCMotor(uint8_t L_PWM, uint8_t R_PWM);

        ~DCMotor();

        void set_limits(float min, float max);

        /** Set callback for angle measurement
         */
        void set_angle_callbacks(uint8_t pin, void (*setup)(uint8_t), float (*read)(uint8_t, float[2]));

        void set_speed(float speed);
        void set_dir(bool dir);

        float get_angle()
        {
            if(angle_read != NULL)
                return angle_read(angle_pin, qlim);

            return 0.0f;
        }
};

class StepperMotor : public Motor
{
    private:
        uint8_t step_pin;
        uint8_t dir_pin;

        /** Step resolution
         *  1 -> Full Step
         *  2 -> Half Step
         *  4 -> 1/4 Step
         *  8 -> 1/8 Step
         *  ...
         */
        uint8_t res;

        /* Timers use byte or word for counters? */
        bool word;

        /* Output compare register */
        union OC_REG
        {
            volatile uint8_t * byte;
            volatile uint16_t * word;
        } oc_reg;

        /* Counter register */
        union CNT_REG
        {
            volatile uint8_t * byte;
            volatile uint16_t * word;
        } cnt_reg;

        /* Interrupt variables */
        volatile uint8_t * int_port;
        uint8_t int_pin;

        float step_angle;
        
        /* Variables for speed <-> counter transformation */
        float cte1; 
        float cte2;
        float cte3;

        volatile uint16_t desired_counter;
        volatile uint16_t curr_counter;

        bool stopped = true;

        /* True -> Next change is HIGH to LOW */
        volatile bool level = false;

        bool curr_direction = CW;

        float angle;

        /** Set angle, in degrees.
         */
        void set_angle(float offset)
        {
            angle = offset;
        }

    public:
        /** IMPORTANT: Timer responsible for step_pin will be set as normal mode, so, any PWM connected to the same timer will
         *             stop working as expected.
         */
        StepperMotor(uint8_t step_pin, uint8_t dir_pin, uint8_t res, float step_angle);

        /** Place this inside the appropriate interrupt.
         */
        void do_step();

        /** Based on Arduino's analogWrite().
         * 
         *  @param  speed   Speed, in m/s.
         */
        void set_speed(float speed);

        void set_dir(bool dir);

        float get_angle()
        {
            return angle;
        }

        void start(float speed, volatile bool* end, float angle);
};

#endif