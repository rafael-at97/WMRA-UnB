#include "motors.h"

/****************
 * DC functions *
 ****************/

DCMotor::DCMotor(uint8_t L_PWM, uint8_t R_PWM)
{
    driver.btsdriver = new BTS7960(L_PWM, R_PWM);

    k = 12.5;
    pwm_deadzone = 30;
}

DCMotor::~DCMotor()
{
    delete driver.btsdriver;
}

void DCMotor::set_limits(float min, float max)
{
    qlim[0] = min;
    qlim[1] = max;
}

void DCMotor::set_angle_callbacks(uint8_t pin, void (*setup)(uint8_t), float (*read)(uint8_t, float[2]))
{
    angle_pin = pin;

    if(setup != NULL)
    {
        setup(angle_pin);
    }
    else
    {
        /* Simply set as input */
        pinMode(pin, INPUT);
    }

    angle_read = read;
}

void DCMotor::set_speed(float speed)
{
    if(speed >= 0.0f)
    {
        set_dir(BTS7960_CW);
        r = speed;
    }
    else
    {
        set_dir(BTS7960_CCW);
        r = -speed;
    }
}

void DCMotor::control_update()
{
    int pwm;

    pwm = r*k;

    if(pwm < pwm_deadzone && pwm != 0)
        pwm = pwm_deadzone;

    driver.btsdriver->setSpeed(pwm);
}

void DCMotor::set_dir(bool dir)
{
    driver.btsdriver->setDirection(dir);
}

/*********************
 * Stepper functions *
 *********************/

StepperMotor::StepperMotor(uint8_t step_pin, uint8_t dir_pin, uint8_t res, float step_angle)
{
    this->step_pin = step_pin;
    this->dir_pin = dir_pin;

    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);

    digitalWrite(dir_pin, HIGH);

    this->angle = 0.0f;

    this->res = res;

    word = true;

    uint8_t timer_pin = digitalPinToTimer(step_pin);

    uint8_t cs = 0;

    float temp;

    #if defined(TCCR0A) && defined(WGM00) && defined(WGM01) \
     && defined(TCCR0B) && defined(WGM02) \
     && defined(TIMSK0)
    if(timer_pin == TIMER0A || timer_pin == TIMER0B)
    {
        /* Set normal mode */
        bitClear(TCCR0A, WGM00);
        bitClear(TCCR0A, WGM01);
        bitClear(TCCR0B, WGM02);       

        /* Interrupt port */
        this->int_port = &TIMSK0;
        
        (this->cnt_reg).byte = (volatile uint8_t *)&TCNT0;
        
        cs |= TCCR0B & CS_MASK;

        word = false;

        #if defined(COM0A0) && defined(COM0A1) && defined(OCIE0A)
        if(timer_pin == TIMER0A)
        {
            /* Disconnect auto-update */
            bitClear(TCCR0A, COM0A0); 
            bitClear(TCCR0A, COM0A1);

            (this->oc_reg).byte = (volatile uint8_t *)&OCR0A;

            /* Interrupt pin */
            this->int_pin = OCIE0A;
        }
        #endif
        #if defined(COM0B0) && defined(COM0B1) && defined(OCIE0B)
        else
        {
            /* Disconnect auto-update */
            bitClear(TCCR0A, COM0B0);
            bitClear(TCCR0A, COM0B1);

            (this->oc_reg).byte = (volatile uint8_t *)&OCR0B;
            
            /* Interrupt pin */
            this->int_pin = OCIE0B;
        }
        #endif
    }
    #endif

    #if defined(TCCR1A) && defined(WGM10) && defined(WGM11) \
     && defined(TCCR1B) && defined(WGM12) && defined(WGM13) \
     && defined(TIMSK1)
    if(timer_pin == TIMER1A || timer_pin == TIMER1B || timer_pin == TIMER1C)
    {
        /* Set normal mode */
        bitClear(TCCR1A, WGM10);
        bitClear(TCCR1A, WGM11);
        bitClear(TCCR1B, WGM12);
        bitClear(TCCR1B, WGM13);

        this->int_port = &TIMSK1;

        (this->cnt_reg).word = (volatile uint16_t *)&TCNT1;

        cs |= TCCR1B & CS_MASK;

        #if defined(COM1A0) && defined(COM1A1) && defined(OCIE1A)
        if(timer_pin == TIMER1A)
        {
            /* Disconnect auto-update */
            bitClear(TCCR1A, COM1A0); 
            bitClear(TCCR1A, COM1A1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR1A;

            /* Interrupt pin */
            this->int_pin = OCIE1A; 
        }
        #endif
        #if defined(COM1B0) && defined(COM1B1) && defined(OCIE1B)
        else if(timer_pin == TIMER1B)
        {
            /* Disconnect auto-update */
            bitClear(TCCR1A, COM1B0); 
            bitClear(TCCR1A, COM1B1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR1B;

            /* Interrupt pin */
            this->int_pin = OCIE1B;
        }
        #endif
        #if defined(COM1C0) && defined(COM1C1) && defined(OCIE1C)
        else
        {
            /* Disconnect auto-update */
            bitClear(TCCR1A, COM1C0); 
            bitClear(TCCR1A, COM1C1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR1C;

            /* Interrupt pin */
            this->int_pin = OCIE1C;
        }
        #endif
    }
    #endif

    #if defined(TCCR2A) && defined(WGM20) && defined(WGM21) \
     && defined(TCCR2B) && defined(WGM22) \
     && defined(TIMSK2)
    if(timer_pin == TIMER2A || timer_pin == TIMER2B)
    {
        /* Set normal mode */
        bitClear(TCCR2A, WGM20);
        bitClear(TCCR2A, WGM21);
        bitClear(TCCR2B, WGM22);       

        /* Interrupt port */
        this->int_port = &TIMSK2;

        (this->cnt_reg).byte = (volatile uint8_t *)&TCNT2;
        
        cs |= TCCR2B & CS_MASK;

        word = false;

        #if defined(COM2A0) && defined(COM2A1) && defined(OCIE2A)
        if(timer_pin == TIMER2A)
        {
            /* Disconnect auto-update */
            bitClear(TCCR2A, COM2A0); 
            bitClear(TCCR2A, COM2A1);

            (this->oc_reg).byte = (volatile uint8_t *)&OCR2A;

            /* Interrupt pin */
            this->int_pin = OCIE2A;
        }
        #endif
        #if defined(COM2B0) && defined(COM2B1) && defined(OCIE2B)
        else
        {
            /* Disconnect auto-update */
            bitClear(TCCR2A, COM2B0);
            bitClear(TCCR2A, COM2B1);

            (this->oc_reg).byte = (volatile uint8_t *)&OCR2B;

            /* Interrupt pin */
            this->int_pin = OCIE2B;
        }
        #endif
    }
    #endif

    #if defined(TCCR3A) && defined(WGM30) && defined(WGM31) \
     && defined(TCCR3B) && defined(WGM32) && defined(WGM33) \
     && defined(TIMSK3)
    if(timer_pin == TIMER3A || timer_pin == TIMER3B || timer_pin == TIMER3C)
    {
        /* Set normal mode */
        bitClear(TCCR3A, WGM30);
        bitClear(TCCR3A, WGM31);
        bitClear(TCCR3B, WGM32);
        bitClear(TCCR3B, WGM33);

        this->int_port = &TIMSK3;

        (this->cnt_reg).word = (volatile uint16_t *)&TCNT3;

        cs |= TCCR3B & CS_MASK;
        
        #if defined(COM3A0) && defined(COM3A1) && defined(OCIE3A)
        if(timer_pin == TIMER3A)
        {
            /* Disconnect auto-update */
            bitClear(TCCR3A, COM3A0); 
            bitClear(TCCR3A, COM3A1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR3A;

            /* Interrupt pin */
            this->int_pin = OCIE3A; 
        }
        #endif
        #if defined(COM3B0) && defined(COM3B1) && defined(OCIE3B)
        else if(timer_pin == TIMER3B)
        {
            /* Disconnect auto-update */
            bitClear(TCCR3A, COM3B0); 
            bitClear(TCCR3A, COM3B1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR3B;

            /* Interrupt pin */
            this->int_pin = OCIE3B;
        }
        #endif
        #if defined(COM3C0) && defined(COM3C1) && defined(OCIE3C)
        else
        {
            /* Disconnect auto-update */
            bitClear(TCCR3A, COM3C0); 
            bitClear(TCCR3A, COM3C1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR3C;

            /* Interrupt pin */
            this->int_pin = OCIE3C;
        }
        #endif
    }
    #endif

    #if defined(TCCR4A) && defined(WGM40) && defined(WGM41) \
     && defined(TCCR4B) && defined(WGM42) && defined(WGM43) \
     && defined(TIMSK4)
    if(timer_pin == TIMER4A || timer_pin == TIMER4B || timer_pin == TIMER4C)
    {
        /* Set normal mode */
        bitClear(TCCR4A, WGM40);
        bitClear(TCCR4A, WGM41);
        bitClear(TCCR4B, WGM42);
        bitClear(TCCR4B, WGM43);

        this->int_port = &TIMSK4;

        (this->cnt_reg).word = (volatile uint16_t *)&TCNT4;

        cs |= TCCR4B & CS_MASK;
        
        #if defined(COM4A0) && defined(COM4A1) && defined(OCIE4A)
        if(timer_pin == TIMER4A)
        {
            /* Disconnect auto-update */
            bitClear(TCCR4A, COM4A0); 
            bitClear(TCCR4A, COM4A1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR4A;

            /* Interrupt pin */
            this->int_pin = OCIE4A; 
        }
        #endif
        #if defined(COM4B0) && defined(COM4B1) && defined(OCIE4B)
        else if(timer_pin == TIMER4B)
        {
            /* Disconnect auto-update */
            bitClear(TCCR4A, COM4B0); 
            bitClear(TCCR4A, COM4B1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR4B;

            /* Interrupt pin */
            this->int_pin = OCIE4B;
        }
        #endif
        #if defined(COM4C0) && defined(COM4C1) && defined(OCIE4C)
        else
        {
            /* Disconnect auto-update */
            bitClear(TCCR4A, COM4C0); 
            bitClear(TCCR4A, COM4C1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR4C;

            /* Interrupt pin */
            this->int_pin = OCIE4C;
        }
        #endif
    }
    #endif

    #if defined(TCCR5A) && defined(WGM50) && defined(WGM51) \
     && defined(TCCR5B) && defined(WGM52) && defined(WGM53) \
     && defined(TIMSK5)
    if(timer_pin == TIMER5A || timer_pin == TIMER5B || timer_pin == TIMER5C)
    {
        /* Set normal mode */
        bitClear(TCCR5A, WGM50);
        bitClear(TCCR5A, WGM51);
        bitClear(TCCR5B, WGM52);
        bitClear(TCCR5B, WGM53);

        this->int_port = &TIMSK5;

        (this->cnt_reg).word = (volatile uint16_t *)&TCNT5;

        cs |= TCCR5B & CS_MASK;
        
        #if defined(COM5A0) && defined(COM5A1) && defined(OCIE5A)
        if(timer_pin == TIMER5A)
        {
            /* Disconnect auto-update */
            bitClear(TCCR5A, COM5A0); 
            bitClear(TCCR5A, COM5A1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR5A;

            /* Interrupt pin */
            this->int_pin = OCIE5A; 
        }
        #endif
        #if defined(COM5B0) && defined(COM5B1) && defined(OCIE5B)
        else if(timer_pin == TIMER5B)
        {
            /* Disconnect auto-update */
            bitClear(TCCR5A, COM5B0); 
            bitClear(TCCR5A, COM5B1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR5B;

            /* Interrupt pin */
            this->int_pin = OCIE5B;
        }
        #endif
        #if defined(COM5C0) && defined(COM5C1) && defined(OCIE5C)
        else
        {
            /* Disconnect auto-update */
            bitClear(TCCR5A, COM5C0); 
            bitClear(TCCR5A, COM5C1);

            (this->oc_reg).word = (volatile uint16_t *)&OCR5C;

            /* Interrupt pin */
            this->int_pin = OCIE5C;
        }
        #endif
    }
    #endif

    /* Save prescale factor on temporary variable */
    switch(cs)
    {
        case 1:
            temp = 1;
            break;
        case 2:
            temp = 8;
            break;
        case 3:
            temp = 64;
            break;
        case 4:
            temp = 256;
            break;
        case 5:
            temp = 1024;
            break;
        default:
            break;
    }

    /* Start deactivated */
    bitClear(*(this->int_port), this->int_pin);

    this->step_angle = step_angle;
    temp = 1 / (FREQUENCY / temp); /* 16Mhz and prescaler */

    this->cte1 = step_angle/temp;
}

void StepperMotor::set_speed(float desired_speed)
{
    float counter, speed;

    /* Get current speed */
    if(stopped)
        speed = 0.0f;
    else
    {
        speed = ( cte1/curr_counter );
        if(curr_direction == CW)
            speed = -speed;
    }

    /* If different signs, first go to 0 */
    if(speed*desired_speed < 0.0f)
        desired_speed = 0.0f;

    if(desired_speed >= 0.0f)
        set_dir(CCW);
    else
    {
        set_dir(CW);
        desired_speed = -desired_speed;
    }

    if(desired_speed < ERROR)
    {
        if(!stopped)
        {
            /* Disable interrupt */
            bitClear(*int_port, int_pin);

            /* LOW to avoid unnecessary SET */
            digitalWrite(step_pin, LOW);

            /* Set stopped flag */
            stopped = true;
        }
    }
    else
    {
        /* Calculate maximum value based on acceleration */
        if(desired_speed > speed)
        {
            counter = (-speed + sqrt(speed*speed + cte2))*cte3;

            desired_speed = (desired_speed > cte1/counter) ? cte1/counter : desired_speed;
        }

        /* Limit due to 16 bit precision */
        if(desired_speed < W_MIN)
            desired_speed = W_MIN;

        /* Use float and only then store on uint16_t */
        counter = ( cte1/desired_speed );
        if(counter > (float)(__UINT16_MAX__*2) - ERROR)
            desired_counter = __UINT16_MAX__;
        else
            desired_counter = (uint16_t)( counter/2 ); /* Divide by 2 to generate two pulses, one to RESET, other to SET */        
    
        if(stopped)
        {
            /* Enable interrupt */
            bitSet(*(this->int_port), this->int_pin);

            /* Force interrupt */
            if(word)
            {
                *((this->oc_reg).word) = *((this->cnt_reg).word) + 10;
            }
            else
            {
                *((this->oc_reg).byte) = *((this->cnt_reg).byte) + 10;
            }
        }
    }
}

void StepperMotor::do_step()
{
    /* If High-to-low, simply write low */
    if(level == true)
    {
        digitalWrite(step_pin, LOW);
        level = false;
    }
    else
    {
        digitalWrite(step_pin, HIGH);
        level = true;

        if(curr_direction == CCW)
            angle += RAD_TO_DEG*step_angle;
        else
            angle -= RAD_TO_DEG*step_angle;
    }

    curr_counter = desired_counter;

    /* Update flag */
    stopped = false;

    if(word)
        *((oc_reg).word) = *((cnt_reg).word) + curr_counter;
    else
        *((oc_reg).byte) = *((cnt_reg).byte) + curr_counter;
}

void StepperMotor::set_dir(bool dir)
{
    if(curr_direction != dir)
    {
        switch(dir)
        {
            case CCW:
                digitalWrite(dir_pin, HIGH);
                curr_direction = CCW;
                break;
            case CW:
                digitalWrite(dir_pin, LOW);
                curr_direction = CW;
                break;

            default:
                break;
        }
    }
}

void StepperMotor::start(float speed, volatile bool* end, float angle)
{
    /* Save AVR Status */
    uint8_t oldSREG = SREG;

    /* Set desired speed */
    set_speed(speed);

    /* Make sure interrupts are enabled */
    sei();

    while(!(*end)); /* While flag hasn't been raised */

    set_speed(0.0f); /* Stop */
    set_angle(angle); /* Attibute angle value */

    /* Restore AVR Status */
    SREG = oldSREG;
}