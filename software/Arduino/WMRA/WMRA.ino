#include "robotic_arm.h"
#include "motors.h"
#include "joystick.h"

#define SIMULATION 1

/** Micro switches defines **/
#define J1_S    (12)    
#define J1_E    (11)
#define J2_S    (21)
#define J2_E    (20)
#define J3_S    (19)
#define J3_E    (18)
#define J4_S    (10)
#define J4_E    (50)
#define J5_S    (51)
#define J5_E    (52)
#define J6_S    (53)

#define N_SW        (12) /* Number of switches used, 11 are actually used, the 12th is there for compatibility */
#define N_PC        (6)  /* Number of pin change interrupts */

#define START_POS   (0)
#define END_POS     (1)

#define INT_NORMAL  (0x0)
#define INT_PC      (0x1)

/** Micro switches flags **/
volatile bool uswitches[N_SW] = 
{
    false, false,
    false, false,
    false, false,
    false, false,
    false, false,
    false, false, 
};

const uint8_t PC_TO_IDX[] = 
{
    10, /* J6_S to uswitches[10] */
    9,  /* J5_E to uswitches[9]  */
    8,  /* J5_S to uswitches[8]  */
    7,  /* J4_E to uswitches[7]  */
    6,  /* J4_S to uswitches[6]  */
    1,  /* J1_E to uswitches[1]  */
    0,  /* J1_S to uswitches[0]  */
};

/* MATRIXES defines */
#define R1  (0) 
#define R2  (1)
#define R3  (2)
#define R4  (3)
#define R5  (4)
#define R6  (5)

#define C1  (0)
#define C2  (1)
#define C3  (2)
#define C4  (3)
#define C5  (4)
#define C6  (5)

/* Joystick planes of action */
#define XY          (0)
#define XZ          (1)
#define ROT         (2) /* For bitwise check of any rotation */
#define ROT_XZ      (2)
#define ROT_YZ      (3)
#define WORKSPACES  (4)

#define MAX_JS      (100)

float a1 = 0.014;
float a2 = 0.35;
float a3 = 0.35;
float a4 = 0.165;
float d6 = 0.198;
        
/** Motors **/
DCMotor *dc1;

StepperMotor *stepper1;

Joystick js(3, A3, A4);
robotic_arm wmra;

/* General maximum speed */
#define MAX_V   (0.15)  /* Linear  = m/s */
#define MAX_W   (0.80)  /* Angular = rad/s */

/* Maximum speed per joint, (rad/s) */
float max_speed[] = 
{
    //0.14,
    //0.14,
    //0.21,
    //0.41,
    //0.76,
    //0.79
    1.571,
    1.571,
    1.571,
    1.571,
    1.571,
    1.571
};

/** Joystick variables **/
uint8_t workspace = XY;

/** Joystick button debounce **/
volatile unsigned long last_isr = 0;
long debounce_time_ms = 500;

float not_zero(float val)
{
    if(fabs(val) < 1e-3)
        val = 1e-3;

    return val;
}

/** Calculates inverse jacobian.
 *  
 *  The values which appear too often on the calculations are left in form of variables, seeking to reduce the number of math operations
 *  needed in exchange for a little spent memory. 
 *  In total, this function uses:
 *                          - 126 Multiplications   (Divisions included)
 *                          - 47 Sums               (Subtractions included)
 *                          - 28 Auxiliar Variables (All 'float')
 *
 *  Note: Column and Row indexes are inverted because the math is done based on cofactor values, and 
 *        adjugate matrix calls for inversion.
 * 
 *  \param invJacobian Local for result of caculations to be stored
 *  \param q           Value for joints displacement values
 */
void calcInverseJacobian(float invJacobian[DOF][DOF], float q[DOF])
{   
    /** 
     * Inital calculations of most common used sines and cosines
     */
    float c1    = cos(q[0]);
    float s1    = sin(q[0]);
    float c5    = cos(q[4]);
    float s5    = sin(q[4]);
    float c234  = cos(q[1]+q[2]+q[3]);
    float s234  = sin(q[1]+q[2]+q[3]);

    /********************************/ 
    /* Calculations for determinant */
    /********************************/ 
    float S3_A2_A3      = not_zero(sin(q[2])*a2*a3);

    float C23_A3        = cos(q[1]+q[2])*a3;         
    float C2A2_C23A3    = cos(q[1])*a2 + C23_A3;

    float C5_D6         = c5*d6;                              
    float A4_C5D6       = a4 + C5_D6;

    float A1c           = a1 + C2A2_C23A3 + c234*( A4_C5D6 );
    float A1c_c234c5d6  = not_zero(A1c + c234*C5_D6);

    float S5_D6         = s5*d6;
    float temp0         = S5_D6 + S5_D6;

    float detJ_sing     = not_zero(-c5*A1c_c234c5d6); /* Without S3 term that causes singularity */
    float detJ          = not_zero(-c5*S3_A2_A3*A1c_c234c5d6);  

    /**************************/
    /* Third row calculations */
    /**************************/

    /*S23_A3 = s23*a3; <- Original name of variable */
    float temp1 = sin(q[1]+q[2])*a3;

    invJacobian[C1][R3] = invJacobian[C5][R3] = invJacobian[C6][R3] = 0.0f;
    invJacobian[C2][R3] = ( temp1 ) / S3_A2_A3;
    invJacobian[C3][R3] = ( -( sin(q[1])*a2 + temp1 ) ) / S3_A2_A3;
    invJacobian[C4][R3] = - ( invJacobian[C2][R3] + invJacobian[C3][R3] );

    /*************************/
    /* Some terms of 6th row */
    /*************************/
    
    invJacobian[C6][R6] = ( -s234*A1c ) / detJ_sing;
    invJacobian[C5][R6] = ( -c234*A1c ) / A1c_c234c5d6;

    /*C5D6_C5C234 = C5_D6*C234 <- Original name of variable */
    temp1 = C5_D6*c234;

    invJacobian[C1][R6] = ( temp1 ) / A1c_c234c5d6;

    /********************/ 
    /* 1st and 2nd rows */
    /********************/
    float S4_A3      = sin(q[3])*a3;
    float S34A2_S4A3 = sin(q[2]+q[3])*a2+S4_A3;

    float S5S234     = s5*s234;       

    float temp2 = c5*(s1*temp0 + c1*A1c_c234c5d6);
    float temp3 = S5S234*(A4_C5D6 + C5_D6)*S4_A3;
    float temp4 = S5S234*(A4_C5D6 + C5_D6)*S34A2_S4A3;

    invJacobian[C1][R1] = ( -s1 ) / A1c_c234c5d6;
    invJacobian[C5][R1] = invJacobian[C1][R1]*c234;
    invJacobian[C6][R1] = ( -s1*s234 ) / detJ_sing;
    invJacobian[C2][R1] = ( -C23_A3*temp2 + s1*temp3 ) / detJ;
    invJacobian[C3][R1] = ( C2A2_C23A3*temp2 - s1*temp4 ) / detJ;
    invJacobian[C4][R1] = - ( invJacobian[C2][R1] + invJacobian[C3][R1] ) - invJacobian[C6][R1]*s5;
    
    temp2 = c5*(c1*temp0 - s1*A1c_c234c5d6);

    invJacobian[C1][R2] = ( c1 ) / A1c_c234c5d6;
    invJacobian[C5][R2] = invJacobian[C1][R2]*c234;
    invJacobian[C6][R2] = ( c1*s234 ) / detJ_sing;
    invJacobian[C2][R2] = ( C23_A3*temp2 - c1*temp3 ) / detJ;
    invJacobian[C3][R2] = ( -C2A2_C23A3*temp2 + c1*temp4 ) / detJ;
    invJacobian[C4][R2] = - ( invJacobian[C2][R2] + invJacobian[C3][R2] ) - invJacobian[C6][R2]*s5;

    /**************************/
    /* Final terms of 6th row */
    /**************************/
    float C4_A3      = cos(q[3])*a3;
    float C34A2_C4A3 = cos(q[2]+q[3])*a2+C4_A3;

    S5S234 = S5S234*A1c;
    
    temp3 = A4_C5D6*S4_A3;
    temp4 = A4_C5D6*S34A2_S4A3;

    /*C5D6_C5C234_S5D6 = C5D6_C5C234*S5_D6; <- Original name of variable */
    temp1 = temp1*c5*S5_D6;

    /*C5D6_S5C234_A1c = C5_D6*s5*c234*A1c; <- Original name of variable */
    temp0 = C5_D6*s5*c234*A1c;

    invJacobian[C2][R6] = ( C23_A3*temp1 - temp0*C4_A3 + S5S234*temp3 ) / detJ;
    invJacobian[C3][R6] = ( -C2A2_C23A3*temp1 + temp0*C34A2_C4A3 - S5S234*temp4 ) / detJ;
    invJacobian[C4][R6] = - ( invJacobian[C2][R6] + invJacobian[C3][R6] ) - s5*invJacobian[C6][R6];

    /****************/
    /* Rows 4 and 5 */
    /****************/
    float A1cc234_c5d6 = A1c*c234 + C5_D6;
    
    invJacobian[C6][R4] = ( -c1*A1cc234_c5d6 ) / detJ_sing;
    invJacobian[C6][R5] = ( -s1*A1cc234_c5d6 ) / detJ_sing;

    /*C5S234_A1c = C5S234*A1c; <- Original name of variable */
    temp1 = s234*A1c;

    /*C5D6_C5S234 = C5_D6*temp1; <- Original name of variable */
    temp0 = C5_D6*s234;
    
    invJacobian[C5][R4] = ( c1*temp1 ) / A1c_c234c5d6;
    invJacobian[C5][R5] = ( s1*temp1 ) / A1c_c234c5d6;
    
    invJacobian[C1][R4] = ( -c1*temp0 ) / A1c_c234c5d6;
    invJacobian[C1][R5] = ( -s1*temp0 ) / A1c_c234c5d6;

    /*C5D6_S5S234 <- Original name of variable */
    S5S234 = C5_D6*S5S234;

    /*C5D6_C5S234_S5D6 = temp0*S5_D6; <- Original name of variable */
    temp0 = temp0*c5*S5_D6;
    
    temp2 = c5*s1*A1c_c234c5d6 - s5*c1*A1cc234_c5d6;
    
    invJacobian[C2][R4] = ( -c1*( C23_A3*temp0 - S5S234*C4_A3 ) - temp2*temp3 ) / detJ;
    invJacobian[C3][R4] = ( c1*( C2A2_C23A3*temp0 - S5S234*C34A2_C4A3 ) + temp2*temp4 ) / detJ;
    invJacobian[C4][R4] = - ( invJacobian[C2][R4] + invJacobian[C3][R4] ) - s5*invJacobian[C6][R4] - invJacobian[C1][R1]*A1c_c234c5d6;

    temp2 = c5*c1*A1c_c234c5d6 + s1*s5*A1cc234_c5d6;

    invJacobian[C2][R5] = ( -s1*( C23_A3*temp0 - S5S234*C4_A3 ) + temp2*temp3 ) / detJ;
    invJacobian[C3][R5] = ( s1*( C2A2_C23A3*temp0 - S5S234*C34A2_C4A3 ) - temp2*temp4 ) / detJ;
    invJacobian[C4][R5] = - ( invJacobian[C2][R5] + invJacobian[C3][R5] ) - s5*invJacobian[C6][R5] - invJacobian[C1][R2]*A1c_c234c5d6;
}

void calculate_speed(float invJacobian[DOF][DOF], float q[DOF], float qdot[DOF], float v[DOF])
{
    uint8_t i, j;
    float sum;

    calcInverseJacobian(invJacobian, q);

    for(i=0 ; i<DOF; i++)
    {
        sum = 0;
        for(j=0 ; j<DOF; j++)
        {
            sum += invJacobian[i][j]*v[j];
        }

        /* Speed limit */
        if(fabs(sum) > max_speed[i])
            sum = max_speed[i] * (fabs(sum)/sum); /* Maintain sign */
        
        qdot[i] = sum;
    }
}

float read_joint_pot(uint8_t port, float qlim[2])
{
    return analogRead(port)*( (qlim[1]-qlim[0])/1023 ) + qlim[0];
}

void change_axis(void)
{
    if( millis() - last_isr < debounce_time_ms )
        return;
    
    workspace = (workspace + 1) % WORKSPACES;
    last_isr = millis();
}

#if SIMULATION

unsigned long simu_interval_time, simu_last_time;

void simulate_angles(float q[DOF], float qdot[DOF])
{
    uint8_t i;

    simu_interval_time = (millis() - simu_last_time);

    /* Joints 1 to 6 */
    for(i=1 ; i<=6 ; i++)
    {
        /* Rough estimate -> d = v*t */
        if(i==2)
            q[INDEX_COMPAT(i)] = DEG_TO_RAD*wmra.read_angle(i);
        else
            q[INDEX_COMPAT(i)] += (qdot[INDEX_COMPAT(i)] * (float)simu_interval_time)/1000.0;
    }

    simu_last_time = millis();
}

#endif

void read_angles(float q[DOF])
{
    uint8_t i;

    for(i=1 ; i<=DOF ; i++)
    {
        if(i==1 || i==2)
            q[INDEX_COMPAT(i)] = DEG_TO_RAD*wmra.read_angle(i);
    }
}

void serial_send(float q[DOF])
{
    uint8_t i;
    uint8_t *data;

    if(Serial.available())
    {
        /* Flush in-stream */
        while(Serial.read() != -1)
        {
            delay(1);
        }

        /* Send values */
        for(i=0 ; i<DOF ; i++)
        {
            /* Convert to byte for sending */
            data = (uint8_t*)&q[i];
            Serial.write(data, 4);
        }
    }
}

void wrist_to_base(float q[DOF], float v[3])
{
    static float T[3][3];
    float new_v[3] = {0.0f, 0.0f, 0.0f};

    uint8_t i;

    /** 
     * Inital calculations of most common used sines and cosines
     */
    float c1    = cos(q[0]);
    float s1    = sin(q[0]);
    float c5    = cos(q[4]);
    float s5    = sin(q[4]);
    float c234  = cos(q[1]+q[2]+q[3]);
    float s234  = sin(q[1]+q[2]+q[3]);

    float aux = s5*c234;

    T[R1][C1] = c1*aux - s1*c5;
    T[R2][C1] = s1*aux + c1*c5;
    T[R3][C1] = s5*s234;
    
    aux = c5*c234;

    T[R1][C2] = c1*aux + s1*s5;
    T[R2][C2] = s1*aux - c1*s5;
    T[R3][C2] = c5*s234;

    T[R1][C3] = c1*s234;
    T[R2][C3] = s1*s234;
    T[R3][C3] = -c234;

    for(i=0 ; i<=C3 ; i++)
    {
        new_v[i] = T[i][C1]*v[R1] + T[i][C2]*v[R2] + T[i][C3]*v[R3];
    }

    v[0] = new_v[0];
    v[1] = new_v[1];
    v[2] = new_v[2];
}

void joystick_to_speed(float q[DOF], float v[DOF])
{
    /* Read joystick and convert to speed */
    float js_xy[N_AXIS];
    
    js.read(js_xy);

    /* If rotation, convert max_value to rad/s */
    if(workspace & ROT)
    {
        js_xy[0] = js_xy[0] * (MAX_W / MAX_JS);
        js_xy[1] = js_xy[1] * (MAX_W / MAX_JS);   
    }
    else
    {
        /* m/s */
        js_xy[0] = js_xy[0] * (MAX_V / MAX_JS);
        js_xy[1] = js_xy[1] * (MAX_V / MAX_JS);        
    }
    
    switch(workspace)
    {
        case XY:
            v[0] = js_xy[0];
            v[1] = js_xy[1];
            v[2] = 0.0f;
            v[3] = 0.0f;
            v[4] = 0.0f;
            v[5] = 0.0f;
            break;

        case XZ:
            v[0] = js_xy[0];
            v[1] = 0.0f;
            v[2] = js_xy[1];
            v[3] = 0.0f;
            v[4] = 0.0f;
            v[5] = 0.0f;
            break;

        case ROT_XZ:
            v[0] = 0.0f;
            v[1] = 0.0f;
            v[2] = 0.0f;
            v[3] = 0.0f;
            v[4] = js_xy[0];
            v[5] = js_xy[1];        
            break;

        case ROT_YZ:
            v[0] = 0.0f;
            v[1] = 0.0f;
            v[2] = 0.0f;
            v[3] = js_xy[1];
            v[4] = js_xy[0];
            v[5] = 0.0f;        
            break;

        default:
            break;
    }

    /* If rotation, treat as specification of wrist speed */
    if(workspace & ROT)
        wrist_to_base(q, v+3); /* 3 -> Displacement to start on angular speeds */ 
}

void set_speed(float qdot[DOF])
{
    uint8_t i = 1;
    
    /* Update reference for all*/
    for(i=1 ; i<=DOF ; i++)
    {   
        if( (qdot[INDEX_COMPAT(i)] >= 1e-3 && !uswitches[(INDEX_COMPAT(i)<<1)+END_POS]  ) || /* Moving forward and not achieved end yet */
            (qdot[INDEX_COMPAT(i)] < -1e-3 && !uswitches[(INDEX_COMPAT(i)<<1)+START_POS]) || /* Moving backwards and not achieved start */
            (fabs(qdot[INDEX_COMPAT(i)]) < 1e-3) )      /* Always set speed to shutdown */
            wmra.set_speed(i, qdot[INDEX_COMPAT(i)]);
    }
}

/*************************
 * Interrupts for motors *
 *************************/

/* Stepper 1 */
ISR(TIMER5_COMPC_vect)
{
    stepper1->do_step();
}

/* DC Motors */
ISR(TIMER3_COMPC_vect)
{
    dc1->control_update();
    
    /* Set new counter to 20ms forward */
    OCR3C = TCNT3 + 1250; /* 0.020*(16Mhz/256 prescaler) */
}

/****************************
 * Interrupts for uSwitches *
 ****************************/

volatile uint8_t int_pin;
volatile uint8_t int_idx;
volatile uint8_t int_pos;
volatile uint8_t int_mode;

void check_uswitch_common()
{
    uint8_t value;
    uint8_t i = 0, idx;

    if(int_mode == INT_NORMAL)
    {
        value = (uint8_t)digitalRead(int_pin);
    
        /* (joint_idx<<1 + int_pos) transform joint switch to flag idx */
        idx = (INDEX_COMPAT(int_idx)<<1)+int_pos;
    
        if(value == HIGH)
            uswitches[idx] = false;
        else
        {
            wmra.set_speed(int_idx, 0.0f);
            uswitches[idx] = true;
        }
    }
    else
    {
        /* Get reading of whole port */
        value = *portInputRegister(digitalPinToPort(int_pin));

        /* Check 6 positions sequentially, if LOW, stop and set flag, if HIGH, reset flag */
        for(i=0 ; i<N_PC ; i++)
        {
            idx = PC_TO_IDX[i];
            
            if(value & _BV(i))
                uswitches[idx] = false;
            else
            {
                wmra.set_speed((idx>>1) + 1, 0.0f);
                uswitches[idx] = true;
            }
        }
    }
}

void joint2_sw_s()
{
    int_pin = J2_S;
    int_idx = 2;
    int_pos = START_POS;
    int_mode = INT_NORMAL;
    
    check_uswitch_common();   
}

void joint2_sw_e()
{
    int_pin = J2_E;
    int_idx = 2;
    int_pos = END_POS;
    int_mode = INT_NORMAL;
    
    check_uswitch_common();
}

void joint3_sw_s()
{
    int_pin = J3_S;
    int_idx = 3;
    int_pos = START_POS;
    int_mode = INT_NORMAL;
    
    check_uswitch_common();   
}

void joint3_sw_e()
{
    int_pin = J3_E;
    int_idx = 3;
    int_pos = END_POS;
    int_mode = INT_NORMAL;
    
    check_uswitch_common();
}

ISR(PCINT0_vect)
{
    int_pin = J1_S; /* Any pin on the port will suffice */
    int_mode = INT_PC;

    check_uswitch_common();
}

/**************
 * Main Setup *
 **************/

/* Testing */
long loop_time;

void setup() 
{
    /* Disable interrupts */
    cli();

    Serial.begin(115200);

    /******************************
     * Micro Switches preparation *
     ******************************/
    
    pinMode(J2_S, INPUT_PULLUP); /* PULLUP for testing */
    attachInterrupt(digitalPinToInterrupt(J2_S), joint2_sw_s, CHANGE);

    pinMode(J2_E, INPUT_PULLUP); /* PULLUP for testing */
    attachInterrupt(digitalPinToInterrupt(J2_E), joint2_sw_e, CHANGE);

    pinMode(J3_S, INPUT_PULLUP); /* PULLUP for testing */
    attachInterrupt(digitalPinToInterrupt(J3_S), joint3_sw_s, CHANGE);

    pinMode(J3_E, INPUT_PULLUP); /* PULLUP for testing */
    attachInterrupt(digitalPinToInterrupt(J3_E), joint3_sw_e, CHANGE);

    /** Pin Change Interrupts **/
    /* Set PB0 to PB6 as input */
    DDRB = DDRB & B10000000;

    /* Disable Pull-Up on Port B from 0 to 6 */
    PORTB = PORTB & B10000000;
    PORTB = PORTB | B01111101; /* Testing purposes, enable pullup on unused */

    /* Set interrupt mask */
    PCMSK0 = PCMSK0 | B01111111;
    bitClear(PCMSK0, PCINT7); /* Make sure PCINT7 is disabled */

    /* Activate interrupt PCIE0 */
    bitSet(PCICR, PCIE0);

    /**********
     * Motors *
     **********/

    /* Set timer frequency for DC, 256 prescaler */
    bitSet(TCCR3B, CS12);
    bitClear(TCCR3B, CS11);
    bitClear(TCCR3B, CS10);

    /* Normal mode on timer */
    bitClear(TCCR3B, WGM33);
    bitClear(TCCR3B, WGM32);
    bitClear(TCCR3A, WGM31);
    bitClear(TCCR3A, WGM30);

    /* Disconnect output */
    bitClear(TCCR3A, COM3C0);
    bitClear(TCCR3A, COM3C1);

    /* Start timer on 0 */
    OCR3C = 0x0;

    /* Set interrupt */
    bitSet(TIMSK3, OCIE3C);
    
    /* Set timer frequency for steppers, 256 prescaler */
    bitSet(TCCR5B, CS12);
    bitClear(TCCR5B, CS11);
    bitClear(TCCR5B, CS10);

    dc1 = new DCMotor(9, 8);
    dc1->set_limits(-45, 135);
    dc1->set_angle_callbacks(A0, NULL, read_joint_pot);
    
    stepper1 = new StepperMotor(44, 40, 1, 1.8*DEG_TO_RAD);

    /* Steppers calibration */
    #if !SIMULATION
    stepper1->start(0.2, &uswitches[1], 90.0); /* Go with speed 0.2rad/s until hit uswitches[1](J1_E) and then assume 90.0 degrees */
    #endif
    
    wmra.set_link(1,     0,     0,     0);
    wmra.set_link(2,  PI/2,    a1,     0, dc1);
    wmra.set_link(3,     0,    a2,     0);
    wmra.set_link(4,     0,    a3,     0);
    wmra.set_link(5,  PI/2,    a4,     0);
    wmra.set_link(6, -PI/2,     0,    d6);

    /************
     * Joystick *
     ************/

    js.set_ISR(change_axis);
    js.set_limits(0, MAX_JS);   /* Between 0 and 100, like percentage */
    js.calibrate();

    /* Enable interrupts */
    sei();
}

/*************
 * Main Loop *
 *************/

void loop() 
{
    static float invJacobian[DOF][DOF];
    static float q[DOF] = {1.571f, 0.0f, 0.785f, 0.0f, -1.571, 0.0f};
    static float qdot[DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static float v[DOF] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    //loop_time = millis();

    #if SIMULATION
    simulate_angles(q, qdot);
    #else
    read_angles(q);
    #endif
    
    serial_send(q);

    joystick_to_speed(q, v);

    calculate_speed(invJacobian, q, qdot, v);

    cli();
    set_speed(qdot);
    sei();

    //Serial.println(millis() - loop_time);
}
