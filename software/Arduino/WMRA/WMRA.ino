#include "robotic_arm.h"
#include "joystick.h"

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

#define XY          (0)
#define XZ          (1)
#define ROT         (2)
#define WORKSPACES  (3)

float a2 = 0.2;
float a3 = 0.3;
float a4 = 0.4;
float a5 = 0.5;
float d3 = 1.3;
float d4 = 1.4;
float d6 = 1.5;

Joystick js(2, A0, A5);
robotic_arm wmra;

int workspace = XY;
volatile unsigned long last_isr = 0;
long debounce_time_ms = 500;

/** Calculates inverse jacobian.
 *  
 *  The values which appear too often on the calculations are left in form of variables, seeking to reduce the number of math operations
 *  needed in exchange for a little spent memory. 
 *  In total, this function uses:
 *                          - 137 Multiplications   (Divisions included)
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
    float S3_A3_A4      = sin(q[2])*a3*a4;

    float C23_A4        = cos(q[1]+q[2])*a4;         
    float C2A3_C23A4    = cos(q[1])*a3 + C23_A4;

    float C5_D6         = c5*d6;                              
    float A5_C5D6       = a5 + C5_D6;

    float A2c           = a2 + C2A3_C23A4 + c234*( A5_C5D6 );
    float A2c_c234c5d6  = A2c + c234*C5_D6;

    float temp0         = s5*d6;
    float D3s           = d3 + d4 + temp0;
    
    temp0               = D3s + temp0;
    
    float detJ          = -c5*S3_A3_A4*A2c_c234c5d6;  

    /**************************/
    /* Third row calculations */
    /**************************/

    /*S23_A4 = s23*a4; <- Original name of variable */
    float temp1 = sin(q[1]+q[2])*a4;

    invJacobian[C1][R3] = invJacobian[C5][R3] = invJacobian[C6][R3] = 0;
    invJacobian[C2][R3] = ( -c5*temp1*A2c_c234c5d6 ) / detJ;
    invJacobian[C3][R3] = ( c5*( sin(q[1])*a3 + temp1 )*A2c_c234c5d6 ) / detJ;
    invJacobian[C4][R3] = - ( invJacobian[C2][R3] + invJacobian[C3][R3] );

    /*************************/
    /* Some terms of 6th row */
    /*************************/

    /*C5C234 = c5*c234; <- Original name of variable */
    temp1 = c5*c234;
    
    invJacobian[C6][R6] = ( -S3_A3_A4*s234*A2c ) / detJ;
    invJacobian[C5][R6] = ( S3_A3_A4*temp1*A2c ) / detJ;

    /*C5D6_C5C234 = C5_D6*C5C234 <- Original name of variable */
    temp1 = C5_D6*temp1;

    invJacobian[C1][R6] = ( -S3_A3_A4*temp1 ) / detJ;

    /********************/ 
    /* 1st and 2nd rows */
    /********************/
    float S4_A4      = sin(q[3])*a4;
    float S34A3_S4A4 = sin(q[2]+q[3])*a3+S4_A4;

    float S1_S3_A3_A4 = s1*S3_A3_A4;
    float C1_S3_A3_A4 = c1*S3_A3_A4;

    float S5S234     = s5*s234;       

    float temp2 = c5*(s1*temp0 + c1*A2c_c234c5d6);
    float temp3 = S5S234*(A5_C5D6 + C5_D6)*S4_A4;
    float temp4 = S5S234*(A5_C5D6 + C5_D6)*S34A3_S4A4;

    invJacobian[C1][R1] = ( S1_S3_A3_A4*c5 ) / detJ;
    invJacobian[C5][R1] = invJacobian[C1][R1]*c234;
    invJacobian[C6][R1] = ( -S1_S3_A3_A4*s234 ) / detJ;
    invJacobian[C2][R1] = ( -C23_A4*temp2 + s1*temp3 ) / detJ;
    invJacobian[C3][R1] = ( C2A3_C23A4*temp2 - s1*temp4 ) / detJ;
    invJacobian[C4][R1] = - ( invJacobian[C2][R1] + invJacobian[C3][R1] ) - invJacobian[C6][R1]*s5;
    
    temp2 = c5*(c1*temp0 - s1*A2c_c234c5d6);

    invJacobian[C1][R2] = ( -C1_S3_A3_A4*c5 ) / detJ;
    invJacobian[C5][R2] = invJacobian[C1][R2]*c234;
    invJacobian[C6][R2] = ( C1_S3_A3_A4*s234 ) / detJ;
    invJacobian[C2][R2] = ( C23_A4*temp2 - c1*temp3 ) / detJ;
    invJacobian[C3][R2] = ( -C2A3_C23A4*temp2 + c1*temp4 ) / detJ;
    invJacobian[C4][R2] = - ( invJacobian[C2][R2] + invJacobian[C3][R2] ) - invJacobian[C6][R2]*s5;

    /**************************/
    /* Final terms of 6th row */
    /**************************/
    float C4_A4      = cos(q[3])*a4;
    float C34A3_C4A4 = cos(q[2]+q[3])*a3+C4_A4;

    S5S234 = S5S234*A2c;
    
    temp3 = A5_C5D6*S4_A4;
    temp4 = A5_C5D6*S34A3_S4A4;

    /*C5D6_C5C234_D3s = C5D6_C5C234*D3s; <- Original name of variable */
    temp1 = temp1*D3s;

    /*C5D6_S5C234_A2c = C5_D6*s5*c234*A2c; <- Original name of variable */
    temp0 = C5_D6*s5*c234*A2c;

    invJacobian[C2][R6] = ( C23_A4*temp1 - temp0*C4_A4 + S5S234*temp3 ) / detJ;
    invJacobian[C3][R6] = ( -C2A3_C23A4*temp1 + temp0*C34A3_C4A4 - S5S234*temp4 ) / detJ;
    invJacobian[C4][R6] = - ( invJacobian[C2][R6] + invJacobian[C3][R6] ) - s5*invJacobian[C6][R6];

    /****************/
    /* Rows 4 and 5 */
    /****************/
    float A2cc234_c5d6 = A2c*c234 + C5_D6;

    /*C5S234 = c5*s234; <- Original name of variable */
    temp1 = c5*s234;
    /*C5D6_C5S234 = C5_D6*temp1; <- Original name of variable */
    temp0 = C5_D6*temp1;
    
    invJacobian[C6][R4] = ( -C1_S3_A3_A4*A2cc234_c5d6 ) / detJ;
    invJacobian[C6][R5] = ( -S1_S3_A3_A4*A2cc234_c5d6 ) / detJ;

    /*C5S234_A2c = C5S234*A2c; <- Original name of variable */
    temp1 = temp1*A2c;
    
    invJacobian[C5][R4] = ( -C1_S3_A3_A4*temp1 ) / detJ;
    invJacobian[C5][R5] = ( -S1_S3_A3_A4*temp1 ) / detJ;
    
    invJacobian[C1][R4] = ( C1_S3_A3_A4*temp0 ) / detJ;
    invJacobian[C1][R5] = ( S1_S3_A3_A4*temp0 ) / detJ;

    /*C5D6_S5S234 <- Original name of variable */
    S5S234 = C5_D6*S5S234;

    /*C5D6_C5S234_D3s = temp0*D3s; <- Original name of variable */
    temp0 = temp0*D3s;
    
    temp2 = c5*s1*A2c_c234c5d6 - s5*c1*A2cc234_c5d6;
    
    invJacobian[C2][R4] = ( -c1*( C23_A4*temp0 - S5S234*C4_A4 ) - temp2*temp3 ) / detJ;
    invJacobian[C3][R4] = ( c1*( C2A3_C23A4*temp0 - S5S234*C34A3_C4A4 ) + temp2*temp4 ) / detJ;
    invJacobian[C4][R4] = - ( invJacobian[C2][R4] + invJacobian[C3][R4] ) - s5*invJacobian[C6][R4] - invJacobian[C1][R1]*A2c_c234c5d6;

    temp2 = c5*c1*A2c_c234c5d6 + s1*s5*A2cc234_c5d6;

    invJacobian[C2][R5] = ( -s1*( C23_A4*temp0 - S5S234*C4_A4 ) + temp2*temp3 ) / detJ;
    invJacobian[C3][R5] = ( s1*( C2A3_C23A4*temp0 - S5S234*C34A3_C4A4 ) - temp2*temp4 ) / detJ;
    invJacobian[C4][R5] = - ( invJacobian[C2][R5] + invJacobian[C3][R5] ) - s5*invJacobian[C6][R5] - invJacobian[C1][R2]*A2c_c234c5d6;
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

void read_angles(float q[DOF])
{
    uint8_t i;

    /* Joint 1 is stepper motor */

    /* Joints 2 to 4 are normal DC motors */
    for(i=2 ; i<=4 ; i++)
    {
        q[INDEX_COMPAT(i)] = wmra.read_angle(i);
    }

    /* Joints 5 and 6 also stepper motors */
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

void setup() {
    wmra.set_link(1,     0,     0,      0);
    
    wmra.set_link(2,  PI/2, 0.014,      0);
    wmra.set_link_limits(2, -45, 135);
    wmra.set_link_callbacks(2, NULL, read_joint_pot, A3);

    wmra.set_link(3,     0,  0.35,  0.069);
    wmra.set_link(4,     0,  0.35, -0.069);
    wmra.set_link(5,  PI/2, 0.165,      0);
    wmra.set_link(6, -PI/2,     0,  0.198);

    js.set_ISR(change_axis);
    js.set_limits("xy", -15, 15);
    js.calibrate();
    
    Serial.begin(115200);
}

void loop() {
    // put your main code here, to run repeatedly:
    static float invJacobian[DOF][DOF];
    static float q[DOF] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    //read_angles(q);

    //Serial.println(wmra.read_angle(2));
    
    //serial_send(q);

    //Serial.print("x: ");
    //Serial.print(js.read('x'));
    Serial.println(analogRead(A0));
    //Serial.print(" y: ");
    //Serial.println(js.read('y'));
    //Serial.println(analogRead(A5));
    //Serial.print(" Workspace: ");
    //Serial.println(workspace);

    //calculate_speed(invJacobian, q, qdot, v);
}
