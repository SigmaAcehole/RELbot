#include "MyApp.h"


MyApp::MyApp() 
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
}

MyApp::~MyApp()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}
// Initialize variables and constants
int count_max = 16384;  // Encoder range (2^14)
int rotation_max = 63815;
double pi = 3.14159265359;
double rad_per_count = 2*pi/rotation_max;  // radians per count  
int left_encoder_prev, right_encoder_prev = 0; // Encoder state variable
int left_encoder, right_encoder = 0; // Encoder pulse count
float left_angle, right_angle = 0;  // Encoder angle in radian
int left_counter, right_counter = 0;    // Encoder rollover count

/** Direction of rotation notation
 * Left encoder -> 0 = static, 1 = clock-wise, -1 = anti-clockwise
 * Right encoder -> Same notation but need to manually inverse the calculated angle
*/
int left_dir, right_dir = 0; 


float count_to_radian(int encoder, int encoder_prev, int dir, char motor){
    // Check for rollover condition
    float angle;
    if(motor == 'l'){
        if(dir == 1 && (encoder - encoder_prev) < -15000){
            left_counter++;
        }
        else if(dir == -1 && (encoder - encoder_prev) > 15000){
            left_counter--;
        }
    long int count_effective = encoder + left_counter * count_max;
    angle = count_effective * rad_per_count;
    }
    else {
        if(dir == 1 && (encoder - encoder_prev) < -15000){
            right_counter++;
        }
        else if(dir == -1 && (encoder - encoder_prev) > 15000){
            right_counter--;
        }
    long int count_effective = encoder + right_counter * count_max;
    angle = count_effective * rad_per_count;
    }
    return angle;

}


int MyApp::preProc()
{
    // Input from ROS2
    // Left motor setpoint
    u[2] = RosData.x;
    // Right motor setpoint
    u[3] = RosData.y;
    // Input from FPGA
    // Read encoder count
    left_encoder = FpgaInput.channel2;
    right_encoder = FpgaInput.channel1; 

    // Convert to radians
    /**
     * Step 1: Find the direction of rotation i.e., check if count is incrementing or decrementing or static.
     * Step 2: If there is a rollover in positive direction, increment the counter. If there is rollover in negative direction, decrement the counter.
     * Step 3: Convert encoder count to radians.
    */
    // Left encoder
    if((left_encoder - left_encoder_prev) >= 1 && (left_encoder - left_encoder_prev) < 1000){
        left_dir = 1;   // Clockwise rotation
    }
    if((left_encoder - left_encoder_prev) <= -1 && (left_encoder - left_encoder_prev) > -1000){
        left_dir = -1;  // Anti-clockwise
    }
    if(left_encoder == left_encoder_prev){
        left_dir = 0;   // Static
    }

    switch(left_dir){
        case 0:  // Static
        if((left_encoder - left_encoder_prev) > 15000){
            left_counter--;
        }
        if((left_encoder - left_encoder_prev) < -15000){
            left_counter++;
        }
        left_angle = (left_encoder + left_counter * count_max) * rad_per_count;
        break;

        case 1:  // Clockwise rotation
        if((left_encoder - left_encoder_prev) < -15000){
            left_counter++;
        }
        left_angle = (left_encoder + left_counter * count_max) * rad_per_count;
        break;

        case -1:  // Anti-clockwise rotation
        if((left_encoder - left_encoder_prev) > 15000){
            left_counter--;
        }
        left_angle = (left_encoder + left_counter * count_max) * rad_per_count;
        break;
    }

    // Right encoder
    if((right_encoder - right_encoder_prev) >= 1 && (right_encoder - right_encoder_prev) < 1000){
        right_dir = 1;   // Clockwise rotation
    }
    if((right_encoder - right_encoder_prev) <= -1 && (right_encoder - right_encoder_prev) > -1000){
        right_dir = -1;  // Anti-clockwise
    }
    if(right_encoder == right_encoder_prev){
        right_dir = 0;   // Static
    }

    switch(right_dir){
        case 0:  // Static
        if((right_encoder - right_encoder_prev) > 15000){
            right_counter--;
        }
        if((right_encoder - right_encoder_prev) < -15000){
            right_counter++;
        }
        right_angle = (right_encoder + right_counter * count_max) * rad_per_count;
        break;

        case 1:  // Clockwise rotation
        if((right_encoder - right_encoder_prev) < -15000){
            right_counter++;
        }
        right_angle = (right_encoder + right_counter * count_max) * rad_per_count;
        break;

        case -1:  // Anti-clockwise rotation
        if((right_encoder - right_encoder_prev) > 15000){
            right_counter--;
        }
        right_angle = (right_encoder + right_counter * count_max) * rad_per_count;
        break;
    }

    // Update state
    left_encoder_prev = left_encoder;
    right_encoder_prev = right_encoder;

    // Xenomai to ROS
    // Left motor encoder
    u[0] = left_angle;
    // Right motor encoder
    u[1] = right_angle;

    // Send to ROS2
    XenoData.x = left_angle;
    XenoData.y = right_angle;

    evl_printf("Setpoint left:%f right:%f\n", RosData.x, RosData.y);
    evl_printf("Encoder left:%f right:%f\n", left_angle, right_angle);
    return 0;
}



int MyApp::postProc()
{  
    /*Mapping from percentage to PWM may be needed (-100/100 to -2047/2047)*/
    // Left motor output (port 2)
    FpgaOutput.pwm2 = std::clamp(y[0], -100.0, 100.0);   // PWM signal
    //FpgaOutput.pwm2 = y[0];
    FpgaOutput.val2 = 1;      // Direction
    // Right motor output (port 1)
    FpgaOutput.pwm1 = std::clamp(y[1], -100.0, 100.0);   // PWM signal
    //FpgaOutput.pwm2 = y[1];
    FpgaOutput.val1 = 1;      // Direction
    
    evl_printf("xeno_thread pwm_left:%lf pwm_right:%lf\n", y[0], y[1]);


    // evl_printf("xeno_thread pwm_left:%d pwm_right:%d\n", FpgaOutput.pwm2, FpgaOutput.pwm1);
    return 0;
}

       

