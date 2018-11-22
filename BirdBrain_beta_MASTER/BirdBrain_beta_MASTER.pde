//                                                                                BirdBrain_beta_xx
/************************************************************************************************************
 *  BirdBrain -- Basic stabilisation and navigation firmware for a soaring-flight capable UAV.
 *
 *  Based on code from the Ardupilot project (https://code.google.com/p/ardupilot/downloads/list).
 *  Uses a 3x3 degrees of measurement Attitude and Heading Reference System based on the Razor AHRS
 *  (please see header of the AHRS.pde file for details).
 *
 *  Soaring/gliding flight related code by Martin Bergman <bergman.martin at gmail dot com> 2013.
 *  License: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/legalcode
 *
 *  Uses input from an external variometer like the the one described here:
 *                           https://sites.google.com/site/xnebmisc/home/vario-project-3
 *
 *  The following specific hardware is furthermore assumed:
 *      Microcontoller :  ATMega328 @ 3.3V, 8MHz
 *      Multiplexer    :  3DRobotics FailSafe Mux, cp schematic "BirdBrain_Mux__rev05.pdf"
 *      IMU            :  Sensor array as per schematic "BirdBrain_μC+IMU__rev03.pdf"
 *
 *      R/C system with "mode 2" Tx and sequential PWM signals.
 *
 ***********************************************************************************************************/


#include <avr/io.h>
#include <math.h>
#include <Wire.h>
#include "Macros.h"
#include "Config.h"

#define USE_DIP_CONFIG  true     // Set to true to enable DIP switch configuration. See System.pde file

///#########################  GLOBAL VARIABLE DECLARATIONS  #################################

/*  Radio channel assignments
    0 : Ch1 - Ailerons (if used, otherwise Rudder)
    1 : Ch2 - Elevator                                                                                  */

int16_t  radio_min[]     = {CH1_MIN, CH2_MIN};  // Preset in Config.h.
int16_t  radio_trim[]    = {CH1_TRIM,CH2_TRIM}; // Preset in Config.h.
int16_t  radio_max[]     = {CH1_MAX, CH2_MAX};  // Preset in Config.h.

int16_t  radio_in[]      = {1500,1500};         // Current PWM values from the transmitter (microseconds).
int16_t  last_radio_in[] = {1500,1500};         // Saved values for smoother transition from manual mode.
int16_t  radio_out[]     = {1500,1500};         // PWM servo values for ROLL (RUDD) and PITCH (ELEV)
float    servo_out[]     = {0,0};               // Current commands to the servos (-45 to +45 degrees).
uint16_t  ch1_temp       = 1500;
uint16_t  ch2_temp       = 1500;

uint16_t  ch1_tempLast   = 1500;
uint16_t  ch2_tempLast   = 1500;


/*  PID control cases (k-array items):
    0 : Rudder (or ailerons) servo control
    1 : Elevator servo control
    2 : Roll set-point control                                                                    */

float  kp[]={SERVO_ROLL_P, SERVO_PITCH_P, NAV_ROLL_P};      // cp Config.h
float  ki[]={SERVO_ROLL_I, SERVO_PITCH_I, NAV_ROLL_I};      // cp Config.h
float  kd[]={SERVO_ROLL_D, SERVO_PITCH_D, NAV_ROLL_D};      // cp Config.h

const float  integrator_max[] = {SERVO_ROLL_INT_MAX, SERVO_PITCH_INT_MAX,  NAV_ROLL_INT_MAX};

float  integrator[] = {0,0,0};     // PID Integrators.
float  prev_error[] = {0,0,0};     // PID latest error for derivative.
//float  prev_input[] = {0,0,0};     // P_I_D latest input for derivative.          // Varför float??

// IMU sensor outputs as processed by the AHRS algorithms
// ------------------------------------------------------
int32_t  ref_roll   = 0;   // deg * 100 : Angle in the Y/Z plane (How much we're banking).
int32_t  ref_pitch  = 0;   // deg * 100 : Angle in the X/Z plane, i.e. the longitudinal axis' inclination,
                           //        ... relative the horizon. (This is not the aerodynamic AoA, per se).
// Navigation variables
// --------------------
int8_t   DIP_selector       = 0;   // DIP-switch array for setting optional config choices in the field.
int8_t   DIP_configuration  = 0;   // Identifies one of eight predefined sets of configuration values.
uint8_t  DIP_value =0;  ///>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<
int8_t   vertical_velocity  = 0;   // decimeter/sec : Value received from external variometer circuit.
int32_t  now_heading        = 0;   // deg * 100 : +/-18000 Current actual heading (a.k.a. apparent course).
int16_t  run_win[RW_LENGTH]    ;   // "Running window" i.e. a simple FIR Moving Average.
int32_t  targ_heading       = 0;   // deg * 100 : +/-18000 Current desired setpoint on the compass.
int32_t  heading_default    = 0;   // deg * 100 : +/-18000 The pre-set heading in robotic coasting mode.
float    hd_accumulator     = 0;   // Used for filtering the heading-default value
int32_t  course_error       = 0;   // deg * 100 : 18000 to -18000.
int16_t  progressor = TURN_RATE;   // deg * 10 : Offset that advances targ_heading in helixing mode.
                                         /// Egentligen inte en define utan variabel i kommande versioner
// Stabilisation variables
// -----------------------
int32_t  nav_roll           = 0;   // deg * 100 : Target roll angle.
int32_t  nav_pitch          = 0;   // deg * 100 : Target pitch angle.

// Modal variables
// ---------------
uint8_t  control_mode       = REMOTE_CTRL;      // 1 - Set by RC TX switch.
uint8_t  guidance_mode      = GLIDE_NVGN;       // 1 - Set in operation by vario input.

// Timers, Counters and Flags
// --------------------------
uint32_t  timestamp         = 0;   // Time (ms) when the last iteration of main control loop ended.
uint32_t  medium_loopTimer  = 0;   // Duration in milliseconds of the medium speed navigation control loop.
uint8_t  medium_loopCounter = 0;   // Counter for branching from main loop to medium frequency loop.
uint8_t   slow_loopCounter  = 0;   // Counter for branching from medium loop to the slow loop.
uint32_t  deltaMilliseconds = 0;   // Delta Time in milliseconds.
uint32_t  dTnav             = 0;   // Delta Time in milliseconds for navigation computations.
uint8_t   hd_read_counter   = 0;   // Used for filtering the heading_default value.
boolean   hd_is_set     = false;   // Flag.
uint8_t   switch_position   = 0;   // Present state of the bipolar switch paddle.
uint8_t   old_switch_pos    = 0;   // Previous state of ditto.

///############################  AHRS VARIABLE DECLARATIONS  #################################

// IMU variables
// -------------
float  accel[3];          // Stores the NEGATED acceleration, which equals gravity if IMU is stationary.
float  accel_min[3];
float  accel_max[3];

float  magn[3];
float  magnetom_min[3];
float  magnetom_max[3];
float  magnetom_tmp[3];

float  gyro[3];
float  gyro_average[3];
int16_t  gyro_num_samples = 0;

// DCM (Derivative Cosine Matrix) variables
// ----------------------------------------
float  Mag_Heading;
float  Accel_Vector[3]          = {0, 0, 0};        // Store the acceleration in a vector.
float  Gyro_Vector[3]           = {0, 0, 0};        // Store the gyros turn rate in a vector.
float  Omega_Vector[3]          = {0, 0, 0};        // Corrected Gyro_Vector data.
float  Omega_P[3]               = {0, 0, 0};        // Omega Proportional correction.
float  Omega_I[3]               = {0, 0, 0};        // Omega Integrator.
float  Omega[3]                 = {0, 0, 0};
float  errorRollPitch[3]        = {0, 0, 0};
float  errorYaw[3]              = {0, 0, 0};
float  DCM_Matrix[3][3]         = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float  Update_Matrix[3][3]      = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float  Temporary_Matrix[3][3]   = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles (expressed as radians)
// ----------------------
float  yaw;
float  pitch;
float  roll;

// DCM timing in the main loop
// ---------------------------
uint32_t  timestamp_old;
float  G_Dt;                                        // Integration time for DCM algorithm.
extern uint32_t timer0_millis;

// Calibration variables and flags
// -------------------------------
boolean  commands_enabled_flag          = false;    // Set initially by jumper detection.
boolean  output_stream_on_flag          = false;    // Set by user input in calibration mode
boolean  output_single_on_flag          = false;    // Set by user input in calibration mode
int16_t  curr_calibration_sensor        = 0;
boolean  reset_calibration_session_flag = true;
int16_t  num_accel_errors               = 0;
int16_t  num_magn_errors                = 0;
int16_t  num_gyro_errors                = 0;


///#############################  SETUP  ######################################

void setup() 
{
    init_BirdBrain();
}

///#############################  LOOP   ######################################

void loop()       /**  We want the main loop to execute at 50Hz if possible*/
{          
    if (DIYmillis() - timestamp >= OUTPUT_DATA_INTERVAL){       // 20 milliseconds
        time_Ladies_time();                                     // "Oh, how time flies"
        fast_loop();
        medium_loop();
    }
}

///##########################  LOOP FUNCTIONS  ################################

void fast_loop()                               /// This should run at the maximum speed (50Hz/20ms)
{
    run_AHRS();                                // Retrieves our attitude
    get_vario_value();                         // Sets navigation mode with predefined variometer type
    update_control_by_mode();                  // Specific code for the pertinent control mode
}


void medium_loop()                             /// This is the semi-pedestrian 10 Hz loop (100 ms)
{
    switch (medium_loopCounter){
        case 0:
            medium_loopCounter++;
            dTnav = DIYmillis() - medium_loopTimer;	 // Update navigation delta-Timer
            medium_loopTimer = DIYmillis();
            break;

        case 1:
            medium_loopCounter++;
            calc_nav_roll();                   // Get roll angle required to reach navigation setpoint
            break;

        case 2:
            medium_loopCounter++;
            check_n_set_heading_default();     // Is the heading-setter button pressed just now?   
            break;

        case 3:
            medium_loopCounter++;
            set_course(guidance_mode);         // Calculate the plane's desired apparent course
            break;

        case 4:	
            medium_loopCounter=0;
            slow_loop();                       // High time for a brief visit to the lazy lane
            break;
    }
}


void slow_loop()                               /// This is the slooow 3-1/3 Hz loop  (@300 ms)
{
    switch (slow_loopCounter){
        case 0:
            slow_loopCounter++;
            if(output_stream_on_flag == false){   // Only blink if pin 13 is not already engaged
                digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
            }
            break;

        case 1:
            slow_loopCounter++;
            check_control_switch_pos(); // Poll the radio's switch channel for changes to control_mode
            break;

        case 2:
            slow_loopCounter = 0;
#if DEBUG__PRINT_CONTROL_VARS
            print_control_variables();
#endif
            break;
    }
}


///##########################  MODE LOGIC FUNCTIONS  ###########################

void get_vario_value(void)                    // This executes in the fast loop and sets guidance_mode. 
#if VARIO_VERSION == 3                        // Vario type XNEB v3 is updated @ 11.9 Hz, which means ...
{                                             // A four-byte frame arrives each 4.2 loops, so needs 5 loops!
    if(Serial.available() > 3 ){              // We should have received at least four bytes now.
        uint8_t inByte = Serial.read();       // Read a byte from the serial port,
        if(inByte == STARTBYTE){              // ... and if it's indeed the expected startbyte:
            vertical_velocity = Serial.read();// Save the next arrival, which is the vario data byte.
            Serial.read(); Serial.read();     // Dispose of the next two (superfluous) bytes.
        }
        if (vertical_velocity > 127){         // Serial.write casts to bytes, so sub-zero values wrap to <255.
            vertical_velocity -= 255;         // This should properly restore negative values.
        }
    }
    if(vertical_velocity > PUKKA_LIFT){       // Now examine it and select the best navigation mode.
        guidance_mode = HELIX_NVGN;
    }
    else {
        guidance_mode = GLIDE_NVGN;
    }
}
#else if VARIO_VERSION == 4
{
        // ny funktion för "MEAS"
}
#endif


void update_control_by_mode(void)               // This gets called in the fast loop.
{
    switch(control_mode){
        case ROBOTIC_CTRL:
            steady_now_steady();                // Stabilise flight
            set_servos_2();                     // Write out the servo PWM values
            break;

        case REMOTE_CTRL:
            read_RCRX();                        // (avläs PWM för att vara uppdaterad om servoställningar)
            break;
    }
}


void check_n_set_heading_default(void)           // This function is called in the medium speed loop.
{
    if(digitalRead(BUTTON_PIN) == LOW){          // Button is pressed! 
        if(hd_is_set == false){                  // We haven't started sampling yet.
            delay(150);                          // Idle awhile.
            if(digitalRead(BUTTON_PIN) == LOW){  // Now we must presume it's a legitimate trigging.
                hd_is_set = true;                // In which case we may proceed to the actual sampling.
                heading_default = hd_accumulator = 0;// First reset the present default HDG value.
            }
            else {                               // It would appear the button was realeased again...
                hd_is_set = false;
                return;                          // Abort. Mea culpa.
            }                          
        }
        if((hd_is_set == true) && (digitalRead(BUTTON_PIN) == LOW)){
            if(hd_read_counter < 10){
            hd_accumulator += 10 * TO_DEG(yaw);  // First collect ten "amplified" magnetometer samples
            hd_read_counter++;                   // (This, obviously, needs 10 x 100ms = 1 second)
            }                   /// THIS DOES NOT WORK IF POINTING IN +/-180 WRAP-AROUND DIRECTION!
            else {
                hd_accumulator += 50;            // Add 50 to compensate for ten lost random fractions
                heading_default = hd_accumulator;// Then save the average sample expressed as deg * 100
                digitalWrite(OKAY_LED_PIN, HIGH);// Turn the LED on
                // demo_servos();                // Conspicuously confirm that a heading has been saved
                while(digitalRead(BUTTON_PIN) == LOW){} // Just blocking until released
                delay(50);                       // This should minimize risk of a trigger bounce
                hd_read_counter = 0;             // Reset counter
                hd_is_set = false;               // So we can repeat when the button is pushed next
                digitalWrite(OKAY_LED_PIN, LOW); // Turn the LED off; We're done for now.
                #if DEBUG__PRINT_CONTROL_VARS
                Serial.print("\t==========>  Saved hd: ");
                Serial.println(heading_default, DEC);
                #endif
            }
        }
    }
    else {                                        // Nothing has happened ...
    } return;
}
