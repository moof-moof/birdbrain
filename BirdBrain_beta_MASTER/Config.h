                                              /** Supporting file for BirdBrain_beta_xx */
/// OUTPUT OPTIONS (AHRS)
/*****************************************************************/

// Sensor data output interval in milliseconds. This may not work, if faster than 20ms (=50Hz).
// Code parts for both AHRS and navigation/stabilisation is tuned for 20ms, so better leave it like that.
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Set your serial port baud rates used to send (and/or receive) data here!
#define OUTPUT__BAUD_RATE 57600
#define VARIO__BAUD_RATE 9600

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes

// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;  // Text or binary?         (Only used in calibration mode)

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false 

// If set true, an error message can be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
bool output_errors = false;  // true or false


/// SENSOR CALIBRATION (AHRS)
/*****************************************************************/ /// (Per calibration 13-04-05)

// Put MIN/MAX and OFFSET readings for your board here!
// Cp. the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
/// (The placeholder values below are just examples that apply to a particular set of sensors).

// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -276)
#define ACCEL_X_MAX ((float)  262)
#define ACCEL_Y_MIN ((float) -242)
#define ACCEL_Y_MAX ((float)  296)
#define ACCEL_Z_MIN ((float) -269)
#define ACCEL_Z_MAX ((float)  269)

// Magnetometer (standard calibration)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
// #define MAGN_X_MIN ((float) -685)
// #define MAGN_X_MAX ((float)  464)
// #define MAGN_Y_MIN ((float) -520)
// #define MAGN_Y_MAX ((float)  644)
// #define MAGN_Z_MIN ((float) -761)
// #define MAGN_Z_MAX ((float)  277)

// Magnetometer (extended calibration)
/** Comment: The Processing Magnetometer_calibration program kept crashing after a short while.
Typical errors were:
  -- Exception in thread "Animation Thread" java.lang.RuntimeException: Eigenvalue Decomposition failed
  -- Exception in thread "Thread-1" java.lang.NullPointerException) 
Furthermore, the dmesg command returned reports of frequent overrun(s) in the ttyUSB stream.
Testing under Windows 7, however, the ellipsoid values below were successfully obtained!      (mb 13-04-05) */

#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {-112.849, 38.0300, -17.8251};
const float magn_ellipsoid_transform[3][3] = {{0.846987, 0.0129140, 0.00481790}, {0.0129140, 0.851844, 0.0141091}, {0.00481790, 0.0141091, 0.998416}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) -0.02)
#define GYRO_AVERAGE_OFFSET_Y ((float) 36.97)
#define GYRO_AVERAGE_OFFSET_Z ((float) -14.57)


/// DEBUG OPTIONS (AHRS)
/**************************************/

// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false
// Print the AHRS output, navigation and stabilisation variables at 3.33 Hz
#define DEBUG__PRINT_CONTROL_VARS false


/// RADIO AND PWM PARAMS
/***************************************/

#define POSITION_1  REMOTE_CTRL     // This is the manual mode: Radio controlled input via the mux
#define POSITION_2  ROBOTIC_CTRL    // "Look Ma, no hands!"

#define AUTO_TRIM 0         // 0 = no, 1 = set the trim of the radio when switching from Manual.
                            // This only works if the Tx sticks are released before switching over.
//#define SET_RADIO_LIMITS 0  // 0 = no, 1 = set the limits of the Channels with the radio at launch 
#define RADIO_TYPE 0        // 0 = sequential PWM pulses (e.g. Spektrum), 1 = simultaneous PWM pulses
#define CH1_TRIM 1485       // (Microseconds) Trim of Ailerons/ Rudder
#define CH1_MIN 1155        // (Microseconds) Range of Ailerons/ Rudder
#define CH1_MAX 1805        // (Microseconds) Range of Ailerons/ Rudder
#define CH2_TRIM 1420       // (Microseconds) Trim of Elevator
#define CH2_MIN 1070        // (Microseconds) Range of Elevator
#define CH2_MAX 1910        // (1955 if combined with full rudder deflection either way)
#define REVERSE_ROLL 1      // Put -1 to reverse roll, 1 otherwise
#define REVERSE_PITCH 1     // Put -1 to reverse pitch, 1 otherwise


/// GUIDANCE PARAMETERS
/***************************************/

#define VARIO_VERSION 3     // "3" is XNEB v3 with BMP085 baro, "4" is XNEB v4 with MS5611 baro.
#define PUKKA_LIFT 2        // dm/sec : critical vertical velocity value used for selecting nav modes
#define TURN_RATE 36        // deg/sec (25- 50): Step size for advancing setpoint heading in turns
#define RW_LENGTH 5         // Number of samples in "running window" for moving average/FIR filter.

//#define ROLL_TRIM 0         // deg * 100 : allows us to offset bad sensor placement
#define BANK_MAX 4000       // deg * 100 : The maximum commanded bank angle (left and right)

#define PITCH_DATUM 0       // deg * 100 : Angle when wing root chord is horizontal - use IMU to find this value.
//#define PITCH_TRIM 0        // deg * 100 : allows us to offset bad sensor placement
#define PITCH_MAX 1500      // deg * 100 : The maximum commanded pitch up angle 
#define PITCH_MIN -2000     // deg * 100 : The maximum commanded pitch down angle
#define ALPHA_LD 50     // deg * 100 : Pitch setpoint relative the level reference angle (PITCH_DATUM)
#define ALPHA_LIFT 150  // deg * 100 : Pitch setpoint relative the level reference angle (PITCH_DATUM)


/// GUIDANCE & STABILISATION PID GAINS
/******************************************/

#if USE_DIP_CONFIG
    #define SERVO_ROLL_P          var_SERVO_ROLL_P
    #define SERVO_ROLL_I          var_SERVO_ROLL_I
    #define SERVO_ROLL_D          var_SERVO_ROLL_D
    #define SERVO_ROLL_INT_MAX    var_SERVO_ROLL_INT_MAX
    #define ROLL_SLEW_LIMIT       var_ROLL_SLEW_LIMIT
    #define SERVO_PITCH_P         var_SERVO_PITCH_P
    #define SERVO_PITCH_I         var_SERVO_PITCH_I
    #define SERVO_PITCH_D         var_SERVO_PITCH_D
    #define SERVO_PITCH_INT_MAX   var_SERVO_PITCH_INT_MAX 
    #define PITCH_COMP            var_PITCH_COMP
    #define NAV_ROLL_P            var_NAV_ROLL_P
    #define NAV_ROLL_I            var_NAV_ROLL_I
    #define NAV_ROLL_D            var_NAV_ROLL_D
    #define NAV_ROLL_INT_MAX      var_NAV_ROLL_INT_MAX
#else
    //ATTITUDE: ROLL GAINS   [Start with changes of no more than 25% at a time]
    //-------------------------------------------------------------------------
    #define SERVO_ROLL_P .004           // Primary value to tune - overall proportional term determines
                                        // ... how much rudder/aileron you use to turn
    #define SERVO_ROLL_I .00            // roll PID integrator gain (value should generally be low)
    #define SERVO_ROLL_D .00            // roll PID derivative gain (for advanced users - 
                                        // ... should be zero for most airframes)
    #define SERVO_ROLL_INT_MAX 500      // Maximium integrator value in degrees * 100
    #define ROLL_SLEW_LIMIT 0           // Use to limit slew rate of roll servo. 0 = slew rate is not limited
                                        // Value is degree per second limit

    //ATTITUDE: PITCH GAINS   [Start with changes of no more than 25% at a time]
    //--------------------------------------------------------------------------
    #define SERVO_PITCH_P .005          // Pitch Proportional gain
    #define SERVO_PITCH_I .0            // Pitch integrator gain (value should generally be low)
    #define SERVO_PITCH_D .0            // Pitch derivative gain (zero for most airframes)
    #define SERVO_PITCH_INT_MAX 500     // Maximum integrator value in degrees * 100
    #define PITCH_COMP .20              // Pitch compensation vs. Roll bank angle. 

    //NAV: ROLL GAINS    [Start with changes of no more than 25% at a time]
    //---------------------------------------------------------------------
    #define NAV_ROLL_P .7               // Primary value to tune - overall proportional term determines how
                                        // ... aggressively we bank to change heading
    #define NAV_ROLL_I .01              // roll PID integrator gain (value should generally be low)
    #define NAV_ROLL_D .02              // roll PID derivative gain (for advanced users -
                                        // ... should be zero for most airframes)
    #define NAV_ROLL_INT_MAX 500        // Maximium integrator value in degrees * 100
#endif
