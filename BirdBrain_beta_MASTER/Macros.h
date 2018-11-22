                                              /** Supporting file for BirdBrain_beta_xx */

/***************************************************************************************************
*  A brief description of the implemented control and guidance modes:
*  --------------------------------------------------------------------
*  REMOTE_CTRL  = Manual control mode via the mux - No automatic stabilisation or guidance attempted.
*  ROBOTIC_CTRL = Autonomous control mode - Maintains stable flight in either of two navigation modes:
*   GLIDE_NVGN  = Simple robotic gliding flight guided by the pre-set default heading.
*   HELIX_NVGN  = Robotic soaring flight - Autonomous circling in encountered areas of rising air.
****************************************************************************************************/

#define REMOTE_CTRL 1
#define ROBOTIC_CTRL 2
#define HELIX_NVGN 0
#define GLIDE_NVGN 1

#define OUTPUT_DATA_INTERVAL 20
#define STARTBYTE 0xE0

// Pin numbering
#define JMPR_SINK_PIN  A0
#define JMPR_DETECT_PIN  A1     // Pin for preflight sensing of configuration mode
#define BUTTON_PIN  A2          // Trigger for inputting desired heading
#define OKAY_LED_PIN  12        // Confirmation signal LED
#define STATUS_LED_PIN 13       // Pin number of status LED

// Radio channels (Note channels are from 0!)
#define CH_ROLL 0       // Better not change these definitions. If you use a non-standard RC channel setup, 
#define CH_PITCH 1      // ... simply rearrange the relevant receiver-to-mux board leads.

// PID enumeration
#define CASE_SERVO_ROLL 0
#define CASE_SERVO_PITCH 1
#define CASE_NAV_ROLL 2

// Sensor calibration scale and offset values (AHRS)
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope (ITG-3200) (AHRS)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculates the scaled gyro readings in radians/sec.

// DCM PID parameters (AHRS)
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int) 0x69) // b1101000/0x68 if pin AD0 is logic low; b1101001/0x69 if pulled high

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

// Constant params (AHRS)
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
