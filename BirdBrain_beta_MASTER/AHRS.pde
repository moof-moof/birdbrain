                                              /** Supporting file for BirdBrain_beta_xx */
/************************************************************************************************************
 *  3x3 Degrees of Measurement Attitude and Heading Reference System for a "3x3 DOF IMU" 
 *
 *  Based on Peter Bartz excellent Razor AHRS v1.4.1, which is copyright 2011-2012 Quality & Usability Lab,
 *  Deutsche Telekom Laboratories, TU Berlin, and released under GNU GPL (General Public License) v3.0.
 *
 *  History:
 *   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
 *     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel.
 *
 *   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik
 *     (david.zsolt.malik@gmail.com) for new Sparkfun 9DOF Razor hardware.
 *
 *   * Updated, restructured, extended and generally cleaned-up by Peter Bartz (peter-bartz@gmx.de).
 *
 *   * The present version is an abridged fork to suit the specific and rather more limited requirements
 *     of an autonomous soaring UAV application by Martin Bergman (bergman.martin at gmail dot com) 2013. 
 *     License as such: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/legalcode. 
 *
 *
 *  This application assumes the following specific hardware (see schematic file "BirdBrain_μC+IMU__v01.pdf"):
 *  
 *      Microcontoller :  ATMega328 @ 3.3V, 8MHz
 *      Accelerometer  :  ADXL345
 *      Magnetometer   :  HMC5883L
 *      Gyroscope      :  ITG-3200/3205
 *
 *  
 *  For the physical orientations of the individual sensor chips see file "IMU chip orientation.pdf".
 *  Axes definitions are as per aerospace convention:
 *  
 *      X axis pointing forward
 *      Y axis pointing to the right
 *      Z axis pointing down.
 *        
 *      Positive yaw   : clockwise
 *      Positive roll  : right wing down
 *      Positive pitch : nose up
 *  
 *  Transformation order: first YAW then PITCH then ROLL.
 *
 ***********************************************************************************************************/


void init_AHRS()
{
    delay(50);                              // Give sensors enough time to start
    I2C_Init();
    Accel_Init();
    Magn_Init();
    Gyro_Init();
    
    // Read sensors, init DCM algorithm
    delay(20);                              // Give sensors enough time to collect data
    reset_sensor_fusion();                  // DENNA BÖR KANSKE KÖRAS VARJE START AV ROBOTIC NAV MODES

    // Init serial connection
    if(check_if_commands_enabled()){
        Serial.begin(OUTPUT__BAUD_RATE);    // 57600
        Serial.println("__________________________________________________\n");
        Serial.println("\n *** Inits AHRS with User Commands Enabled.");
        Serial.print("                     Output Mode:   "); Serial.println(output_mode, DEC);
        Serial.print("                     Output Format: "); Serial.println(output_format, DEC);
        turn_output_stream_on();            // Init output
    }
    else if(OUTPUT__STARTUP_STREAM_ON == false){
        Serial.begin(VARIO__BAUD_RATE);     // 9600
        Serial.println("\n *** Inits AHRS with User Commands Disabled _but_ Output Streaming ON");
        turn_output_stream_on();            // Init output
    }
    else {
        Serial.begin(VARIO__BAUD_RATE);     // 9600
        Serial.println("\n *** Inits AHRS with User Commands Disabled.");
        turn_output_stream_off();            // Inhibit output
    }
}


void read_sensors() 
{
    Read_Gyro();                            // Read gyroscope
    Read_Accel();                           // Read accelerometer
    Read_Magn();                            // Read magnetometer
}


void reset_sensor_fusion() 
{
    float temp1[3];
    float temp2[3];
    float xAxis[] = {1.0f, 0.0f, 0.0f  };

    read_sensors();                         // Read every sensor
    timestamp = millis();
    
    // GET PITCH      (Using y-z-plane-component/x-component of gravity vector)
    pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

    // GET ROLL       (Compensate pitch of gravity vector)
    Vector_Cross_Product(temp1, accel, xAxis);
    Vector_Cross_Product(temp2, xAxis, temp1);
    // Normally using x-z-plane-component/y-component of compensated gravity vector:
//  roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2])); // Skall denna vara UK-ad?
    // Since we compensated for pitch, x-z-plane-component equals z-component:
    roll = atan2(temp2[1], temp2[2]);

    // GET YAW
    Heading_calculator();
    yaw = Mag_Heading;

    // Init rotation matrix
    init_rotation_matrix(DCM_Matrix, yaw, pitch, roll); // Init DCM with unfiltered orientation
}



void compensate_sensor_errors()             // Apply calibration to all raw sensor readings
{
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    
    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++) {
        magnetom_tmp[i] = magn[i] - magn_ellipsoid_center[i];
    }
//    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magn);
#else
    magn[0] = (magn[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magn[1] = (magn[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magn[2] = (magn[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}


void check_reset_calibration_session() 
{
  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;

  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magn[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

  reset_calibration_session_flag = false;
}


void oiler_who()
{
    // Apply sensor calibration:
    compensate_sensor_errors();
    
    // Run DCM algorithm:
    Heading_calculator();                   // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();                         // Update the YPR angles based on the actual attitude 
}

 
boolean check_if_commands_enabled()           // This flag is set once only, i.e. in init_AHRS().
{ 
    if(digitalRead(JMPR_DETECT_PIN) == LOW)  commands_enabled_flag = true;  // Circuit is closed, so "true".
    else  commands_enabled_flag = false;
//    digitalWrite(JMPR_DETECT_PIN, LOW);     // Release the pullup: we don't need this circuit anymore.
    return commands_enabled_flag;
}


void run_AHRS()
{
    // Read incoming control messages?. NOTE: For this to work the vario serial input must be disconnected.
    if (commands_enabled_flag && (Serial.available() >= 2)) {
        respond_to_control_message_input(); 
    }
    // Update sensor readings
    read_sensors();
    oiler_who();
    if (!commands_enabled_flag) {
        ref_roll = ((int16_t)TO_DEG(roll) * 100);
        ref_pitch = ((int16_t)TO_DEG(pitch) * 100);
        return;                             // This is as far as we care to go if actually flying.
    }
    else  output_whatever();                // When commands are enabled, we just output whatever is required!
                                            /// DENNA FUNKTION KOSTAR ENSAM 2190 bytes!
#if DEBUG__PRINT_LOOP_TIME
    Serial.print("loop time (ms) = ");
    Serial.println(DIYmillis() - timestamp);
#endif
}
