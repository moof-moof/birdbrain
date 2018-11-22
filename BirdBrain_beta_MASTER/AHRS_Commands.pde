                                              /** Supporting file for BirdBrain_beta_xx */

/********************* Serial commands that the firmware understands: ************************
  
  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
  
      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.
      
      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      
      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.
      
      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards. In fact it's too much and an output frame rate of 50Hz can not be maintained.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
      
      // Error message output        
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.
    
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
         
         
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.
          
  
  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
  
  The status LED will be on if streaming output is enabled and off otherwise.
  
  Byte order of binary output is little-endian: least significant byte comes first.
*/


/********************  FUNCTIONS FOR SERIAL I/O CONTROL ****************************/

void turn_output_stream_on(void)
{
  output_stream_on_flag = true;
  digitalWrite(STATUS_LED_PIN, HIGH);       // #13
}


void turn_output_stream_off(void)
{
  output_stream_on_flag = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}


// Blocks until another byte is available on serial port
char readChar(void)
{
  while (Serial.available() < 1) { } /// Block ... Just waiting for a user control input? 
  return Serial.read();
}


void respond_to_control_message_input(void)
{
  if (Serial.read() == '#') // Start of new control message
  {
    int command = Serial.read(); // Commands
    if (command == 'f') // request one output _f_rame
      output_single_on_flag = true;
    else if (command == 's') // _s_ynch request
    {
      // Read ID
      byte id[2];
      id[0] = readChar();
      id[1] = readChar();

      // Reply with synch message
      Serial.print("#SYNCH");
      Serial.write(id, 2);
      Serial.println();
    }
    else if (command == 'o') // Set _o_utput mode
    {
      char output_param = readChar();
      if (output_param == 'n')  // Calibrate _n_ext sensor
      {
        curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
        reset_calibration_session_flag = true;
      }
      else if (output_param == 't') // Output angles as _t_ext
      {
        output_mode = OUTPUT__MODE_ANGLES;
        output_format = OUTPUT__FORMAT_TEXT;
      }
      else if (output_param == 'b') // Output angles in _b_inary format
      {
        output_mode = OUTPUT__MODE_ANGLES;
        output_format = OUTPUT__FORMAT_BINARY;
      }
      else if (output_param == 'c') // Go to _c_alibration mode
      {
        output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
        reset_calibration_session_flag = true;
      }
      else if (output_param == 's') // Output _s_ensor values
      {
        char values_param = readChar();
        char format_param = readChar();
        if (values_param == 'r')  // Output _r_aw sensor values
          output_mode = OUTPUT__MODE_SENSORS_RAW;
        else if (values_param == 'c')  // Output _c_alibrated sensor values
          output_mode = OUTPUT__MODE_SENSORS_CALIB;
        else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
          output_mode = OUTPUT__MODE_SENSORS_BOTH;
        if (format_param == 't') // Output values as _t_text
          output_format = OUTPUT__FORMAT_TEXT;
        else if (format_param == 'b') // Output values in _b_inary format
          output_format = OUTPUT__FORMAT_BINARY;
      }
      else if (output_param == '0') // Disable continuous streaming output
      {
        turn_output_stream_off();
        reset_calibration_session_flag = true;
      }
      else if (output_param == '1') // Enable continuous streaming output
      {
        reset_calibration_session_flag = true;
        turn_output_stream_on();
      }
      else if (output_param == 'e') // _e_rror output settings
      {
        char error_param = readChar();
        if (error_param == '0') output_errors = false;
        else if (error_param == '1') output_errors = true;
        else if (error_param == 'c') // get error count
        {
          Serial.print("#AMG-ERR:");
          Serial.print(num_accel_errors); Serial.print(",");
          Serial.print(num_magn_errors); Serial.print(",");
          Serial.println(num_gyro_errors);
        }
      }
    }  
  }
  else
  { } // Skip character
}

