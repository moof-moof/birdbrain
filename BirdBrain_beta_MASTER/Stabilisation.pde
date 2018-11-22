                                              /** Supporting file for BirdBrain_beta_xx */

/************************************************************************
 * Function that governs the control surfaces to achieve desired attitude   (Called in the fast loop)
 ************************************************************************/


void steady_now_steady()
{
    // Calculate desired servo output for the roll.
    // servo_out has switched to degrees * 1
    // ---------------------------------------------
    servo_out[CH_ROLL] = PID((nav_roll - ref_roll), deltaMilliseconds, CASE_SERVO_ROLL, ref_roll);
    servo_out[CH_PITCH] = PID((nav_pitch + abs(ref_roll * PITCH_COMP) - ref_pitch), deltaMilliseconds, CASE_SERVO_PITCH, ref_pitch);  
    
    // Call slew rate limiter if used
    // ------------------------------
    #if(ROLL_SLEW_LIMIT != 0)
        servo_out[CH_ROLL] = roll_slew_limit(servo_out[CH_ROLL]);
    #endif
}


/************************************************************
 * Computes desired roll angle based on navigation data input    (Called in medium freq loop)
 ************************************************************/

void calc_nav_roll()
{
    nav_roll = PID(calc_course_error(), dTnav, CASE_NAV_ROLL, now_heading);
    nav_roll = constrain(nav_roll,-BANK_MAX, BANK_MAX);    //returns bank angle in degrees*100 (max 4000)
}


float roll_slew_limit(float servo)
{                                   // Does this really work?!
    static float last;
    float temp = constrain(servo, last-ROLL_SLEW_LIMIT * deltaMilliseconds/1000.f, last + ROLL_SLEW_LIMIT * deltaMilliseconds/1000.f);
    last = servo;
    return temp;
}


/********************************************          THIS IS THE SIMPLER PID FORM TO USE:
 * Proportional Integrator Derivative Control #1        * IF LOOP-TIME REGULARITY IS NOT GUARANTEED
 ********************************************           * IF DERIVATIVE KICK CAN BE TOLERATED      */

float PID(int32_t PID_error, int32_t dt, int16_t PID_case, int32_t PID_input) // Last param not used here!
{
    float output;

    float derivative = 1000.0f * (PID_error - prev_error[PID_case]) / (float)dt;
    prev_error[PID_case] = PID_error;
    output = (kp[PID_case] * PID_error);                    //Compute and add proportional component.
                                                            //Positive error produces positive output
    integrator[PID_case] += (float)PID_error*(float)dt/1000.0f; 
    integrator[PID_case] = constrain(integrator[PID_case], -integrator_max[PID_case], integrator_max[PID_case]); 
  
    output += integrator[PID_case] * ki[PID_case];          //Add the integral component
    output += derivative * kd[PID_case];                    //Add the derivative component

    return output;  
}


/********************************************          THIS IS THE PREFERRED PID FORM TO USE:
 * Proportional Integrator Derivative Control #2        * IF LOOP-TIME REGULARITY IS NOT GUARANTEED
 ********************************************           * IF DERIVATIVE KICK MUST BE ELIMINATED    */
/*
float P_I_D(int32_t PID_error, int32_t dt, int16_t PID_case, int32_t PID_input)  // Needs overhaul...
{
    float output;

    float derivative = 1000.0f * (PID_input - prev_input[PID_case]) / (float)dt;
    prev_input[PID_case] = PID_error;                       //     ^^^
    output = (kp[PID_case] * PID_input);                    //Compute proportional component.
                                                            //Positive error produces positive output
    integrator[PID_case] += (float)PID_error*(float)dt/1000.0f; 
    integrator[PID_case] = constrain(integrator[PID_case], -integrator_max[PID_case], integrator_max[PID_case]); 
  
    output += integrator[PID_case] * ki[PID_case];          //Add the integral component
    output -= derivative * kd[PID_case];                    //Subtract the derivative component
//        ^^^
    return output; 
} */



void reset_I_D(void)                    // Keeps old data out of calculations if we are changing mode, etc.
{
    integrator[CASE_NAV_ROLL] = 0;      // For I
    prev_error[CASE_NAV_ROLL] = 0;      // For D
//    prev_input[CASE_NAV_ROLL] = 0;      // For D in P_I_D func

    integrator[CASE_SERVO_ROLL] = 0;
    prev_error[CASE_SERVO_ROLL] = 0;
//    prev_input[CASE_SERVO_ROLL] = 0;

    integrator[CASE_SERVO_PITCH] = 0;
    prev_error[CASE_SERVO_PITCH] = 0;
//    prev_input[CASE_SERVO_PITCH] = 0;
}



void  print_control_variables(void)         // For debugging purposes
{
    if(control_mode == ROBOTIC_CTRL){
        Serial.print(vertical_velocity, DEC);
        if(guidance_mode == GLIDE_NVGN)  Serial.println(" Glide! ");
        else                             Serial.println(" Soar!!!"); 
    }
    
    Serial.print(" Y ");        Serial.print(TO_DEG(yaw));
    Serial.print("  \tR ");     Serial.print(TO_DEG(roll));
    Serial.print("  \tP ");     Serial.print(TO_DEG(pitch)); 
    Serial.print("  |\tn_r ");  Serial.print(nav_roll);
    Serial.print("\tr_r ");     Serial.print(ref_roll);
    Serial.print("\terr");      Serial.print(nav_roll-ref_roll);
    Serial.print("  |\tsrv[R] "); Serial.print(servo_out[CH_ROLL]);
    Serial.print(" (");         Serial.print(radio_out[CH_ROLL]);
    Serial.print(")\tsrv[P] "); Serial.print(servo_out[CH_PITCH]);
    Serial.print(" (");         Serial.print(radio_out[CH_PITCH]); Serial.println(")");
}
