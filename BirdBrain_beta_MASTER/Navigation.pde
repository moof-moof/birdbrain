                                              /** Supporting file for BirdBrain_beta_xx */

/// ALL NAVIGATION FUNCTIONS DEFINED HERE ARE CALLED DIRECTLY OR INDIRECTLY IN THE MEDIUM LOOP (10Hz)

// Find the error for the navigation PID algorithm:
// ------------------------------------------------
int32_t calc_course_error()
{
    now_heading = ((int16_t)TO_DEG(yaw) * 100);         // Updates the heading input here. (+/- 18000)
    course_error = targ_heading - now_heading;          // Calculates error.
    if (course_error > 18000)   course_error -= 36000;  // Constrains runaway values to +/- 18000
    if (course_error < -18000)  course_error += 36000;
//Serial.print("\n now_h ");  Serial.print(now_heading);
//Serial.print("\t c_e ");  Serial.print(course_error);
    return course_error;
}


/*
int16_t fir_filter(int16_t inval)    /// Confuses error calculation at wrap-around values. Don't use.
{
    int32_t in_val_sum = 0;
    int16_t out_val = 0;

    for (int8_t i = 0; i < (RW_LENGTH - 1); i++){
        run_win[i] = run_win[i + 1];        // Nudge-nudge
        in_val_sum += run_win[i];           // Sum up the values in the window
    }
    run_win[RW_LENGTH - 1] = inval;         // Stick this loop's input into last position of window array
    in_val_sum += run_win[RW_LENGTH - 1];   // Include latest value in the aggregate
    out_val = in_val_sum / RW_LENGTH;       // And what do we get?

    return out_val;
}
*/

// Guidance commands that output an appropriate setpoint for the navigation PID algorithm:
// ---------------------------------------------------------------------------------------
void set_course(uint8_t mode)
{
    if(mode == GLIDE_NVGN){
        if (DIP_selector == 0) {                   // Normal state of affairs
            coast_it();
        }
        else coast_it();//screw_it();     // For trimming the gains and rates w/o "disturbing" lift.
    }
    else {
        screw_it();
    }
    targ_heading = head_wrap(targ_heading);
}



void coast_it()
{
    targ_heading = heading_default;                           // "Static" setpoint (+/- 18000)
    nav_pitch = ALPHA_LD - PITCH_DATUM;                       // Run baby, run
}



void screw_it()   /// Note: Timing in the loop for this function is offset from error calculation timing!
{
    targ_heading = ((int16_t)TO_DEG(yaw) * 100) + progressor;  // Dynamic setpoint
    nav_pitch = ALPHA_LIFT - PITCH_DATUM;                      // Float baby, float
}


/*
void screw_it()
{
    if(ABS(course_error) < (progressor / 2)){
        targ_heading = ((int16_t)TO_DEG(yaw) * 100) + progressor;
        nav_pitch = ALPHA_LIFT - PITCH_DATUM;
    }
}
*/


int32_t  head_wrap(int32_t hdg)         // used for keeping track of continuous turning in HELIX_NVGN
{
    if (hdg > 36000)  hdg -= 36000;
    if (hdg < 0)      hdg += 36000;
    return hdg;
}
