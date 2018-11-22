                                              /** Supporting file for BirdBrain_beta_xx */

//void demo_servos()  // Används i preflight_procedures och check_n_set_heading...
//{
//    delay(30);
//    set_servo_mux(true);
//    OCR1A = 1600 * 2;       // CH 1
//    OCR1B = 1600 * 2;       // CH 2
//    delay(400);
//    OCR1A = 1400 * 2;
//    OCR1B = 1400 * 2;
//    delay(200);
//    OCR1A = 1500 * 2;
//    OCR1B = 1500 * 2;
//    set_servo_mux(false);
//    delay(30);
//}

//void set_servo_mux(boolean mode)    // Used for demo-ing servos
//{
//    while(TCNT1 < 20000){};         /// BLOCKING UNTIL WHAT??   TCNT1 är en 16-bit timer/counter
//    if (mode){                      /// TCNT1 is 16 bit register, that saves counter value
//        //Take over the MUX    
//        pinMode(4, OUTPUT);
//        digitalWrite(4, HIGH);
//    }else {
//        //Release the MUX to allow Manual Control
//        digitalWrite(4, LOW); 
//        pinMode(4, INPUT);
//    }
//}


void set_servos_2()                      // wants +/-45°
{
    set_ch1_degrees(servo_out[CH_ROLL]); // 45 ° = right turn (unless reversed)
    set_ch2_degrees(servo_out[CH_PITCH]);
}


void set_ch1_degrees(float deg)
{
    radio_out[CH_ROLL] = radio_trim[CH_ROLL] + ((float)REVERSE_ROLL * deg * 11.111f);
    radio_out[CH_ROLL] = constrain(radio_out[CH_ROLL],     radio_min[CH_ROLL], radio_max[CH_ROLL]);
    radio_out[CH_ROLL] = constrain(radio_out[CH_ROLL],     1000,     2000);
    OCR1A = radio_out[CH_ROLL];// * 2;     //OCR1A is the channel 1 pulse width in _half_ microseconds
}                                       /// Gäller detta även vid 8 Mz? Verkar inte så ...


void set_ch2_degrees(float deg)
{
    radio_out[CH_PITCH] = radio_trim[CH_PITCH] + ((float)REVERSE_PITCH * deg * 11.111f);
    radio_out[CH_PITCH] = constrain(radio_out[CH_PITCH],     radio_min[CH_PITCH],     radio_max[CH_PITCH]);
    radio_out[CH_PITCH] = constrain(radio_out[CH_PITCH],     1000,     2000);
    OCR1B = radio_out[CH_PITCH];// * 2;     //OCR1B is the channel 2 pulse width in _half_ microseconds
}


void init_PWM()
{
    // Timer 1
    TCCR1A = ((1<<WGM11) | (1<<COM1B1) | (1<<COM1A1)); //Fast PWM: ICR1=TOP, OCR1x=BOTTOM,TOV1=TOP
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Clock scaler = 8, 2,000,000 counts per second
    OCR1A = 1500;   // Rudder  - for example 1500 = 45°; 2000 = 90°
    OCR1B = 1500;   // Elevator
    ICR1 = 20000;   // 50Hz freq...Datasheet says (system_freq/prescaler)/target frequency,
                    // so (8000000hz/8)/50hz = 20000,

    // Enable pin change interrupt 2 - PCINT23..16   ...........?? Kolla ev kollision med 3x3 AHRS-en
    PCICR = _BV(PCIE2);
    
//    // Enable pin change interrupt 0 -  PCINT7..0    ...........?? Kolla ev kollision med 3x3 AHRS-en
//    PCICR |= _BV(PCIE0);
}
