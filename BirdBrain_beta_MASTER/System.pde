                                              /** Supporting file for BirdBrain_beta_xx */
void init_BirdBrain()
{
    /// ATmega328 TQFP pin-out is assumed below:
    /// ~~~~~~~~~~~~~~~~~~~~~~~~~~~  PORTC
    pinMode(JMPR_SINK_PIN, OUTPUT); // PC0 -        Low end of jumper circuit
    pinMode(JMPR_DETECT_PIN,INPUT); // PC1 -        High end of jumper circuit
    pinMode(BUTTON_PIN, INPUT);     // PC2 -        Trigger for setting default heading
    // A3                           // PC3 -
    // A4                           // PC4 - SDA    i2c I/O (3x3 DOF IMU board)
    // A5                           // PC5 - SCL    i2c I/O (3x3 DOF IMU board)
    // A6                           // PC6 -
    // A7                           // PC7 -

    /// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PORTD
    // D0                           // PD0 - RXD    Serial RX : Vario signals, or user serial input
    // D1                           // PD1 - TXD    Serial TX : Debugging output
    pinMode(2, INPUT); //(5V -> 3V3)   PD2 - INT0   INPUT Rudder/Aileron
    pinMode(3, INPUT); //(5V -> 3V3)   PD3 - INT1   INPUT Elevator 
    pinMode(4, INPUT); //(5V <-> 3V3)  PD4 - XCK/T0  MUX pin : Connected to PB1 (MUX/MISO) on ATtiny45
    pinMode(5, INPUT);              // PD5 - T0     Optional DIP switches for setting up to four 
    pinMode(6, INPUT);              // PD6 - T1     ... bits of "on the fly" configuration choices.
    pinMode(7, INPUT);              // PD7 - AIN0   "   "   "   "   "   "   "   "   "   "   "   "

    /// ~~~~~~~~~~~~~~~~~~~~~~~~~~~  PORTB
    pinMode(8, INPUT);              // PB0 - AIN1   "   "   "   "   "   "   "   "   "   "   "   "
    pinMode(9, OUTPUT);             // PB1 - OC1A   Rudder PWM out
    pinMode(10, OUTPUT);            // PB2 - OC1B   Elevator PWM out
    // D11                          // PB3 - MOSI/OC2   
    pinMode(OKAY_LED_PIN, OUTPUT);  // PB4 - MISO   Confirmation signal LED
    pinMode(STATUS_LED_PIN,OUTPUT); // PB5 - SCK    Status signal LED (for AHRS calibration mode)


    digitalWrite(JMPR_SINK_PIN, LOW);     // Drop analogue pin 0 to ground
    digitalWrite(JMPR_DETECT_PIN, HIGH);  // Set pullup resistor on analogue pin 1
    digitalWrite(BUTTON_PIN, HIGH);       // Set pullup resistor on analogue pin 2
    digitalWrite(OKAY_LED_PIN, LOW);      // Turn LED off
    digitalWrite(STATUS_LED_PIN, LOW);    // Turn LED off
#if USE_DIP_CONFIG
    digitalWrite(5, HIGH);                // Set pullup resistor on digital pin 5
    digitalWrite(6, HIGH);                // Set pullup resistor on digital pin 6
    digitalWrite(7, HIGH);                // Set pullup resistor on digital pin 7
    digitalWrite(8, HIGH);                // Set pullup resistor on digital pin 8
#endif

    init_AHRS();
    init_control_switch();              // Setup control switch   // Här kollas switchen första gången
    init_radio();                       // Connect to radio 
    init_PWM();                         // Setup PWM timers
    reset_control_switch();             // Set the correct control mode  // Här kollas andra gången!
#if USE_DIP_CONFIG
    check_DIP_bits();                   // Determine if any temporary configuration choices apply.
#endif
    Serial.println("__________________________________________________\n");
    Serial.print("\n *** Inits BirdBrain Beta.      ");
    Serial.print("Free RAM: ");
    Serial.println(freeRAM(), DEC);
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    Serial.println("<><><><><><><><><><><><><><><>");
    if (USE_DIP_CONFIG){Serial.println("USE_DIP_CONFIG  true");}
    else {Serial.println("USE_DIP_CONFIG  false");}
    Serial.print("DIP_selector: "); Serial.println(DIP_selector, DEC);
    Serial.print("DIP_value: "); Serial.println(DIP_value, BIN);
    Serial.print("DIP_configuration: "); Serial.println(DIP_configuration, DEC);
    Serial.println("<><><><><><><><><><><><><><><>");
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    if(switch_position == POSITION_1){  // RC Tx switch is Off (position 1)
        Serial.println("\n Startup mode: Old-School Manual Control (via MUX)\n");
        Serial.println("__________________________________________________\n");
        preflight_procedures();
    }
    else {                              // RC Tx switch is On (position 2)
        Serial.println("\n  Startup mode WARNING!\n   Robotic Mode??? You're kidding, right? {%^D)\n" );
        Serial.println("_____________________________________________________________________\n");
    }
    delay(100);
}



void preflight_procedures(void)    // Called exclusively in RC mode  - Needs 5v translation
{
//    demo_servos();                               // Makes the servos wiggle
//                                                 // DO SOMETHING HERE
//    demo_servos(); demo_servos(); demo_servos(); // All systems: Go!
}



void init_control_switch(void)          /// Används i init_BirdBrain() ovan.
{
    old_switch_pos = switch_position = read_switch();
}



void reset_control_switch(void)         /// Används bara i init_BirdBrain() ovan
{
    old_switch_pos = 0;
    check_control_switch_pos();
}



void check_control_switch_pos(void)     /// Denna funk exekveras i Slow loop och indirekt i setup()
{
    switch_position = read_switch();
    if (old_switch_pos != switch_position){ /// Första varvet i setup() blir alltid true emedan old==0
        switch(switch_position){
            case 1:                     // First Switch Position 
                    toggle_control_mode(POSITION_1);
                    break;

            case 2:                     // Second Switch Position
                    toggle_control_mode(POSITION_2);
                    break;
        }
        old_switch_pos = switch_position;
        reset_I_D();                    // reset navigation integrators if new switch position
    }                                   // ˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇˇ
    else {                              // i.e. old_switch_pos == switch_position
        if (control_mode == REMOTE_CTRL) {
            last_radio_in[CH_ROLL] = radio_trim[CH_ROLL];
            last_radio_in[CH_PITCH] = radio_trim[CH_PITCH];
        }
    }
}



byte read_switch(void)
{
    if(digitalRead(4) == HIGH){     // Muxpin high? (nominally 5 volts)
        return 2;                   // 2nd Switch Position => ON => ROBOTIC_CTRL
    }
    else {
        return 1;                   // 1st Switch Position => OFF => REMOTE_CTRL
    }
}


void toggle_control_mode(byte mode)
{
    if(control_mode == mode){
    return;                         // Mustn't switch modes if we are already in the correct mode.
    }
#if AUTO_TRIM == 1
    /// Här är control_mode fortfarande = föregående mode! Ergo måste den NYA vara ROBOTIC_CTRL:
    if(control_mode == REMOTE_CTRL){
        OCR1A = radio_out[CH_ROLL] = last_radio_in[CH_ROLL];   // Will this smooth transition?
        OCR1B = radio_out[CH_PITCH] = last_radio_in[CH_PITCH]; 
    }
#endif
    control_mode = mode;    // OK, There's somethin happening here. What it is ain't exactly clear...

    switch(control_mode) {
        case REMOTE_CTRL:              // Gör vad som behövs för att stoppa PID:en, ev spara trimvärden?
            break;

        case ROBOTIC_CTRL:             // Gör vad som behövs för att dra igång PID, etc
#if DEBUG__PRINT_CONTROL_VARS
            Serial.println(" ######### Going robotic! #########\n");
#endif
            reset_sensor_fusion();                      // Är detta verkligen så smart??
            now_heading = ((int16_t)TO_DEG(yaw) * 100);
            break;
    }
}


uint32_t freeRAM()
{
    uint8_t * heapptr, * stackptr;
    stackptr = (uint8_t *)malloc(4);    // use stackptr temporarily
    heapptr = stackptr;                 // save value of heap pointer
    free(stackptr);                     // free up the memory again (sets stackptr to 0)
    stackptr = (uint8_t *)(SP);         // save value of stack pointer
    return stackptr - heapptr;
}



void time_Ladies_time()
{
    deltaMilliseconds = DIYmillis() - timestamp; // Should be equal to OUTPUT_DATA_INTERVAL
    timestamp_old = timestamp;
    timestamp = DIYmillis();
    if (timestamp > timestamp_old){    // In case the actual loop time fluctuates
        G_Dt = (float)(timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    } else G_Dt = 0;
}


uint32_t DIYmillis()                    // This function replaces the usual arduino millis() function
{
    uint32_t m;
    uint32_t m2;
    
/**  timer0_millis could change inside timer0 interrupt, and we don´t want to disable interrupts, 
     so instead we can do two readings and compare.   */
    m = timer0_millis;
    m2 = timer0_millis;
    if (m != m2){                        // timer0_millis corrupted?
        m = timer0_millis;               // This should be fine...
    }
    return m;
}

#if USE_DIP_CONFIG                      // Initialise these global variables only when required.
    float var_SERVO_ROLL_P          = 0.0;
    float var_SERVO_ROLL_I          = 0.0;
    float var_SERVO_ROLL_D          = 0.0;
    int   var_SERVO_ROLL_INT_MAX    = 0;
    float var_ROLL_SLEW_LIMIT       = 0;
    float var_SERVO_PITCH_P         = 0.0;
    float var_SERVO_PITCH_I         = 0.0;
    float var_SERVO_PITCH_D         = 0.0;
    int   var_SERVO_PITCH_INT_MAX   = 0;
    float var_PITCH_COMP            = 0.0;
    float var_NAV_ROLL_P            = 0.0;
    float var_NAV_ROLL_I            = 0.0;
    float var_NAV_ROLL_D            = 0.0;
    int   var_NAV_ROLL_INT_MAX      = 0;
#endif

void check_DIP_bits(void)                /// Används i init_BirdBrain() ovan.
{
    uint8_t DIP_pins[] = {5, 6, 7, 8};      // DIP switch pins mapping 

    if (digitalRead(DIP_pins[0]) == LOW) {  // If switch #1 is ON: Store entire DIP array value.
        DIP_selector = 1;
        for(uint8_t i=0; i<=3; i++){
            DIP_value = (DIP_value << 1) | !digitalRead(DIP_pins[i]);  // VILKEN ORDNING? L>R ELLER R>L?
        }
        switch (DIP_value){
            case B1000:
            DIP_configuration = 0;          // Default, same as if no DIP selection, for now.
            break;
            
            case B1001:
            DIP_configuration = 1;
            break;
            
            case B1010:
            DIP_configuration = 2;
            break;
            
            case B1011:
            DIP_configuration = 3;
            break;
            
            case B1100:
            DIP_configuration = 4;
            break;
            
            case B1101:
            DIP_configuration = 5;
            break;
            
            case B1110:
            DIP_configuration = 6;
            break;
            
            case B1111:
            DIP_configuration = 7;
            break;
        }
        // Här borde switchen nedan komma in, eftersom vi vet att DIP_selector == 1
    }
    else{
        DIP_selector = 0;
        DIP_configuration = 0;
    }
//#if USE_DIP_CONFIG
    switch (DIP_configuration){
        case 0:                                    // Default, same as with no DIP selection.
            var_SERVO_ROLL_P          =    .004   ;/// TESTAT (NO DIP) 13052?: ÖVERSKJUTER
            var_SERVO_ROLL_I          =    .0     ;/// SETPOINT (HEADING_DEFAULT) OM MAN BÖRJAR
            var_SERVO_ROLL_D          =    .0     ;/// LÅNGT FRÅN SETPOINT. DESSUTOM HINNER DEN
            var_SERVO_ROLL_INT_MAX    = 500       ;/// GÅ I SPIRALDYKNING OCH TAPPA MYCKET HÖJD.
            var_ROLL_SLEW_LIMIT       =   0       ;/// FUNKAR BÄTTRE MED START NÄRMARE SETPOINT,
            var_SERVO_PITCH_P         =    .005   ;/// MEN FORTFARANDE OVERSHOOT-TENDENSER.
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .7     ;
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .02    ;
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;

        case 1:                                       /// ETT SNÄPP UPP PÅ PROPO-VÄRDENA
            var_SERVO_ROLL_P          =    .005   ;   // +.01
            var_SERVO_ROLL_I          =    .0     ;
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .006   ;   // +.01
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .8     ;   // +.1
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .02    ;
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;

        case 2:                                       /// TVÅ SNÄPP UPP PÅ PROPO-VÄRDENA
            var_SERVO_ROLL_P          =    .006   ;   // +.02
            var_SERVO_ROLL_I          =    .0     ;
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .007   ;   // +.02
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .9     ;   // +.2
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .02    ;
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;

        case 3:                                       /// ETT SNÄPP NER PÅ PROPO-VÄRDENA
            var_SERVO_ROLL_P          =    .003   ;   // -.01
            var_SERVO_ROLL_I          =    .0     ;
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .004   ;   // -.01
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .6     ;   // -.1
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .02    ;
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;

        case 4:                                       /// TESTAD 130703 PÅ FOTBOLLSPLAN
            var_SERVO_ROLL_P          =    .004   ;   /// MED OKLART RESULTAT PGA ANDRA PROBLEM
            var_SERVO_ROLL_I          =    .0     ;   /// -- Nytt test bör göras omgående
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .005   ;
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .4     ;   // -.3
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .03    ;   // +.01
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;
        
        case 5:                                         
            var_SERVO_ROLL_P          =    .004   ;
            var_SERVO_ROLL_I          =    .0     ;
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .005   ;
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .3     ;   // +.1
            var_NAV_ROLL_P            =    .4     ;   // -.3
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .03    ;   // +.01
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;

        case 6:                                         
            var_SERVO_ROLL_P          =    .004   ;
            var_SERVO_ROLL_I          =    .0     ;
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .005   ;
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .7     ;
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .02    ;
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;

        case 7:                                         
            var_SERVO_ROLL_P          =    .004   ;
            var_SERVO_ROLL_I          =    .0     ;
            var_SERVO_ROLL_D          =    .0     ;
            var_SERVO_ROLL_INT_MAX    = 500       ;
            var_ROLL_SLEW_LIMIT       =   0       ;
            var_SERVO_PITCH_P         =    .005   ;
            var_SERVO_PITCH_I         =    .0     ;
            var_SERVO_PITCH_D         =    .0     ;
            var_SERVO_PITCH_INT_MAX   = 500       ;
            var_PITCH_COMP            =    .2     ;
            var_NAV_ROLL_P            =    .7     ;
            var_NAV_ROLL_I            =    .01    ;
            var_NAV_ROLL_D            =    .02    ;
            var_NAV_ROLL_INT_MAX      = 500       ;
        break;
    }
//#endif
    for(uint8_t ii = 0; ii <= 3; ii++){
        digitalWrite(DIP_pins[ii], LOW);     // Release pullups again -- save a nanoAmp.
    }
}
