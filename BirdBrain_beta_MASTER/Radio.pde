                                              /** Supporting file for BirdBrain_beta_xx */

uint16_t timer1count    = 0;
uint16_t timer2count    = 0;

uint16_t timer1diff     = 1500;
uint16_t timer2diff     = 1500;

uint8_t ch_read         = 0;
boolean ch1_read        = 0;
boolean ch2_read        = 0;



void read_RCRX(void)                    // This is read in fast_loop() when in REMOTE_CTRL
{
    //Filter Radio input
    timer1diff -= 46;
    ch1_tempLast = ch1_temp = timer1diff;         // RUDD (AIL)

    timer2diff -= 38;
    ch2_tempLast = ch2_temp = timer2diff;         // ELEV

//// filter
//if( ch1_tempLast-ch1_temp < 50) ch1_temp = ch1_tempLast;
//if( ch2_tempLast-ch2_temp < 50) ch2_temp = ch2_tempLast;

    radio_in[CH_ROLL] = ch1_temp;
    radio_in[CH_PITCH] = ch2_temp;
}



void init_radio(void)
{
//    // enable pin change interrupt on PB3, PB5 (digital PWM pins 11, 13)       
//    PCMSK0 = _BV(PCINT3) | _BV(PCINT5);                     /// Behövs interrupt på pin D11 och D13 ??
        
    // enable pin change interrupt on PD2,PD3 (digital PWM pins 2,3)
    PCMSK2 = _BV(PCINT18) | _BV(PCINT19); ///För att läsa CH1 & CH2.
}



#if RADIO_TYPE == 0                     // i.e. sequential PWM pulse type
ISR(PCINT2_vect) {                      // "Pin Change Interrupt Request2" handled here
    int cnt = TCNT1;
    
    if(PIND & B00000100) {              // ch 1 (pin 2)
        if (ch1_read == 0) {
            ch1_read = 1;
            timer1count = cnt;
        }
    }
    else if (ch1_read == 1) {
        ch1_read = 0;
        if (cnt < timer1count) {        // Timer1 reset during the read of this pulse
           timer1diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
        }
        else {
          timer1diff = (cnt - timer1count);
        }
    }
    
    if(PIND & B00001000) {              // ch 2 (pin 3)
        if (ch2_read==0){
            ch2_read = 1;
            timer2count = cnt;
        }
    }
    else if (ch2_read == 1) {
        ch2_read = 0;
        if (cnt < timer2count) {        // Timer1 reset during the read of this pulse
           timer2diff = (cnt + 40000 - timer2count);    // Timer1 TOP = 40000
        }
        else {
          timer2diff = (cnt - timer2count);
        }
    }
}
#endif
