/*******************************************************
Name: functions.c
Date: 2/3/2011, updated 5/1/12
Authors: Frank Sup, Jacob Miller-Mack and Andrew Erwin
Comments:  Support file for functions
 *******************************************************/
#include "p33FJ64MC202.h"   // Include p33FJ64MC202 header file
#include "math.h"           // Include math libary
#include "support.h"
#include "PIC_serial.h"
#include "PID.h"     // Include Defitions and function prototypes
#include <stdio.h>          // Include C standard I/O library
#include <stdlib.h>         // include C standard library
#include <string.h>         // for work with strings

extern long unsigned int run_time;  // run time in milliseconds
extern int wait_flag;     // signals end of sample period
extern int sample_time;
extern int post_period;
extern int blink_period;
extern int step_period;
extern motor_struct motorX, motorY;
//extern pwm_struct pwm1, pwm2;
extern command_struct command;
extern int pause_all;
extern int printHeader;
extern int sys_state;
extern int testing;
extern int postflag;
char toPrint2[BUFFER_SIZE];
extern ramp_struct testRampRot;
extern ramp_struct testRampCab;
extern int ramp_timer_rot;
extern int ramp_timer_cab;
extern int coilPWM;
extern int post_period;
extern int blink_period;
extern int x_target;
extern int y_target;
extern gradient_data_struct* quadrant;
//
//////////////////////////////////////////////////////////
// Initialize of Clock Frequency

//Configuration Bits for Clock Initialization Parameters
_FOSCSEL(FNOSC_FRC); // Select Internal FRC at POR
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON); // Enable Clock Switching and use OSC2 as i/0
_FWDT(FWDTEN_OFF); // Turn off Watchdog Timer
_FICD(JTAGEN_OFF & ICS_PGD1); // Turn off JTAG and communicate on PGC1/EMUC1 and PGD1/EMUD1

void init_clock(void) {
    // Configure PLL prescaler, PLL postscaler, PLL divisor
    // 7.37 MHz * 43 / 2 / 2 = 79.227 MHz
    PLLFBD = 41; // M = 43
    CLKDIVbits.PLLPOST = 0; // N2 = 2
    CLKDIVbits.PLLPRE = 0; // N1 = 2

    // Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };
} //end init_clock

//////////////////////////////////////////////////////////
// Timer1 interupt service routine
// Used to enforce Sample Time

void _ISRFASTER _T1Interrupt(void) {
    run_time++; // increment millisecond timer
    if(run_time % sample_time == 0)
      wait_flag = 1; // Signal end of sample time
    IFS0bits.T1IF = 0;
} // end T1Interupt

// Timer2 iterrupt service routine
// used to update pwm1 and pwm2 duty cycles
void _ISRFASTER _T2Interrupt(void) {
//    OC1RS = motorX.pwm.duty;
//    OC1R = PWM_PERIOD - motorX.pwm.duty;
//
//    OC2RS = motorY.pwm.duty;
//    OC2R = PWM_PERIOD - motorY.pwm.duty;
//
//    OC3RS = coilPWM;
//    OC3R = PWM_PERIOD - coilPWM;

    IFS0bits.T2IF = 0; // reset interrupt flag
} // end T2Interupt

//////////////////////////////////////////////////////////
// Intialize Sample Time (1 msec)

void init_samptime(void) {
    TMR1 = 0;
    PR1 = 4952 - 1; // Set the period register - 1 msec
    T1CON = 0x8010; // TMR1 on, prescalar set to 1:8 Tclk/2
    _T1IF = 0; // Clear Flag
    _T1IE = 1; // Enable Source
} //end init_samptime


//////////////////////////////////////////////////////////
//Initialize Pins

void init_pins(void){
/* Pin Overview Map
          NEW:
                RB0 - thermopile quad1  RA0 - Potentiometer
                RB1 - thermopile quad2  RA1 - thermocouple in absorber
                RB2 - thermopile quad3  RA2 - GRN_LED
                RP3 - thermopile quad4  RA3 - RED_LED
                RP4 - UART1 TX          RA4 - Button
                RP5 - UART1 RX
                RP6 - Enc1 phaseA
                RP7 - Enc1 phaseB
                RP8 - Enc2 phaseA
                RP9 - Enc2 phaseB
                RP10 - MOTY_EN
                RP11 - MOTY_DIR
                RB12 - MOTY_STEP
                RB13 - MOTX_EN
                RB14 - MOTX_DIR
                RB15 - MOTX_STEP
  */
    AD1PCFGL = 0b1111111111000000;  // AN0-5 analog inputs
    //      0b5432109876543210
    TRISA = 0b0000000000010011; // RA0,1,4 input, rest PORTA pins output
    ODCA = 0;           // all PORTA pins set NOT open drain
    LATA = 0x0000;      // all PORTA output pins set to zero
    //      0b5432109876543210
    TRISB = 0b0000001111101111; // RB0-3,5-9, input,rest PORTB pins set to output
    ODCB = 0x0000;  //  no open drain pins on PORTB
    LATB = 0x0000;  //  all PORTB output pins set to zero

    // encoder pin assignment
    RPINR14bits.QEA1R = 6; // encoder1 phase A on RP6
    RPINR14bits.QEB1R = 7; // encoder1 phase B on RP7
    RPINR16bits.QEA2R = 9; // encoder2 phase A on RP9
    RPINR16bits.QEB2R = 8; // encoder2 phase B on RP8

    // pwm pin assignments
//    RPOR5bits.RP10R = 0b10010; // RP10 to output compare 1 (pwm1)
//    RPOR6bits.RP13R = 0b10011; // RP13 to output compare 2 (pwm2)
//    RPOR2bits.RP5R =  0b10100; // RP5 to output compare 3(voice coil pwm)

    // init voice coil PWM
    OC3CON = 0x0000; // disable PWM module
//    OC3RS = coilPWM; // set slave duty cycle
//    OC3R = PWM_PERIOD - coilPWM; // duty cycle
//    OC3CON = 0x0006; // activate PWM module

    // UART1 pin assignment
    RPINR18bits.U1RXR = 5;  // UART1 RX set to RP5
    RPOR2bits.RP4R = 0x3;   // RP4 set to UART1 TX

} // end init_pins

/*  button debouncing routine for polling button pin each sample period
    returns BTN_OFF unless button has just been let up.
    otherwise button has just been released:
    returns BTN_SHORT if button has been pressed more than BTN_THRESHOLD_LOW ms
    returns BTN_LONG if button has been pressed more than BTN_THRESHOLD_HI ms
 */
int btnDebounce(void) {
    // shared variables among all debounce calls

    // pushbutton state: off, short press, or long press
    static int button_state = BTN_OFF;

    // button debouncing counter gets incremented each ms button is down
    static int button_cts = 0;

    // check if button pin is high
    if (BTN_PORT & BTN_PIN) {
        button_cts++; // if pin's high, increment button count
    } else if (button_cts > 0) {
        button_cts--; // if button pin is low, decrement button count until zero
    }

    // decide what to do if button is not being pressed right now
    if (button_cts == 0) {
        switch (button_state) {
            case BTN_LONG: // button has been released after long press
                button_state = BTN_OFF; // reset button state to off
                return BTN_OFF;
            case BTN_SHORT: // button has been released after short press
                button_state = BTN_OFF; // reset button state
                return BTN_SHORT;
            default: // button has not been pressed at all
                return BTN_OFF; // return 0
        }
    }  // set button state if it has newly reached the short press threshold
    else if (button_cts == BTN_THRESHOLD_LOW && button_state == BTN_OFF) {
        button_state = BTN_SHORT; // set button state as short press
    } // set state to long press if it has newly reached long press threshold
    else if (button_cts == BTN_THRESHOLD_HI && button_state == BTN_SHORT) {
        button_state = BTN_OFF; // set button state to long press
        return BTN_LONG;
    }

    return BTN_OFF;
} // end btnDebounce

// initialize analog to digital converter

void init_ADC(void) {

    AD1CON1 = 0x04e0; // 12-bit conversion, auto convert after sampling
    AD1CSSL = 0; // disable scanning
    AD1CON2 = 0; // use MUXA
    AD1CON3 = 0x1f02; // max sample time = 31 * a/d clocktime
    
    AD1CON1bits.SIMSAM = 0; // sample one channel at a time
    AD1CON1bits.ADON = 1; // turn on ADC module
} // end init_ADC

// initialization for PWM1
//void init_pwm1(pwm_struct* pwm1){
//    // set up data structure for pwm1
//    pwm1->output = 0; // set desired output to zero
//    pwm1->duty = 0;
//    pwm1->offset = PWM1_OFFSET;
//    pwm1->enable = 0; // disable pwm
//    pwm1->DIR1_pin = PWM1_DIR1;// FWD direction pin for pwm1
//    pwm1->DIR2_pin = PWM1_DIR2;// REV direction pin for pwm1
//    pwm1->DIR1_state = 1; // state of FWD pin
//    pwm1->DIR2_state = 0; // state of REV pin
//    pwm1->LATCH = &PWM1_LATCH;
//
//
////    //init TMR2
//    OC1CON = 0x0000; // disable PWM module
////    T2CON = 0x8000; // prescalar 1:1
////    _T2IF = 0; // Clear Flag
////    _T2IE = 1; // Enable Source
////
////    // init PWM1
////    TMR2 = 0; // Reset timer value
////    PR2 = PWM_PERIOD - 1; // set PWM period
////    OC1RS = pwm1->duty; // set slave duty cycle
////    OC1R = PWM_PERIOD - pwm1->duty; // duty cycle
////    OC1CON = 0x0006; // activate PWM module
//}

// initialization for PWM2
//void init_pwm2(pwm_struct* pwm2){
//    // set up data structure for pwm1
//    pwm2->output = 0; // set desired output to zero
//    pwm2->duty = 0;
//    pwm2->offset = PWM2_OFFSET;
//    pwm2->enable = 0; // disable pwm
//    pwm2->DIR1_pin = PWM2_DIR1;// FWD direction pin number for pwm2
//    pwm2->DIR2_pin = PWM2_DIR2;// REV direction pin number for pwm2
//    pwm2->DIR1_state = 1; // state of FWD pin
//    pwm2->DIR2_state = 0; // state of REV pin
//    pwm2->LATCH = &PWM2_LATCH;
//
//    OC2CON = 0x0000; // disable PWM module
//
////    // init PWM1
////    OC2RS = pwm2->duty; // set slave duty cycle
////    OC2R = PWM_PERIOD - pwm2->duty; // duty cycle
////    OC2CON = 0x0006; // activate PWM module
//}

// initialize quadrature encoder module
void init_encoders(enc_struct* encoder1, enc_struct* encoder2){
    /* possible index_mod e settings:
    111 = Quad Enc Interface enabled (x4 mode) position counter reset by (MAXxCNT)
    110 = Quad Enc Interface enabled (x4 mode) Index Pulse reset of position ctr
    101 = Quad Enc Interface enabled (x2 mode) position counter reset by (MAXxCNT)
    100 = Quad Enc Interface enabled (x2 mode) Index Pulse reset of position ctr
    011 = Unused (Module disabled)
    010 = Unused (Module disabled)
    001 = Starts 16-bit Timer
    000 = Quadrature Encoder Interface/Timer off
     */
    //  QEI1CONbits.QEISIDL = 0;  // continue module operation in even in idle mode
    //
    //  QEI1CONbits.QEIM = index_mode; // set interface mode according to index_mode
    //
    //  QEI1CONbits.SWPAB   = 0;  // don't swap A and B phases
    //  QEI1CONbits.PCDOUT  = 0;  // disable position counter status output
    //  QEI1CONbits.POSRES  = 0;  // don't reset position ctr with index pulse
    //
    //  DFLT1CONbits.IMV    = 0;  // index match value (??)
    //  DFLT1CONbits.CEID   = 1;  // disable interrupts due to count errors
    //  DFLT1CONbits.QEOUT  = 0;  // disable digital filter output
    //  DFLT1CONbits.QECK   = 3;  // set clock subdivision 1:16

    QEI1CON = 0x0700; // initialize encoder interface
    MAX1CNT = ONE_REV; // number of counts per encoder revolution
    POS1CNT = 0; // reset encoder counts

    QEI2CON = 0x0700; // initialize encoder interface
    MAX2CNT = ONE_REV; // number of counts per encoder revolution
    POS2CNT = 0; // reset encoder counts

    // set up data structure for encoder1
    encoder1->POSCOUNT = &POS1CNT; // register pointer for encoder1
    // initialize variables to zero
    encoder1->posn_ref = 0;
    encoder1->posn_count = 0;
    encoder1->posn_delta = 0;
    encoder1->posn_old = 0;
    encoder1->posn_countold = 0;
    encoder1->posn = 0;

    // set up data structure for encoder2
    encoder2->POSCOUNT = &POS2CNT; // register pointer for encoder2
    // initialize variables to zero
    encoder2->posn_ref = 0;
    encoder2->posn_count = 0;
    encoder2->posn_delta = 0;
    encoder2->posn_old = 0;
    encoder2->posn_countold = 0;
    encoder2->posn = 0;
} // end init_Encoder

// function to read a/d channel. returns 12-bit (0 to 4096) voltage reading.
int read_ADC(int channel) {
    int i = 0; // counter for waiting loop
    AD1CHS0 = channel; // indicate which pin is to be read
    AD1CON1bits.SAMP = 1; // begin sampling
    i++;  // twiddle our thumbs for a couple instruction cycles
    while (!AD1CON1bits.DONE); // wait for sampling to be done
    return ADC1BUF0; // return the a/d results
} // end read_ADC

// function to get encoder data for the specified encoder
// takes pointer to position data strutcure

void readEncoder(enc_struct* pos_data) {
    // Capture the current count from position register
    pos_data->posn_count = *(pos_data->POSCOUNT)/2;

    // calc the difference between samples
    pos_data->posn_delta = pos_data->posn_count - pos_data->posn_countold;
    
    // calculate velocity with moving average
//    pos_data->velocity = (long int)(
//            (pos_data->velocity_old * (1.0 - pos_data->ewma_alpha))
//            +(pos_data->ewma_alpha*(pos_data->posn_delta/(SAMP_TIME*0.001))));
//
//    pos_data->accel = (long int)(pos_data->velocity - pos_data->velocity_old)
//            / (SAMP_TIME * 0.001);
//
//    pos_data->velocity_old = pos_data->velocity;

    // Detect and correct for zero crossing, will be apparent with large deltas.
    // If the difference is less than -50 (bigger than expected), then correct
    if (pos_data->posn_delta < -50) {
        pos_data->posn = pos_data->posn_old + pos_data->posn_delta + ONE_REV;
    }// If delta is greater than 50 (bigger than expected), correct it
    else if (pos_data->posn_delta > 50) {
        pos_data->posn = pos_data->posn_old + pos_data->posn_delta - ONE_REV;
    }// if delta is small, than just update position normally with delta.
    else {
        pos_data->posn = pos_data->posn_old + pos_data->posn_delta;
    }

    // update old values.
    pos_data->posn_countold = pos_data->posn_count;
    pos_data->posn_old = pos_data->posn;

}

// update the pwm duty cycle on desired pwm output
// takes pointer to pwm data structure

//void doPWM(pwm_struct* pwm) {
//
//    if (pwm->output < 0) {
//        if (pwm->output < -PWM_PERIOD)
//            pwm->output = -PWM_PERIOD + 1; // do not exceed 100% Duty
//        pwm->duty = pwm->enable * (-pwm->output + pwm->offset); // convert sign and change direction
//        // set direction pins for FWD direction
//        pwm->DIR1_state = 1;
//        pwm->DIR2_state = 0;
//
//    } else {
//        if (pwm->output > PWM_PERIOD)
//            pwm->output = PWM_PERIOD - 1; // do not exceed 100% Duty
//        pwm->duty = pwm->enable * (pwm->output + pwm->offset); // output duty cyle
//        // set direction pins for REV direction
//        pwm->DIR1_state = 0;
//        pwm->DIR2_state = 1;
//    }
//          if(pwm->duty > PWM_PERIOD)
//            pwm->duty = PWM_PERIOD;
//    // set the direction pins
//    // create a mask with the pwm direction pins cleared
//    unsigned int tempLATCH = *(pwm->LATCH) & ~(pwm->DIR1_pin | pwm->DIR2_pin);
//    // set latch based on desired direction pin states
//    *(pwm->LATCH) = tempLATCH | pwm->DIR1_state*pwm->DIR1_pin
//            | pwm->DIR2_state*pwm->DIR2_pin; //
//}

// generate reference signal for square wave
long int squareRef(int magnitude, int period) {
    static int direction = 1; // shared variable among all calls to this function
    // swap direction every half period
    if (run_time % (period / 2) == 0)
        direction = -direction;
    return (long int)magnitude*CTS_PER_DEG*direction;
}

// function to update the digital output states of the led pins
// takes pointer to LED structure
void updateLEDs(LED_struct* LED){
  // read contents of LATB and create mask with led pins cleared
  unsigned int tempLATCH = *(LED->LATCH) & ~(GRN_LED | RED_LED);
  // set LED pins correctly on LATB
  *(LED->LATCH) = tempLATCH | *(LED->grn)*GRN_LED | *(LED->red)*RED_LED;
}

//send a step to a stepper motor driver
void step(motor_struct* motor, int direction){
  int i;
  unsigned int tempLATCH = *(motor->LATCH) & ~(motor->DIR_PIN | motor->STEP_PIN);
  *(motor->LATCH) = tempLATCH | motor->DIR_PIN*direction | motor->STEP_PIN;
  for(i = 0; i < 10; i++); // wait for pulse to be raised
  *(motor->LATCH) = tempLATCH | motor->DIR_PIN*direction; // remove the pulse and disable;
}

// enable or disable stepper motor
void motor_enable(motor_struct* motor, int enable){
  unsigned int tempLATCH = *(motor->LATCH) & ~(motor->ENABLE_PIN);
  *(motor->LATCH) = tempLATCH | (motor->ENABLE_PIN*!enable);
  int i;
  for(i = 0; i < 10; i++); // wait for pulse to be raised
}

// run a first order digital filter on data
long int filter(gradient_data_struct* data, long int new_value){
  // output = k*(input) + (1-k)*last_output
  data->filtered_value = (new_value*data->k1 + data->filtered_value*data->k2)/1000; //
  data->current_value = new_value; // keep a copy of the current raw value
  return data->filtered_value; // return the filtered value
}

/*toggle pause state and do one-time tasks to enter paused state*/
void pause_toggle(int* pause_flag){
  static long unsigned int time = 0;
  *pause_flag = !(*pause_flag);
  if(*pause_flag){ // system is paused
    serial_bufWrite("<m>System pausing</m>\n", -1);
    time = run_time;
  }
  else{ // system is unpaused
    serial_bufWrite("<m>System resuming</m>\n", -1);
    run_time = time;
  }
//  motorX.pwm.enable = !(*pause_flag);  // toggle enable pwm1 (off when paused)
//  motorY.pwm.enable = !(*pause_flag);
  testing = 0;
}

/* check input buffer for new serial input*/
void pollForSerial(command_struct* command) {
  static char commBuffer[BUFFER_SIZE];
  if (serial_gotNewline()){
    int inputLength = serial_available();
    serial_read(commBuffer, inputLength);
    parseInputString(command, commBuffer);
  }
}

/*takes a string and parses its three arguments */
void parseInputString(command_struct* command, char * string){
  // put three arguments into command structure
  sscanf(string, "%s %s %s", command->arg0, command->arg1, command->arg2);
  doCommand(command);
}

// sets up data structure for data posting
void initPostData(post_data* data, char* headings[], char* units[], int numCols){
  data->printHeader = 1;
  data->numColumns = numCols;
  data->columnLabels = headings;
  data->unitsLabels = units;
}

// write column headings to serial port
void postHeadings(post_data* data){
  serial_bufWrite("<h>", 3);
  int i;
  // print each column heading
  for (i = 0; i < data->numColumns; i++){
    serial_bufWrite(data->columnLabels[i], strlen(data->columnLabels[i]));
    if(i < data->numColumns - 1)
      serial_bufWrite("\t", 1);
  }
  serial_bufWrite("</h>\n", 5);
  postUnits(data);
  printHeader = 0; // reset flag
}

// send column units string over serial (wrapped in <u></u> tags)
void postUnits(post_data* data){
  serial_bufWrite("<u>", 3);
  int i;
  // print each column heading
  for (i = 0; i < data->numColumns; i++){
    serial_bufWrite(data->unitsLabels[i], strlen(data->unitsLabels[i]));
    if(i < data->numColumns - 1)
      serial_bufWrite("\t", 1);
  }
  serial_bufWrite("</u>\n", 5);
}

// print a row of data to serial port
void postRowData(post_data* data){
  // check if header should be printed
  if (printHeader)
    postHeadings(data);
  else{
    serial_bufWrite("<r>", 3);
    int i;
    // print each column heading
    for (i = 0; i < data->numColumns; i++){
      serial_bufWrite(data->dataRow[i], -1);
      if(i < data->numColumns - 1)
        serial_bufWrite("\t", 1);
    }
    serial_bufWrite("</r>\n", 5);
  }
}

// initialize a ramp struct with starting time, position and velocity
void rampInit(ramp_struct* ramp, long unsigned int startTime,
        long int startPosition, double cts_per_ms){
  ramp->startTime = startTime;
  ramp->startPos = startPosition;
  ramp->cts_per_ms = cts_per_ms;
}

// ramp signal
long int doRamp(ramp_struct* ramp, long unsigned int time){
  return ramp->startPos + (long int)((time - ramp->startTime)*ramp->cts_per_ms);
}

void setCoil(int pwmDuty){
  coilPWM = pwmDuty;
}

// serial commands are all listed here
void doCommand(command_struct* command){
  // command: p   -toggle pause state
  if(!strncmp(command->arg0, "pause", 5)){ // see if arg0 is "pause"
    pause_toggle(&pause_all);
  }
//  else if(!strncmp(command->arg0, "print", 5)){
//    if(!strncmp(command->arg1, "header", 6)){
//      printHeader = 1;
//    }
//    else cmdUnknown(command);
//  }
  else if(!strncmp(command->arg0, "go", 2)){
    sys_state = SYS_GO;
    serial_bufWrite("<m>Beginning course.</m>\n", -2);
  }
  else if(!strncmp(command->arg0, "reset", 5)){
    sys_state = SYS_RESET;
    serial_bufWrite("<m>Entering reset mode.</m>\n", -1);
  }
  // test mode allows the system to accept command line setpoints
  else if(!strncmp(command->arg0, "test", 4)){
    if(!pause_all){
      serial_bufWrite("<m>System must be paused to do testing.</m>\n", -1);
    }
    else if(!strncmp(command->arg1, "on", 2)){
      testing = 1;
      serial_bufWrite("<m>Entering test mode. Type 'test off' to exit.</m>\n", -1);
    }
    else if(!strncmp(command->arg1, "off", 4)){
      testing = 0;
//      motorX.pwm.enable = 0;
//      motorY.pwm.enable = 0;
      serial_bufWrite("<m>Exiting test mode.</m>\n", -1);
    }
    else if(!strncmp(command->arg1, "angle", 5)){
      // set angular position in degrees
      if(testing == 1){
//        motorX.pwm.enable = 1;
//        motorX.PID_pos->referenceSig =
                (long int)(atoi(command->arg2)*motorX.enc.cts_per_unit);
      }
      else{
        serial_bufWrite("<m>Test mode is off. Type 'test on' to enter test mode.</m>\n",-1);
      }
    }
    else if(!strncmp(command->arg1, "height", 6)){
      if(testing == 1){
//        motorY.pwm.enable = 1;
//        motorY.PID_pos->referenceSig =
                (long int)(atoi(command->arg2)*motorY.enc.cts_per_unit);
      }
    }
    else cmdUnknown(command);

  }
//  else if(!strncmp(command->arg0, "get", 3)){
//    if(!strncmp(command->arg1, "params", 6)){
//      snprintf(&toPrint2[0], BUFFER_SIZE,
//              "<m>PID parameters:\nkp1: %.2f\tkd1: %.2f\tki1: %.2f\toffset1: %d"
//              "\nkp2: %.2f\tkd2: %.2f\tki2: %.2f\toffset2: %d</m>\n",
//              (double)motorX.PID_pos->kp, (double)motorX.PID_pos->kd,
//              (double)motorX.PID_pos->ki, motorX.pwm.offset,
//              (double)motorY.PID_pos->kp, (double)motorY.PID_pos->kd,
//              (double)motorY.PID_pos->ki, motorY.pwm.offset);
//      serial_bufWrite(toPrint2, -1);
//    }
//  }
    else if(!strncmp(command->arg0, "set", 3)){
//    if(!strncmp(command->arg1, "kp1", 3)){
//      PID_setCoeffs(motorX.PID_pos, atof(command->arg2), motorX.PID_pos->kd, motorX.PID_pos->ki);
//    }
//    else if(!strncmp(command->arg1, "kd1", 3)){
//      PID_setCoeffs(motorX.PID_pos, motorX.PID_pos->kp,atof(command->arg2), motorX.PID_pos->ki);
//    }
//    else if(!strncmp(command->arg1, "ki1", 3)){
//      PID_setCoeffs(motorX.PID_pos, motorX.PID_pos->kp, motorX.PID_pos->kd, atof(command->arg2));
//    }
//    else if(!strncmp(command->arg1, "kp2", 3)){
//      PID_setCoeffs(motorY.PID_pos, atof(command->arg2), motorY.PID_pos->kd, motorY.PID_pos->ki);
//    }
//    else if(!strncmp(command->arg1, "kd2", 3)){
//      PID_setCoeffs(motorY.PID_pos, motorY.PID_pos->kp,atof(command->arg2), motorY.PID_pos->ki);
//    }
//    else if(!strncmp(command->arg1, "ki2", 3)){
//      PID_setCoeffs(motorY.PID_pos, motorY.PID_pos->kp, motorY.PID_pos->kd, atof(command->arg2));
//    }
    if(!strncmp(command->arg1, "sampletime", 10)){
      // set sample period in milliseconds
      sample_time = atoi(command->arg2);
    }
    else if(!strncmp(command->arg1, "postrate", 8)){
      // change data posting period in milliseconds
      post_period = atoi(command->arg2);
      serial_bufWrite("<m>Posting period set to ", -1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", -1);
    }
    else if(!strncmp(command->arg1, "blinkrate", 9)){
      // change data posting period in milliseconds
      blink_period = atoi(command->arg2);
      serial_bufWrite("<m>Blink period set to ", -1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", -1);
    }
    else if(!strncmp(command->arg1, "steprate", 9)){
      step_period = atoi(command->arg2);
      serial_bufWrite("<m>Step period set to ", -1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", -1);

    }
    else if(!strncmp(command->arg1, "x", 1)){
      x_target = atoi(command->arg2);
    }
    else if(!strncmp(command->arg1, "y", 1)){
      y_target = atoi(command->arg2);
    }
    else if(!strncmp(command->arg1, "alpha", 1)){
//      int i;
//      for (i = 0; i < 4; i++){
//        quadrant[i].k1 = atoi(command->arg2);
//        quadrant[i].k2 = 1000-atoi(command->arg2);
//      }
    }
//    else if(!strncmp(command->arg1, "offset1", 7)){
//      motorX.pwm.offset = atoi(command->arg2);
//    }
//    else if(!strncmp(command->arg1, "offset2", 7)){
//      motorY.pwm.offset = atoi(command->arg2);
//    }
//    else if(!strncmp(command->arg1, "vcoil", 5)){
//    // actuate voice coil
//      setCoil(atoi(command->arg2));
//    }
//    else if(!strncmp(command->arg1, "ctsdeg", 6)){
//      motorX.enc.cts_per_unit = atof(command->arg2);
//    }
    else cmdUnknown(command);
  }
  else if(!strncmp(command->arg0, "stop", 4)){
    if(!strncmp(command->arg1, "posting", 7)){
      postflag = 0;
      serial_bufWrite("<m>Posting stopped. Type 'start posting' to resume.</m>\n", -1);
    }
    else if(!strncmp(command->arg1, "motors", 4)){
      motor_enable(&motorX, 0);
      motor_enable(&motorY, 0);
    }
    else cmdUnknown(command);

  }
  else if(!strncmp(command->arg0, "start", 5)){
    if(!strncmp(command->arg1, "posting", 7)){
      postflag = 1;
    }
  }
//  else if(!strncmp(command->arg0, "rampr", 5)){
//    if(testing || sys_state == SYS_GO){
//    // initialize ramp assuming user input is in degrees/sec vel and ms duration
//      rampInit(&testRampRot, run_time, motorX.enc.posn,
//              (double)(atoi(command->arg1))*motorX.enc.cts_per_unit*.001);
//      // set ramp timer to expire based on user input
//      ramp_timer_rot = (long unsigned int)(atoi(command->arg2)); // duration of ramp
//      snprintf(toPrint2,BUFFER_SIZE, "<m>Rampr\ncts/ms: %f\tduration: %d ms\t </m>\n",
//              testRampRot.cts_per_ms, ramp_timer_rot);
//      serial_bufWrite(toPrint2, -1);
//    }
//  }
//  else if(!strncmp(command->arg0, "rampc", 5)){
//    if(testing || sys_state == SYS_GO){
//    // initialize ramp assuming user input is in degrees/sec vel and ms duration
//      rampInit(&testRampCab, run_time, motorY.enc.posn,
//              (double)(atoi(command->arg1))*motorY.enc.cts_per_unit*.001);
//      // set ramp timer to expire based on user input
//      ramp_timer_cab = (long unsigned int)(atoi(command->arg2)); // duration of ramp
//      snprintf(toPrint2,BUFFER_SIZE, "<m>Rampc\ncts/ms: %f\tduration: %d ms\t </m>\n",
//              testRampCab.cts_per_ms, ramp_timer_cab);
//      serial_bufWrite(toPrint2, -1);
//    }
//
//    else{
//      serial_bufWrite("<m>Use: rampr [slope(deg/s)] [duration (ms)]\n"
//              "Testing must be enabled.</m>\n", -1);
//    }
//  }
  else if(!strncmp(command->arg0, "xp", 2)){ // step commands
    motor_enable(&motorX, 1);
    x_target += atoi(command->arg1);
  }
  else if(!strncmp(command->arg0, "xm", 2)){ // step commands
    motor_enable(&motorX, 1);
    x_target -= atoi(command->arg1);
  }
  else{
    cmdUnknown(command);
  }
}

// print error message to serial
void cmdUnknown(command_struct* command){
      serial_bufWrite("<m>Command unknown: ", -1);
      serial_bufWrite(command->arg0, -1);
      serial_bufWrite(" ", 1);
      serial_bufWrite(command->arg1, -1);
      serial_bufWrite(" ", 1);
      serial_bufWrite(command->arg2, -1);
      serial_bufWrite("</m>\n", 5);
}
