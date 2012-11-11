/******************************************************************
Name: main.c
Authors: Frank Sup, Jacob Miller-Mack, Andrew Erwin
Date: 1/28/2011, updated 5/1/12
Comments: MIE-597G Lab 3: Sensors

INCLUDE FILES: p33FJ64MC202.h, p33FJ64MC202.gld (Linker Script),
functions.c, support.h
 ******************************************************************/

#include "PID.h"

#include "p33FJ64MC202.h" // Include p33FJ64MC202 header file
#include "math.h"         // Include math libary
#include "support.h"      // Include Defitions and function prototypes
#include "PIC_serial.h"
#include "PID.h"          // Include custom PID controller library
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// global variable declarations
#define BLINK_PERIOD 250// ms between blinking of light
#define POST_PERIOD 41 // ms between serial data postings
#define RAMP_PERIOD 50  // ms between position control updates
#define STEP_PERIOD 4   // ms between stepper motor pulses

long unsigned int run_time = 0; // 1 ms increments, resets at 49.7 days
int sample_time = SAMP_PERIOD;
int wait_flag = 0; // signal for end of sample time
int sys_state = SYS_GO; // set default system state to reset
int pause_all = 1; // pause system on startup
int testing = 0; // flag to take set points from command line
int LED_BLINK = 0; // blinker oscillates at blink_rate ms
int LED_OFF = 0;
int LED_ON = 1;
int blink_period = 250;
int post_period = POST_PERIOD; // ms between each post (minus 16 for an unknown reason)
int AD_period = 10; // ms between data collection
int step_period = STEP_PERIOD;
long unsigned int AD_clock = 0;
long unsigned int blink_clock = 0; // ms blink rate
long unsigned int post_clock = 0; // next time to post serial data
int postflag = 1; // set if we want to post row data to serial
int ramp_timer_rot;
int ramp_timer_cab;
int printHeader = 1; // flag to print header next post or not
char toprint[BUFFER_SIZE];
int coilPWM; // pwm duty of voice coilchar toprint[BUFFER_SIZE];

int analogData[6]; // storage for current A/D values on all channels
int AD_channel;    // index to read from a different A/D channel each loop

// initialize structure holding LED pin and state info
LED_struct LED = {&LED_LATCH, &LED_OFF, &LED_OFF};

controller_data PID_RotorPos; // rotary motor PID position control
controller_data PID_RotorAcc; // rotary motor PID accel control
controller_data PID_CablePos; // spindle motor PID position control
motor_struct motorX, motorY; // structs for motor pwm and encoder data

command_struct command; // struct to hold incoming command data
post_data data; // row of data for serial posting
ramp_struct testRampRot; // ramp signal for testing
ramp_struct testRampCab;

char toPrint[128]; // temporary storage for serial output

// stepper destinations
int x_target = 0;
int y_target = 0;

int x_pos = 0;
int y_pos = 0;

// stepper maximum travel +/-
int x_max = 10000;
int y_max = 10000;

long unsigned int step_clock = 0;

// constants for rotary position PID.
//float kp_pos1 = 250; // proportional constant for motor1
//float kd_pos1 = 3; // derivative constant
//float ki_pos1 = 0; // integral constant
//
//// constants for cable position PID
//float kp_pos2 = 100; // proportional constant for motor1
//float kd_pos2 = 2; // derivative constant
//float ki_pos2 = 0; // integral constant

/////////////////////////////////////////////////////////////////
// Start Main Function

int main() {
  //main local variables

  //Initialization Routines
  init_clock(); // initialize Clock 80 MHz/40 MIPS
  init_pins(); // initialize pins: LEDs, pot a/d, encoder, pushbutton
  init_samptime(); // initialize sample time (1ms)
  init_ADC(); // initialize A/D converter
  init_encoders(&(motorX.enc), &(motorY.enc)); // initialize quadrature encoders

//  // assign PID regimes to motor objects
//  motorX.PID_pos = &PID_RotorPos;
//  motorY.PID_pos = &PID_CablePos;

  gradient_data_struct quadrant[4];

  // set low pass digital filter parameters for quadrant data
  int i = 0;
  for(i = 0; i < 4; i++){
    quadrant[i].k1 = EWMA_CONSTANT;
    quadrant[i].k2 = 100-EWMA_CONSTANT;
  }

  // conversion factors and stepper motor driver pins
  motorX.enc.cts_per_unit = CTS_PER_MM; // encoder counts per mm
  motorX.LATCH = &MOTX_LATCH;
  motorX.DIR_PIN = MOTX_DIR;
  motorX.STEP_PIN = MOTX_STEP;
  motorX.ENABLE_PIN = MOTX_EN;
  motor_enable(&motorX, 0); // disable motorX

  motorY.enc.cts_per_unit = CTS_PER_MM; // encoder counts per mm
  motorY.LATCH = &MOTY_LATCH;
  motorY.DIR_PIN = MOTY_DIR;
  motorY.STEP_PIN = MOTY_STEP;
  motorY.ENABLE_PIN = MOTY_EN;
  motor_enable(&motorY, 0); // disable motorY

//  init_pwm1(&(motorX.pwm));
//  init_pwm2(&(motorY.pwm));
//
//  // initialize PID controller for motor1. see PID.h for function details
//  PID_init(&PID_RotorPos, &(motorX.enc.posn), kp_pos1, kd_pos1, ki_pos1, sample_time,
//          (long int) (PWM_PERIOD), (long int) (-PWM_PERIOD));
//
//  PID_RotorPos.max_ref = (long int) (360 * motorX.enc.cts_per_unit);
//  PID_RotorPos.min_ref = (long int) (-360 * motorX.enc.cts_per_unit);

//  // cable position PID initialization
//  PID_init(&PID_CablePos, &(motorY.enc.posn), kp_pos2, kd_pos2, ki_pos2, sample_time,
//          (long int) (PWM_PERIOD), (long int) (-PWM_PERIOD));
//  PID_CablePos.max_ref = (long int) (450 * motorY.enc.cts_per_unit); // 450mm max
//  PID_CablePos.min_ref = (long int) (-450 * motorY.enc.cts_per_unit);

  // for posting data to serial port
  serial_begin(BAUDRATE); // initiatize serial connection at 19200 baud
  char * headings[] = {"Time", "X-Step", "X-Enc", "QuadA", "QuadB", "QuadC",
    "QuadD"};
  // "Temp"};
  //"QuadA_filt", "QuadB_filt", "QuadC_filt", "QuadD_filt", "Temp_filt"};
  char* units[] = {"seconds", "in", "in", "cts", "cts", "cts", "cts"};
  //"cts"};
    //"cts", "cts", "cts", "cts", "cts"};
  initPostData(&data, headings, units, NUM_COLUMNS);
  // NUM_COLUMNS is defined in support.h

  run_time = 0; // reset runtime clock

  // initialize command indices
//  int cabCmdIdx = 0;
//  int rotCmdIdx = 0;
//  int coilCmdIdx = 0;
//
//  int numCabCmds = 0;
  // array of cable ramp commands
//  timed_command cabCommands[] = {
//    {.time = 500, .command.arg0 = "rampc", .command.arg1 = "50", .command.arg2 = "1900"},
//    {.time = 14500, .command.arg0 = "rampc", .command.arg1 = "100", .command.arg2 = "2000"},
//    {.time = 16550, .command.arg0 = "rampc", .command.arg1 = "100", .command.arg2 = "600"},
//    {.time = 18150, .command.arg0 = "rampc", .command.arg1 = "130", .command.arg2 = "700"},
//    {.time = 19150, .command.arg0 = "rampc", .command.arg1 = "-100", .command.arg2 = "190"},
//    {.time = 22750, .command.arg0 = "rampc", .command.arg1 = "-100", .command.arg2 = "4000"},
//    {.time = 33500, .command.arg0 = "rampc", .command.arg1 = "100", .command.arg2 = "2600"},
//    {.time = 37850, .command.arg0 = "rampc", .command.arg1 = "-100", .command.arg2 = "2400"},
//    {.time = 44000, .command.arg0 = "rampc", .command.arg1 = "-100", .command.arg2 = "600"},
//  };
//
//  int numRotCmds = 0;
//  // rotary commands
//  timed_command rotCommands[] = {
//    {.time = 500, .command.arg0 = "rampr", .command.arg1 = "5", .command.arg2 = "1500"},
//    {.time = 2000, .command.arg0 = "rampr", .command.arg1 = "5", .command.arg2 = "12500"},
//    {.time = 16500, .command.arg0 = "rampr", .command.arg1 = "80", .command.arg2 = "900"},
//    {.time = 17400, .command.arg0 = "rampr", .command.arg1 = "40", .command.arg2 = "250"},
//    {.time = 17650, .command.arg0 = "rampr", .command.arg1 = "20", .command.arg2 = "500"},
//    {.time = 23850, .command.arg0 = "rampr", .command.arg1 = "10", .command.arg2 = "14100"},
//    {.time = 38950, .command.arg0 = "rampr", .command.arg1 = "10", .command.arg2 = "6000"},
//  };
//
//  int numCoilCmds = 0;
  // voice coil duty cycle commands
//  timed_command coilCommands[] = {
//    {.time = 0, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "10000"},
//    {.time = 500, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "8000"},
//    {.time = 16250, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "10000"},
//    {.time = 20000, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "8000"},
//    {.time = 30900, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "8000"},
//    {.time = 32000, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "10000"},
//    {.time = 37850, .command.arg0 = "set", .command.arg1 = "vcoil", .command.arg2 = "0"}
//  };
  
  // Start of main loop (1 msec sample period)
  while (1) // infinite loop

  {
    if(serial_remaining())
      serial_ping(); // write any unsent serial

    while (!wait_flag); // wait for the next sample period to begin

    /***** begin sample period tasks *****/

    // determine button state
    switch (btnDebounce()) {
      case BTN_LONG: // enter reset mode if long press has been detected
        sys_state = SYS_RESET;
        if (!pause_all)
          pause_toggle(&pause_all);
        break;
      case BTN_SHORT: // toggle pause_all flag
        pause_toggle(&pause_all);
        break;
      default:
        break;
    }

    // sample one of the A/D Channels this period
    if (AD_clock <= run_time || AD_channel > 0){
      //quadrant[AD_channel].current_value =read_ADC(AD_channel);
      filter(&quadrant[AD_channel], read_ADC(AD_channel));
      if(AD_channel == 5){
        AD_channel = 0;
        AD_clock = run_time + AD_period;
      }
      else
        AD_channel++;

    }

    // gather position data from motor encoders
    readEncoder(&(motorX.enc));
    readEncoder(&(motorY.enc));

    // give a pulse to the stepper driver if it's time
    if (step_clock <= run_time){
      if(x_target == x_pos || abs(x_target) > x_max)
        motor_enable(&motorX, 0); // disable motorX
      else{
        motor_enable(&motorX, 1); // enable motorX
        if(x_target > x_pos){
          step(&motorX, 1);
          x_pos++;
          motorX.enc.posn++;
        }
        else if(x_target < x_pos){
          step(&motorX, 0);
          x_pos--;
          motorX.enc.posn--;
        }
        //motor_enable(&motorX,0); // disable motorX
      }
      step_clock = run_time + step_period;
    }

    // light up red LED with state of Hall effect sensor
    //    int hall_state = HALL_SENSE;
    //    LED.red = &hall_state;

    // only proceed if the system is not paused
    if (!pause_all) {
      // set green light on
      LED.grn = &LED_ON;
      // use pot to adjust pwm1 output

      if (sys_state == SYS_GO) {

//        // enable the motors
//        motorX.pwm.enable = 1;
//        motorY.pwm.enable = 1;
//
//        // make sure we don't run off the end of the list of voice coil commands
//        if (coilCmdIdx < numCoilCmds) {
//          if (coilCommands[coilCmdIdx].time <= run_time && !coilCommands[coilCmdIdx].done) {
//            doCommand(&(coilCommands[coilCmdIdx].command)); // set up the command
//            coilCommands[coilCmdIdx].done = 1; // flag it as done
//            coilCmdIdx++; // increment to next command index
//          }
//        }
//
//        // set up next cable command if it's time
//        if (cabCmdIdx < numCabCmds) {
//          if (cabCommands[cabCmdIdx].time <= run_time && !cabCommands[cabCmdIdx].done) {
//            doCommand(&(cabCommands[cabCmdIdx].command));
//            cabCommands[cabCmdIdx].done = 1;
//            cabCmdIdx++;
//          }
//        }
//
//        // set up next rotary command if it's time
//        if (rotCmdIdx < numRotCmds) {
//          if (rotCommands[rotCmdIdx].time <= run_time && !rotCommands[rotCmdIdx].done) {
//            doCommand(&(rotCommands[rotCmdIdx].command));
//            rotCommands[rotCmdIdx].done = 1;
//            rotCmdIdx++;
//          }
//        }
//
//        // check if it's time to update the reference signals
//        if (ramp_timer_rot > 0) { // run ramp if enabled
//          motorX.PID_pos->referenceSig = doRamp(&testRampRot, run_time);
//          ramp_timer_rot = ramp_timer_rot - sample_time; // decrement ramp timer
//        }
//
//        if (ramp_timer_cab > 0) { // run ramp if enabled
//          motorY.PID_pos->referenceSig = doRamp(&testRampCab, run_time);
//          ramp_timer_cab = ramp_timer_cab - sample_time; // decrement ramp timer
//        }
//
//        // set the pwm output for each motor based on controller output
//        motorX.pwm.output = PID_run(motorX.PID_pos);
//        motorY.pwm.output = PID_run(motorY.PID_pos);


      }
      else if (sys_state == SYS_RESET) { // system is in reset
        // not implemented
      }
    } else { // system is paused
      // set green led to blink
      LED.grn = &LED_BLINK;

      if (testing) {
//        // get the current system state
//        // getCurrentState(&(motRotor.enc), &(motCable.enc), &current_state);
//        motorX.pwm.enable = 1;
//        motorY.pwm.enable = 1;
//
//        if (ramp_timer_rot > 0) { // run ramp if enabled
//          motorX.PID_pos->referenceSig = doRamp(&testRampRot, run_time);
//          ramp_timer_rot = ramp_timer_rot - sample_time; // decrement ramp timer
//        }
//
//        if (ramp_timer_cab > 0) { // run ramp if enabled
//          motorY.PID_pos->referenceSig = doRamp(&testRampCab, run_time);
//          ramp_timer_cab = ramp_timer_cab - sample_time; // decrement ramp timer
//        }
//
//        motorX.pwm.output = PID_run(motorX.PID_pos);
//        motorY.pwm.output = PID_run(motorY.PID_pos);
      }
      //    LED.red = &LED_BLINK;
    }

    // tasks below run each loop cycle

//    doPWM(&(motorX.pwm));
//    doPWM(&(motorY.pwm));

    // toggle LED blinker at pre-set oscillation rate
    if (blink_clock <= run_time) {
      blink_clock = run_time + blink_period;
      LED_BLINK = !LED_BLINK;
    }

    // update LED pin states
    updateLEDs(&LED);

    //  post the current row of data to the serial port
    if (postflag && post_clock <= run_time) {
      snprintf(data.dataRow[0], 9, "%lu.%03lu", run_time / 1000, run_time % 1000); // run time seconds
      snprintf(data.dataRow[1], 9, "%c0.%04d", (x_pos < 0 ? '-' : '0'), abs(x_pos)%10000);
              //(double) (x_pos/10000.0)); // x position mm
      snprintf(data.dataRow[2], 9, "%ld", (motorX.enc.posn)); // x position from encoder
//      snprintf(data.dataRow[2], 9, "%.4f",
//              (double) (y_pos/10000.0)); // y position mm
      snprintf(data.dataRow[3], 9, "%d", quadrant[THERM1_CHANNEL].current_value);
      snprintf(data.dataRow[4], 9, "%d", quadrant[THERM2_CHANNEL].current_value);
      snprintf(data.dataRow[5], 9, "%d", quadrant[THERM3_CHANNEL].current_value);
      snprintf(data.dataRow[6], 9, "%d", quadrant[THERM4_CHANNEL].current_value);
//      snprintf(data.dataRow[7], 9, "%d", quadrant[TEMP_CHANNEL].current_value);
//
//      snprintf(data.dataRow[8], 9, "%d.%d", quadrant[THERM1_CHANNEL].filtered_value/10,abs(quadrant[THERM1_CHANNEL].filtered_value%10));
//      snprintf(data.dataRow[9], 9, "%d.%d", quadrant[THERM2_CHANNEL].filtered_value/10,abs(quadrant[THERM2_CHANNEL].filtered_value%10));
//      snprintf(data.dataRow[10], 9, "%d.%d", quadrant[THERM3_CHANNEL].filtered_value/10,abs(quadrant[THERM3_CHANNEL].filtered_value%10));
//      snprintf(data.dataRow[11], 9, "%d.%d", quadrant[THERM4_CHANNEL].filtered_value/10,abs(quadrant[THERM4_CHANNEL].filtered_value%10));
//      snprintf(data.dataRow[12], 9, "%d.%d", quadrant[TEMP_CHANNEL].filtered_value/10,abs(quadrant[TEMP_CHANNEL].filtered_value%10));
//
      postRowData(&data);
      post_clock = run_time + post_period; // reset posting clock
    }
    //check serial buffer for incoming command
    pollForSerial(&command);

    // housekeeping to run at end of sample period
    wait_flag = 0; // reset wait flag
    /***** end sample period tasks *****/
  } // end infinite loop

  return 0;

} // end main
