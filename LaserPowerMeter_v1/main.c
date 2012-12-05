/******************************************************************
Name: main.c
Authors: Jacob Miller-Mack
Date: 11/13/2012
Comments: MIE-415 Senior Design Project: Laser Power Meter

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
#define STEP_PERIOD 3   // ms between stepper motor pulses

long unsigned int run_time = 0; // 1 ms increments, resets at 49.7 days
int sample_time = SAMP_PERIOD;
int wait_flag = 0; // signal for end of sample time
int sys_state = SYS_GO; // set default system state to reset
int pause_all = 1; // pause system on startup
int testing = 0; // flag to take set points from command line
int LED_BLINK = 0; // blinker oscillates at blink_rate ms
int LED_OFF = 0;
int LED_ON = 1;
int blink_period = BLINK_PERIOD;
int post_period = POST_PERIOD; // ms between each post (minus 16 for an unknown reason)
int AD_period = AD_PERIOD; // ms between data collection
int step_period = STEP_PERIOD;
int deriv_period = 500; // ms between determining derivative of signals
long unsigned int deriv_clock = 0;
int pos_control_period = 1000; // ms between determining a new position target
long unsigned int AD_clock = 0;
long unsigned int blink_clock = 0; // ms blink rate
long unsigned int post_clock = 0; // next time to post serial data
long unsigned int pos_control_clock = 0;
long int total_power = 0;
int postflag = 1; // set if we want to post row data to serial
int use_simple_filter = 1; // set to zero for butterworth digital filter
int printHeader = 1; // flag to print header next post or not
char toprint[BUFFER_SIZE];
command_struct deriv_command = {.arg0 = "deriv"};
int AD_channel; // index to read from a different A/D channel each loop

int half_clock = 0; // 500 usec timer


// initialize structure holding LED pin and state info
LED_struct LED = {&LED_LATCH, &LED_OFF, &LED_OFF};

motor_struct motorX, motorY; // structs for motor pwm and encoder data
command_struct command;
command_struct *command_ptr; // struct to hold incoming command data
post_data data; // row of data for serial posting

// stepper maximum travel +/-
int max_travel = 10000;

long int inch_to_mm_scale;
long int mm_to_inch_scale;

int use_mm_pos = 1; // use mm instead of inches for encoders

int need_user_confirm = 0;

// structure array to store data from each of the thermopiles
gradient_data_struct quadrant[5];

// third order butterworth filter with cutoff freq of 0.02 * Fs
// these values are all multiplied by 2^17
digital_filter butter3 = {.filter_order = 3,
  .output_coeffs =
  {131072, -376748, 361298, -115591},
  .input_coeffs =
  {4, 11, 11, 4}};

long unsigned int step_clock = 0; // timer for stepper motor actuation

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

  // set low pass digital filter parameters for quadrant data
  int i = 0;

  for (i = 0; i < 5; i++) {
    // initialize variables
    quadrant[i].filtered_value = 0;
    quadrant[i].unfiltered_value = 0;
    quadrant[i].k1 = EWMA_CONSTANT;
    quadrant[i].k2 = 1000 - EWMA_CONSTANT;
    init_gradientData(&(quadrant[i]), &butter3);
  }
  quadrant[0].cts_at_1kW = QUADA_1KW;
  quadrant[1].cts_at_1kW = QUADB_1KW;
  quadrant[2].cts_at_1kW = QUADC_1KW;
  quadrant[3].cts_at_1kW = QUADD_1KW;

  // conversion factors and stepper motor driver pins
  motorX.enc.cts_per_unit = CTS_PER_MM; // encoder counts per mm
  motorX.LATCH = &MOTX_LATCH;
  motorX.DIR_PIN = MOTX_DIR;
  motorX.STEP_PIN = MOTX_STEP;
  motorX.ENABLE_PIN = MOTX_EN;
  motor_enable(&motorX, 0); // disable motorX
  // stepper destinations
  motorX.target_pos = 0;
  motorX.native_pos = 0;

  motorY.enc.cts_per_unit = CTS_PER_MM; // encoder counts per mm
  motorY.LATCH = &MOTY_LATCH;
  motorY.DIR_PIN = MOTY_DIR;
  motorY.STEP_PIN = MOTY_STEP;
  motorY.ENABLE_PIN = MOTY_EN;
  motor_enable(&motorY, 0); // disable motorY
  motorY.target_pos = 0;
  motorY.native_pos = 0;

  // for posting data to serial port
  serial_begin(BAUDRATE); // initiatize serial connection at 115000 baud
  char * headings[] = {"Time", "X-Step", "Y-Step", "East_filt", "North_filt", "West_filt",
    "South_filt", "Total Power", "Temp","East", "North", "West", "South", "Temp"};
  char* units[] = {"seconds", "in", "in","cts","cts", "cts", "cts",
  "Watts", "Deg C","cts", "cts", "cts", "cts", "cts"};

  inch_to_mm_scale = (long int)((1.0/254.0)*powf(2,13));
  mm_to_inch_scale = (long int)(254.0*powf(2,13))+1;

  if(use_mm_pos){
    units[1] = "mm";
    units[2] = "mm";
    motorX.display_pos = &(motorX.alt_pos);
    motorY.display_pos = &(motorY.alt_pos);
  }
  else{
    motorX.display_pos = &(motorX.native_pos);
    motorY.display_pos = &(motorY.native_pos);    
  }

  // initialize serial output format
  initPostData(&data, headings, units, NUM_COLUMNS);
  // NUM_COLUMNS is defined in support.h

  run_time = 0; // reset runtime clock

  // populate data variables with sensor data
  int j;
  for (i = 0; i < NUM_TAPS; i++) {
    while (run_time < AD_clock); // wait for A/D sample clock to reset
    for (j = 0; j < 5; j++) { // loop through each sensor
      quadrant[j].past_outputs[i].datum =
              quadrant[j].past_inputs[i].datum = (long int) read_ADC(j);
      
      int k; for(k = 0; k < 100; k++); // wait a little between each reading
    }
    AD_clock = run_time + AD_period; // reset A/D sample clock
  }
  
  long unsigned int calib_clock = run_time + CALIB_TIME;

// collect data for offset calibration
  while (run_time < calib_clock) {
    if (AD_clock <= run_time || AD_channel > 0) {
      filter(&quadrant[AD_channel], read_ADC(AD_channel), 0); // don't use offsets
      if (AD_channel == 4) { // reset to channel 0 after all have been through
        AD_channel = 0;
        AD_clock = run_time + AD_period;
      } else {
        AD_channel++;
        AD_clock = run_time + 1; // wait 1 ms to sample next channel
      }
    }
  }

  // determine scaling offsets for four quadrants. this only normalizes the
  // readings at the cold state. power calibration is separate
  //calc_cold_offsets(&quadrant);
  calc_offsets(&quadrant);
  
  for(i = 0; i < 4; i++){
    long int delta_X;;
    if(quadrant[i].cts_at_1kW > 0)
      delta_X = quadrant[i].cts_at_1kW - quadrant[i].offset;
    else
      delta_X  = 1000; // for uncalibrated device
    calc_scale(&(quadrant[i]), 1000, delta_X);
  }

  // temperature calibration. values of RTD_XXX values are defined in support.h
  calc_scale_dbl(&(quadrant[4]), 1000.0/(double)(RTD_100-RTD_0));
  quadrant[4].offset = RTD_0;

  // Start of main loop (1 msec sample period)
  while (1) // infinite loop
  {
    if (serial_remaining())
      serial_ping(); // write any unsent serial data

    // sample one of the A/D Channels this cycle
    if (AD_clock <= run_time || AD_channel > 0) {
      filter(&quadrant[AD_channel], read_ADC(AD_channel), 1);
      if (AD_channel == 4) { // reset to channel 0 after all have been through
        AD_channel = 0;
        AD_clock = run_time + AD_period;
      } else {
        AD_channel++;
        AD_clock = run_time + 4; // wait 4 ms to sample next channel
      }
    }


    if (wait_flag) { // wait for the next sample period to begin
      if(AD_clock <= run_time){
        // sum up total power
        total_power = 0;
        for (i = 0; i < 4; i++){
          total_power += quadrant[i].filtered_value;
        }
      }

      if (deriv_clock <= run_time){
        long int deriv;
        for(i = 0; i < 4; i++){
          deriv = differentiate(&(quadrant[i]));

        }
        //command_ptr = &deriv_command;
        //doCommand(command_ptr);
        deriv_clock = run_time + deriv_period;
      }

      // determine button state
      switch (btnDebounce()) {
        case BTN_LONG: // enter reset mode if long press has been detected
          //sys_state = SYS_RESET;
          if (!pause_all)
            //pause_toggle(&pause_all);
          break;
        case BTN_SHORT: // toggle pause_all flag
          //pause_toggle(&pause_all);
          break;
        default:
          break;
      }

      // gather position data from motor encoders
      //readEncoder(&(motorX.enc));
      //readEncoder(&(motorY.enc));
      if(motorX.direction == 1)
        motorX.native_pos = motorX.target_pos - motorX.step_bin;
      else if(motorX.direction == 0)
        motorX.native_pos = motorX.target_pos + motorX.step_bin;
      if(motorY.direction == 1)
        motorY.native_pos = motorY.target_pos - motorY.step_bin;
      else if(motorY.direction == 0)
        motorY.native_pos = motorY.target_pos + motorY.step_bin;

      motorX.alt_pos = mm_to_inch(motorX.native_pos);
      motorY.alt_pos = mm_to_inch(motorY.native_pos);


//      if (step_clock <= run_time) {
//        run_steppers(); // run stepper motors if necessary
//        step_clock = run_time + step_period;
//      }

      // only proceed if the system is not paused
      if (!pause_all) {
        // set green light on
        LED.grn = &LED_ON;

        if (sys_state == SYS_GO) {

        } else if (sys_state == SYS_RESET) { // system is in reset
          // not implemented
        }
      } else { // system is paused
        // set green led to blink
        LED.grn = &LED_BLINK;
      }

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
        snprintf(data.dataRow[1], 9, "%ld.%1d", *(motorX.display_pos) / 100, abs(*(motorX.display_pos) % 100));
        snprintf(data.dataRow[2], 9, "%ld.%1d", *(motorY.display_pos) / 100, abs(*(motorY.display_pos) % 100));
        snprintf(data.dataRow[3], 9, "%ld", quadrant[THERM1_CHANNEL].filtered_value);
        snprintf(data.dataRow[4], 9, "%ld", quadrant[THERM2_CHANNEL].filtered_value);
        snprintf(data.dataRow[5], 9, "%ld", quadrant[THERM3_CHANNEL].filtered_value);
        snprintf(data.dataRow[6], 9, "%ld", quadrant[THERM4_CHANNEL].filtered_value);
        snprintf(data.dataRow[7], 9, "%ld", total_power);
        snprintf(data.dataRow[8], 9, "%ld.%d", quadrant[TEMP_CHANNEL].filtered_value/10, abs(quadrant[TEMP_CHANNEL].filtered_value%10));
        snprintf(data.dataRow[9], 9, "%ld", quadrant[THERM1_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[10], 9, "%ld", quadrant[THERM2_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[11], 9, "%ld", quadrant[THERM3_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[12], 9, "%ld", quadrant[THERM4_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[13], 9, "%ld", quadrant[TEMP_CHANNEL].unfiltered_value);

        postRowData(&data);
        post_clock = run_time + post_period; // reset posting clock
      }
      //check serial buffer for incoming command
      pollForSerial(&command);

      // housekeeping to run at end of sample period
      wait_flag = 0; // reset wait flag
    }
  } // end infinite loop

  return 0;

} // end main
