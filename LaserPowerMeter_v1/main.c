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
int blink_period = BLINK_PERIOD;
int post_period = POST_PERIOD; // ms between each post (minus 16 for an unknown reason)
int AD_period = AD_PERIOD; // ms between data collection
int step_period = STEP_PERIOD;
long unsigned int AD_clock = 0;
long unsigned int blink_clock = 0; // ms blink rate
long unsigned int post_clock = 0; // next time to post serial data
int postflag = 1; // set if we want to post row data to serial
int use_simple_filter = 0; // set to zero for butterworth digital filter

int printHeader = 1; // flag to print header next post or not
char toprint[BUFFER_SIZE];

int analogData[6]; // storage for current A/D values on all channels
int AD_channel; // index to read from a different A/D channel each loop

// initialize structure holding LED pin and state info
LED_struct LED = {&LED_LATCH, &LED_OFF, &LED_OFF};

motor_struct motorX, motorY; // structs for motor pwm and encoder data

command_struct command; // struct to hold incoming command data
post_data data; // row of data for serial posting

// stepper maximum travel +/-
int max_travel = 10000;

// structure array to store data from each of the thermopiles
gradient_data_struct quadrant[4];

// third order butterworth filter with cutoff freq of 0.02 * Fs
// these values are all multiplied by 2^17
digital_filter butter3 = {.filter_order = 3,
                          .output_coeffs = {131072,	-376748,	361298,	-115591},
                          .input_coeffs = {4,	11,	11,	4}
                         };

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
    init_gradientData(&quadrant[i], &butter3);
  }

  // conversion factors and stepper motor driver pins
  motorX.enc.cts_per_unit = CTS_PER_MM; // encoder counts per mm
  motorX.LATCH = &MOTX_LATCH;
  motorX.DIR_PIN = MOTX_DIR;
  motorX.STEP_PIN = MOTX_STEP;
  motorX.ENABLE_PIN = MOTX_EN;
  motor_enable(&motorX, 0); // disable motorX
  // stepper destinations
  motorX.target_pos = 0;
  motorX.current_pos = 0;

  motorY.enc.cts_per_unit = CTS_PER_MM; // encoder counts per mm
  motorY.LATCH = &MOTY_LATCH;
  motorY.DIR_PIN = MOTY_DIR;
  motorY.STEP_PIN = MOTY_STEP;
  motorY.ENABLE_PIN = MOTY_EN;
  motor_enable(&motorY, 0); // disable motorY
  motorY.target_pos = 0;
  motorY.current_pos = 0;

  // for posting data to serial port
  serial_begin(BAUDRATE); // initiatize serial connection at 115000 baud
  char * headings[] = {"Time", "X-Step", "X-Enc", "QuadA", "QuadB", "QuadC",
    "QuadD", "Temp"};
  //"QuadA_filt", "QuadB_filt", "QuadC_filt", "QuadD_filt", "Temp_filt"};
  char* units[] = {"seconds", "in", "in", "cts", "cts", "cts", "cts", "cts"};
  //"cts", "cts", "cts", "cts", "cts"};

  // initialize serial output format
  initPostData(&data, headings, units, NUM_COLUMNS);
  // NUM_COLUMNS is defined in support.h

  run_time = 0; // reset runtime clock

  // Start of main loop (1 msec sample period)
  while (1) // infinite loop
 {
    if (serial_remaining())
      serial_ping(); // write any unsent serial data

    // sample one of the A/D Channels this cycle
    if (AD_clock <= run_time || AD_channel > 0) {
      //quadrant[AD_channel].current_value =read_ADC(AD_channel);
      filter(&quadrant[AD_channel], read_ADC(AD_channel));
      if (AD_channel == 5) { // reset to channel 0 after all have been through
        AD_channel = 0;
        AD_clock = run_time + AD_period;
      } else {
        AD_channel++;
        AD_clock = run_time + 1; // wait 1 ms to sample next channel
      }
    }


    if (wait_flag) { // wait for the next sample period to begin

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

      // gather position data from motor encoders
      //readEncoder(&(motorX.enc));
      //readEncoder(&(motorY.enc));

      if (step_clock <= run_time) {
        run_steppers(); // run stepper motors if necessary
        step_clock = run_time + step_period;
      }

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
        snprintf(data.dataRow[1], 9, "%c.%04d", (motorX.current_pos < 0 ? '-' : '0'), abs(motorX.current_pos) % 10000);
        //(double) (x_pos/10000.0)); // x position mm
        snprintf(data.dataRow[2], 9, "%c.%04d", (motorY.current_pos < 0 ? '-' : '0'), abs(motorY.current_pos) % 10000);
        snprintf(data.dataRow[3], 9, "%ld", quadrant[THERM1_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[4], 9, "%ld", quadrant[THERM2_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[5], 9, "%ld", quadrant[THERM3_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[6], 9, "%ld", quadrant[THERM4_CHANNEL].unfiltered_value);
        snprintf(data.dataRow[7], 9, "%ld", quadrant[TEMP_CHANNEL].unfiltered_value);

        //      snprintf(data.dataRow[8], 9, "%d.%d", quadrant[THERM1_CHANNEL].filtered_value/10,abs(quadrant[THERM1_CHANNEL].filtered_value%10));
        //      snprintf(data.dataRow[9], 9, "%d.%d", quadrant[THERM2_CHANNEL].filtered_value/10,abs(quadrant[THERM2_CHANNEL].filtered_value%10));
        //      snprintf(data.dataRow[10], 9, "%d.%d", quadrant[THERM3_CHANNEL].filtered_value/10,abs(quadrant[THERM3_CHANNEL].filtered_value%10));
        //      snprintf(data.dataRow[11], 9, "%d.%d", quadrant[THERM4_CHANNEL].filtered_value/10,abs(quadrant[THERM4_CHANNEL].filtered_value%10));
        //      snprintf(data.dataRow[12], 9, "%d.%d", quadrant[TEMP_CHANNEL].filtered_value/10,abs(quadrant[TEMP_CHANNEL].filtered_value%10));

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
