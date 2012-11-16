/*******************************************************
Name: support.h
Date: 2/3/2011, updated 5/1/2012
Authors: Jacob Miller-Mack, Frank Sup, Andrew Erwin
Comments:  Support file for definitions and function prototypes
*******************************************************/

//defines to improve code clarity
#ifndef _SUPPORT_H
#define _SUPPORT_H

#include "PID.h"


#define PI 3.141592653589
#define SAMP_PERIOD 2         // main loop time in millisec

#define BAUDRATE 115200     // baud rate for UART serial comm 115200 is good
#define AD_PERIOD 4         // time between a/d samples

#define WAIT i=0;while(i<15)i++;

#define LED_LATCH LATA
#define GRN_LED 1 << 2      //  pin RA2
#define RED_LED 1 << 3      //  pin RA3

//#define SAMPLE_TIME LATAbits.LATA4 // sample time output on RA4

#define BTN_THRESHOLD_LOW 5 // duration of 'quick' button press (ms)
#define BTN_THRESHOLD_HI 500// duration of 'long' button press (ms)
#define BTN_OFF 0           // button in off state
#define BTN_SHORT 1         // short press of button
#define BTN_LONG 2          // long press of button
#define BTN_PORT PORTA      // button is on PORTA
#define BTN_PIN 1 << 4      // button pin, RA4
//#define HALL_SENSE PORTAbits.RA4 // hall effect sensor on pin RA4

#define SYS_RESET 0         // system states
#define SYS_GO 1

#define THERM1_CHANNEL 0    // thermopile quadrant 1 (east), AN0
#define THERM2_CHANNEL 1    // thermopile quadrant 2 (north), AN1
#define THERM3_CHANNEL 2    // thermopile quadrant 3 (west), AN2
#define THERM4_CHANNEL 3    // thermopile quadrant 4 (south), AN3
#define POT_ADC_CHANNEL 4   // potentiometer pin, AN4
#define TEMP_CHANNEL 5      // thermocouple on AN5

#define ENC_MODE 7          // quadrature enc mode (4x, no index pulse reset)
// digital first order filter constant
#define EWMA_CONSTANT 150   // alpha =  0.0245*1000 corresponds to 2 Hz filter

#define TIME_HISTORY 10    // number of past data points to store

#define ONE_REV 400        // number of encoder counts per shaft revolution


#define SPOOL_CIRC 195.5     // circumference of cable spool in mm
#define CTS_PER_MM 3937.0

#define SQR_PERIOD 4000     // square wave period
#define SQR_MAGNITUDE 30    // +/- degrees of square wave oscillation

#define MOTX_LATCH LATB
#define MOTX_DIR 1 << 15      // x axis motor direction pin RB15
#define MOTX_STEP 1 << 14      // x axis motor step pin RB14
#define MOTX_EN 1 << 13

#define MOTY_LATCH LATB
#define MOTY_DIR 1 << 11      // y axis motor direction pin RB11
#define MOTY_STEP 1 << 12      // y axis motor step pin RB12
#define MOTY_EN 1 << 10

#define COLUMN_LABEL_SIZE 16 // max letters of each output column heading
#define ROW_LENGTH 128       // number of letters in an entire row
#define NUM_COLUMNS 8        // number of output columns

#define BUFFER_SIZE 512     // software serial data buffer size

/* interrupt declaration for faster context switching if the interrupt doesn't
 * access any const variables*/
#define _ISRFASTER __attribute__((interrupt, no_auto_psv))

// structure to store pointers to LED states
typedef struct{
    volatile unsigned int* LATCH;
    int* grn; // state of green LED
    int* red; // state of red LED    
} LED_struct;

// data structure for motor position data
typedef struct {
  // pointer to encoder count register (i.e., POS1CNT)
  volatile unsigned int* POSCOUNT;
  long int posn_ref;
  long int posn_count;
  long int posn_delta;
  long int posn_old;
  long int posn_countold;
  long int posn;
//  long int velocity;
//  long int velocity_old;
//  long int accel;
//  long int ewma_alpha;
  double cts_per_unit;  // conversion factor for convenience
} enc_struct;

// data structure for pwm data
typedef struct{ 
  int output;               // signed output of pwm
  int duty;                 // duty counts for pwm pin
  int offset;     // pwm offset to overcome initial motor friction
  int enable;               // enable motor
  volatile unsigned int* LATCH; // pointer to port latch (ie LATB)
  int DIR1_pin; // DIR1 pin number
  int DIR2_pin; // DIR2 pin number
  int DIR1_state; // state of FWD direction pin
  int DIR2_state; // state of REV direction pin
} pwm_struct;

typedef struct{ // for stepper motor with driver
  enc_struct enc;
  volatile unsigned int* LATCH;
  int DIR_PIN;
  int STEP_PIN;
  int ENABLE_PIN;
  int target_pos;
  int current_pos;
}motor_struct;

// data structure to hold a row of data to send over serial connection
typedef struct{
  int numColumns;
  char** columnLabels; // array column labels
  char** unitsLabels;
  char dataRow[NUM_COLUMNS][10];
  int printHeader; // flag to indicate if header should be printed
} post_data;

typedef struct list_element{
  long int datum;
  struct list_element* next;
  struct list_element* prev;
}list_element;

typedef struct{
  int filter_order;
  long int input_coeffs[TIME_HISTORY];
  long int output_coeffs[TIME_HISTORY];
}digital_filter;

typedef struct {
  long int unfiltered_value;
  long int filtered_value;
  long int k1; // ewma constant multiplied by 1000. i.e., 0.15 -> 150, amount of input value contribution
  long int k2; // 1000-k1, amount of last output value contribution
  // circular buffers to hold past values
  list_element past_outputs[TIME_HISTORY];
  list_element past_inputs[TIME_HISTORY];
  list_element* outputs_head;
  list_element* inputs_head;
  digital_filter* filter; // the default filter to use with this data stream
  long int scaling_factor; // scaling factor multiplied by 2^10;
  int offset; // offset value
  int deriv; // low-passed derivative of values in time history
  long unsigned int integral; // integral of past values
  int average; // average of past values
}gradient_data_struct;




// struct to hold the arguments of an incoming command from serial connection
typedef struct{
  char arg0[16];
  char arg1[16];
  char arg2[16];
} command_struct;

// structure to store ramped position data
typedef struct{
  long unsigned int startTime;
  long int startPos;
  double cts_per_ms;
}ramp_struct;

typedef struct{
  long unsigned int time;
  command_struct command;
  int done;
}timed_command;

// Function Prototypes
void init_clock(void);
void init_samptime(void);
void init_pins(void);                // all pin assignments are made here
int btnDebounce(void);               // routine for debouncing button press

// initialize quadrature encoder interface
void init_encoders(enc_struct* encoder1, enc_struct* encoder2);
void init_ADC(void);                 // initialize ADC1 module
void init_pwm1(pwm_struct* pwm1);      // initialize pwm1
void init_pwm2(pwm_struct* pwm2);      // initialize pwm2

void init_UART(void);                // initialize UART for serial communication

int read_ADC(int channel); // read voltage on analog pin specified by channel
void readEncoder(enc_struct* pos_data);// process encoder data
void doPWM(pwm_struct* pwm);           // set pwm output
void updateLEDs(LED_struct* LED);   // to light up leds
void pause_toggle(int* pause_flag); // toggles pause flag
void halt(void);                    // turn off power to motors and voice coil
// serial communications utilities
void pollForSerial(command_struct* command);  // check serial input buffer
void parseInputString(command_struct* command, char * input);
void doCommand(command_struct* command);
void postHeadings(post_data* data);  // print column headings to serial out
void postUnits(post_data* data);     // print unit labels to serial out
void initPostData(post_data* data, char* headings[], char* units[], int numCols);
void postRowData(post_data* data);   // print a row of data to serial out
void run_steppers();
void step(motor_struct* motor, int direction); // send a pulse to a stepper motor
long int filter(gradient_data_struct* data, long int new_input);
void cmdUnknown(command_struct* command); // send error over serial
void motor_enable(motor_struct* motor, int enable); // enable or disable motor
void init_filter(digital_filter* filter, int filter_order);
void init_gradientData(gradient_data_struct* gradData, digital_filter* filter);
int differentiate(gradient_data_struct* data);
long unsigned int integrate(gradient_data_struct* data);
int get_average(gradient_data_struct* data);
void calc_offsets(gradient_data_struct* data[]);
#endif
