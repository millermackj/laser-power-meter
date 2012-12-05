// PID library header file

// data structure to store process data

#ifndef _PID_H
#define _PID_H

// enable bits for P, I, and D functionality
#define P_EN 0x1
#define I_EN 0x2
#define D_EN 0x4


typedef struct {
  long int referenceSig;  // desired setpoint for control variable
  long int *input_signal; // pointer to input signal for this controller
  long int max_output;    // maximum output value
  long int min_output;    // minimum output value
  long int max_ref;       // saturate reference at max
  long int min_ref;       // saturate reference at min

  long int l_err;         // error from one sample ago
  long int l_l_err;       // error from two samples ago

  long int l_output;      // output from one sample ago
  long int l_l_output;    // output from two samples ago

  float k_err;						// current error control coefficient
  float k_l_err;          // last error control coefficient
  float k_l_l_err;				// last last error control coefficient

  float k_l_out;          // coefficient on last output
	float k_l_l_out;        // coefficient on last last output

  float samplePeriod;     // sample period in seconds

  int SS_debounce;        // flag indicates no. samples error is less than 2%

  long int twoPercent;    // 2% of reference signal
  float kp, kd, ki;

} controller_data;

// assign error coefficients and initialize history variables for a PID process.
void PID_init(controller_data* procData, long int* input_signal, float k_prop, float k_deriv,
    float k_integ, float samplePeriod, long int max_output, long int min_input);
// assign error coefficients only for a PID process
void PID_setCoeffs(controller_data *procData, float kp, float kd, float ki);
// function takes pointer to process data structure and sensed variable value.
// Returns the calculated output for process.
int PID_run(controller_data* procData);
// set reference signal and calculate 2% of steady state
void PID_setReference(controller_data* procData, long int reference);

//zero out last values from a controller
void PID_clearOld(controller_data* procData);

#endif
