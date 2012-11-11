#include "PID.h"

// assign error coefficients for a PID process.
void PID_init(controller_data* procData, long int* input_signal,float kp, float kd,
    float ki, float samplePeriod, long int max_output, long int min_input){
  procData->input_signal = input_signal;
  procData->l_err = 0;
  procData->l_l_err = 0;
  procData->l_l_output = 0;
  procData->l_output = 0;
  procData->max_output = max_output;
  procData->min_output = min_input;
  procData->referenceSig = 0;
  procData->samplePeriod = samplePeriod;

  // update coefficients for PID parameters
  PID_setCoeffs(procData, kp, kd, ki);

}
// set error coefficients according to PID paramters
void PID_setCoeffs(controller_data *procData, float kp, float kd, float ki){
  procData->kp = kp;
  procData->kd = kd;
  procData->ki = ki;

if (ki == 0.0 && kd == 0.0){ // just P control
  procData->k_err = kp;
  procData->k_l_err = 0;
  procData->k_l_l_err = 0;
  procData->k_l_out = 0;
  procData->k_l_l_out = 0;
}
else if(kd > 0.0 && ki == 0.0){ // if just PD control

  procData->k_err = kp + 2.0*kd/procData->samplePeriod;
  procData->k_l_err = kp - 2.0*kd/procData->samplePeriod;
  procData->k_l_l_err = 0;

  procData->k_l_out = -1.0;
  procData->k_l_l_out = 0;
}
else if(kd == 0.0 && ki > 0.0 ){ // PI control
  // TODO: PI control
}
else{ // for PID control

  procData->k_err = kp + (2.0*kd)/procData->samplePeriod
      + (procData->samplePeriod*ki)/2.0;

  procData->k_l_err = procData->samplePeriod*ki - 4.0*kd/procData->samplePeriod;

  procData->k_l_l_err = (2.0*kd)/procData->samplePeriod
      + (procData->samplePeriod*ki)/2.0 - kp;

  procData->k_l_out = 0;
  procData->k_l_l_out = 1; // coefficient on last last output
}

}

// function to take pointer to process data and sensed variable value
// returns the calculated output
int PID_run(controller_data* procData){
  // calculate error
  long int error = procData->referenceSig - *(procData->input_signal);
  long int output;
  output = (long int)(
        procData->k_err * error
      + procData->k_l_err   * procData->l_err
      + procData->k_l_l_err * procData->l_l_err
      + procData->k_l_out   * procData->l_output
      + procData->k_l_l_out * procData->l_l_output);

  // shift error and output values 1 unit back in time
  procData->l_l_err = procData->l_err;
  procData->l_err = error;
  procData->l_l_output = procData->l_output;

  // make sure output is within acceptable range and saturate if necessary
  if(output > procData->max_output)
    output = procData->max_output;
  else if(output < procData->min_output)
      output = procData->min_output;

  procData->l_output = output;
  
//  // check if we're within 2% of steady state value
//  if((error > procData->referenceSig + procData->twoPercent
//          && error < procData->referenceSig - procData->twoPercent)
//    || (error < procData->referenceSig + procData->twoPercent
//          && error > procData->referenceSig - procData->twoPercent)){
//    procData->SS_debounce++; // increment debouncer
//  }
//  else{
//    procData->SS_debounce--; // decrement debouncer if we're out of 2% range
//  }

  return output;
}

// set reference signal and calculate two percent of steady state
void PID_setReference(controller_data* procData, long int reference){
  if (reference != procData->referenceSig){
    // saturate reference signal if necessary to prevent runaway
    if(reference < procData->min_ref)
      procData->referenceSig = procData->min_ref;
    else if(reference > procData->max_ref)
      procData->referenceSig = procData->max_ref;
    procData->twoPercent = (long int)(0.02*reference);
  }
}

// clear history of process data, for controller switch
void PID_clearOld(controller_data* procData){
  procData->l_err = 0;
  procData->l_l_err = 0;
  procData->l_output = 0;
  procData->l_l_output = 0;
}
