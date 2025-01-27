#include "AVR_PWM.h"

AVR_PWM* PWM_Instance;

float frequency;

// Define the pins
#define PWM_PIN 9 // The pin for the PWM output to the load
#define LOAD_PIN A1 // The pin for measuring the load voltage
#define LOAD_VOLTAGE_RANGE 30.0 // the max value of load analog input maps to this voltage, in Volts
#define SOURCE_PIN A0 // The pin for measuring the source voltage
#define SOURCE_VOLTAGE_RANGE 28.0 // the max value of source analog input maps to this voltage, in Volts

// Define the constants
#define MAX_PWM 99.0 // The maximum PWM duty cycle
#define MIN_PWM 0.0 // The minimum PWM duty cycle
uint16_t PWMPeriod = 0 ; 
#define FREQ_MAX 14000.0;


#define LOAD_FACTOR 0.001 // The factor for increasing or decreasing the load
#define CALIBRATION_FACTOR 0.0001 // The factor for increasing or decreasing the load during RINT calibration
#define VOLTAGE_FACTOR 0.80 // The factor for keeping the output voltage within 80% of the open-circuit voltage
#define CALIBRATION_INTERVAL 20000 // The interval for calibrating the open-circuit voltage in milliseconds
#define OPEN_CIRCUIT_MEASUREMENT_TIME 1000 // time to wait after opening the circuit before measuring of open-circuit voltage
#define CALIBRATION_DURATION 3000 // The duration for calibrating the internal resistance in milliseconds
#define RINT_CALIBRATION_MEASUREMENT_TIME 20 // time to wait after changing pwm value before measuring voltage across load  
#define LEARNING_RATE 0.001 // The learning rate for the gradient descent algorithm
#define LEARNING_RATE_LOAD 0.001 // The learning rate for the gradient descent algorithm - load adjustment

#define DEBUG_SERIAL_OUTPUT_INTERVAL 100 // interval for debug output for visualization

// Define the variables
float open_circuit_voltage = 0; // The open-circuit voltage of the source
float open_circuit_load_voltage = 0; // The open-circuit voltage of the source
float internal_resistance_src = 0; // The internal resistance of the source
float internal_resistance_load = 0; // The internal resistance of the source
float load_resistance = 0; // The load resistance
float source_to_load_resistance = 0; // The load resistance

float output_voltage = 0; // The output voltage
float output_current = 0; // The output current
float input_current = 0; // The input current
float output_power = 0; // The output power
float source_voltage = 0; // measured source voltage
float load_voltage = 0; // measured load voltage
float corrected_voltage = 0; // measured load voltage
float error = 0 ; // error for load adjusting
float load = 0 ; // estimated load 

//uint16_t pwm_value = 0; // The PWM value
float pwm_value = 0; // The PWM value

unsigned long last_calibration_time = CALIBRATION_INTERVAL; // The last time the open-circuit voltage was calibrated
unsigned long calibration_start_time = 0; // The start time of the internal resistance calibration
unsigned long last_debug_output = 0; // The start time of the internal resistance calibration

bool is_calibrating = false; // The flag for indicating if the internal resistance is being calibrated

// Define the array for the rint values at various open-circuit voltages
// This is a look up table that can be used to estimate the internal resistance
// The values are in ohms and are based on some hypothetical data
// You can modify this array according to your own data
float rint[11] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1};

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);
  // Set the PWM pin as output
  pinMode(PWM_PIN, OUTPUT);
  // Set the initial PWM value to zero
  analogWrite(PWM_PIN, MIN_PWM);

  frequency = FREQ_MAX;
  PWM_Instance = new AVR_PWM(PWM_PIN, frequency, MIN_PWM);

  if (PWM_Instance)
  {
    PWM_Instance->setPWM();
  }

  PWMPeriod = PWM_Instance->getPWMPeriod();
  
}

void debug_output(){

/*
//    Serial.print("Output voltage: ");
//    Serial.println(output_voltage);
//    Serial.print("Output current: ");
//    Serial.println(output_current);
//    Serial.print("Output power: ");
//    Serial.println(output_power);

  // Print the output voltage, current and power
//  Serial.print(F("V_out= "));
//  Serial.print(output_voltage);
//  Serial.print(F(" V, I_out= "));
  Serial.print(F("R_int= "));
  Serial.print(internal_resistance_src);

  Serial.print(F(" R_int_load= "));
  Serial.print(internal_resistance_load);

  Serial.print(F(" Rload= "));
  Serial.print(load_resistance);

  Serial.print(F(" R_pwm= "));
  Serial.println(source_to_load_resistance);

  Serial.print(F("I_out= "));
  Serial.print(output_current);
  Serial.print(F(" A, P_out= "));
  Serial.print(output_power);
  Serial.print(F(" W, "));

  // Print the corrected source voltage
//  Serial.print(F("V_cor= "));
//  Serial.print(corrected_voltage);
//  Serial.print(F(" V, "));

  // Print the last measured open-circuit voltage
  Serial.print(F("V_oc= "));
  Serial.print(open_circuit_voltage);
  Serial.print(F(" V, "));
  // Print the last measured open-circuit voltage of the load
  Serial.print(F("V_oc_load= "));
  Serial.print(open_circuit_load_voltage);
  Serial.print(F(" V, "));


  // Print the measured source voltage
  Serial.print(F("V_src= "));
  Serial.print(source_voltage);
  Serial.print(F(" V, "));

  // Print the measured load voltage
  Serial.print(F("V_load= "));
  Serial.print(load_voltage);
  Serial.print(F(" V, "));

  // Print the measured load voltage
  Serial.print(F("error= "));
  Serial.print(error,3);

  // Print the measured load voltage
  Serial.print(F(" load= "));
  Serial.print(load,3);

    // Print the load value
  Serial.print(F(" PWM= "));
  Serial.println(pwm_value);

*/

  Serial.print(F("V_oc:"));
  Serial.print(open_circuit_voltage);
  Serial.print(F(","));
  // Print the corrected source voltage
  Serial.print(F("V_cor:"));
  Serial.print(corrected_voltage);

  // Print the measured source voltage
  Serial.print(F(",V_src:"));
  Serial.print(source_voltage);
  Serial.print(F(","));
  // Print the measured load voltage
  Serial.print(F("V_load:"));
  Serial.print(load_voltage);
  Serial.print(F(","));
//  Serial.print(F("maxperiod:"));
//  Serial.print(PWMPeriod);
//  Serial.print(F(","));

  Serial.print(F("R_int:"));
  Serial.print(internal_resistance_src);

  // Print the measured load voltage
  Serial.print(F(",load:"));
  Serial.println(load,3);


}

void loop() {
  // Read the source voltage
  source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

  // Check if it is time to calibrate the open-circuit voltage and rint
  if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {
    // Set the PWM value to zero
      pwm_value = 0;
//    analogWrite(PWM_PIN, MIN_PWM);

      PWM_Instance->setPWM(PWM_PIN, frequency, MIN_PWM);
      
    // Update the open-circuit voltage
    delay (OPEN_CIRCUIT_MEASUREMENT_TIME); // delay to allow open source voltage to return to maximum value
    open_circuit_voltage        = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0); // read open circuit voltage of the source
    open_circuit_load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);     // read open circuit voltage of the load

//    open_circuit_voltage = source_voltage;
  
    // Update the last calibration time
    last_calibration_time = millis();
    // Set the calibration flag to true to trigger the rint calibration 
    is_calibrating = true;
    // Set the rint calibration start time 
    calibration_start_time = millis();
    // Print the open-circuit voltage
//    Serial.print("Open-circuit voltage: ");
//    Serial.println(open_circuit_voltage);
  }//if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {
  
  

  // Check if the internal resistance is being calibrated
  if (is_calibrating) {
    // Check if the calibration duration is over
    if (millis() - calibration_start_time > CALIBRATION_DURATION) {
      // Set the calibration flag to false
      is_calibrating = false;
      // Reset the PWM value
   //   load = 0;
    } else {
      // Increase the PWM value by CALIBRATION_FACTOR
      load = load + 1 * CALIBRATION_FACTOR;
      // Constrain the PWM value within the limits
      load = constrain(load, 0, 1);
      // Write the PWM value to the pin
//      analogWrite(PWM_PIN, pwm_value);
      pwm_value = (PWMPeriod-1)*load ; 
      PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);

      delay(RINT_CALIBRATION_MEASUREMENT_TIME); // delay for some time to allow load voltage to change
      // Read the load voltage
       load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
       source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
      
      // Calculate the load resistance
//      load_resistance = (load_voltage) / ((pwm_value / 255.0)+0.001); //total
      internal_resistance_load = (open_circuit_load_voltage-load_voltage) / ((1-load)+0.001);
      load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ; // only resistance of the load itself

      // Calculate the output current
      output_current = load_voltage / (load_resistance+internal_resistance_load+0.001);
      float source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001); 
       
      // Calculate the output power
      output_power = output_current * output_current * (load_resistance+internal_resistance_load);
      internal_resistance_src = (open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001)) ;

      // Calculate the internal resistance using the gradient descent algorithm
      // The cost function is the difference between the output power and the maximum power
      // The gradient is the derivative of the cost function with respect to the internal resistance
      // The update rule is to subtract the gradient times the learning rate from the current value
      float cost = output_power - (open_circuit_voltage * open_circuit_voltage) / ((4 * internal_resistance_src)+0.01);
      float gradient =  (open_circuit_voltage * open_circuit_voltage) / (((4 * internal_resistance_src * internal_resistance_src) + output_current * output_current)+0.001);
      internal_resistance_src = internal_resistance_src - gradient * LEARNING_RATE+0.001;
      // Constrain the internal resistance to be positive
      internal_resistance_src = max(internal_resistance_src, 0);
/*
      // Print the PWM value
      Serial.print("PWM: ");
      Serial.print(pwm_value);

      Serial.print(" R_load: ");
      Serial.print(load_resistance);

      Serial.print(" R_PWM: ");
      Serial.print(source_to_load_resistance);

      Serial.print(" out_current: ");
      Serial.print(output_current);

      Serial.print(" out_P: ");
      Serial.print(output_power);

      Serial.print(" cost: ");
      Serial.print(cost);

      Serial.print(" gradient: ");
      Serial.print(gradient);

      // Print the internal resistance
      Serial.print(" Internal resistance: ");
      Serial.println(internal_resistance_src);
  */
    }
  }

       load_voltage   = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
       source_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);

      // Calculate the load resistance
      load_resistance = (load_voltage / ((source_voltage-load_voltage)/(1-load)+0.001)); //total
      output_current  = load_voltage / (load_resistance+0.001);
      
      source_to_load_resistance = (source_voltage - load_voltage)/ ((1-load)+0.001); 
      internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.001);
      load_resistance = ((load_voltage) / ((1-load)+0.001)) - internal_resistance_load ; // only resistance of the load itself

      input_current = source_voltage / (load_resistance + source_to_load_resistance+0.001);
      internal_resistance_load = (load_voltage-open_circuit_load_voltage) / ((1-load)+0.001);

    // Calculate the output power
      output_power = output_current * load_voltage;

      internal_resistance_src = (internal_resistance_src *0.9 + 0.1*(open_circuit_voltage-source_voltage)/(source_voltage/((source_to_load_resistance+load_resistance+internal_resistance_load)+0.001))) ;


    // Use the open-circuit voltage corrected by the internal resistance and the load
     corrected_voltage = (open_circuit_voltage - input_current * internal_resistance_src);

/*
    corrected_voltage = analogRead(SOURCE_PIN) * (SOURCE_VOLTAGE_RANGE / 1023.0);
    // Check if the source voltage has increased
    if (source_voltage > corrected_voltage) { 
      // Increase the load by 10%
      pwm_value = pwm_value + MAX_PWM * LOAD_FACTOR;
      // Constrain the PWM value within the limits
      pwm_value = constrain(pwm_value, MIN_PWM, MAX_PWM);
      // Write the PWM value to the pin
      analogWrite(PWM_PIN, pwm_value);
    }

    */

    /*
    // Check if the source voltage has decreased
    if (source_voltage < corrected_voltage) { 
      // Decrease the load to reach the 80% value back based on previous calculations
      // The target voltage is the open-circuit voltage times the voltage factor
      float target_voltage = open_circuit_voltage * VOLTAGE_FACTOR;
      // The target current is the target voltage divided by the load resistance
      float target_current = target_voltage / load_resistance;
      // The target PWM value is the target current times the load resistance times 255
      float target_load = target_current  ;
      // Constrain the target PWM value within the limits
      target_load = constrain(target_load, 0, 1);
      // converge the PWM value to the target by simply averaging
      load = (load + target_load)/2;
      // Write the target PWM value to the pin
//      analogWrite(PWM_PIN, pwm_value);

    }
    */

/*
    // Read the load voltage
    load_voltage = analogRead(LOAD_PIN) * (LOAD_VOLTAGE_RANGE / 1023.0);
    // Calculate the load resistance
    load_resistance = load_voltage / (pwm_value / 255.0);
    // Calculate the output current
    output_current = load_voltage / load_resistance;
    // Calculate the output voltage
    output_voltage = source_voltage - output_current * internal_resistance_src;
    // Calculate the output power
    output_power = output_current * output_voltage;
    // Print the output voltage, current and power

*/

 // error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - source_voltage)/2;
//  error = (error+( VOLTAGE_FACTOR * corrected_voltage) - source_voltage)/2;
  error = (error+( VOLTAGE_FACTOR * open_circuit_voltage) - corrected_voltage)/2;

  // Adjust the load value using gradient descent algorithm
  load = load - LEARNING_RATE_LOAD * error;
  // Constrain the load value between 0 and 1
  load = constrain(load, 0, 1);
  // Write the load value to the PWM pin
//  pwm_value = PWMPeriod/(load * MAX_PWM); 
  pwm_value = (PWMPeriod)*load ; 
  PWM_Instance->setPWM(PWM_PIN, frequency, pwm_value);
//  analogWrite(PWM_PIN, pwm_value);
// delay(100);


  if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
    debug_output(); // print debug output to serial out
    last_debug_output = millis();
  }
    
  
}
