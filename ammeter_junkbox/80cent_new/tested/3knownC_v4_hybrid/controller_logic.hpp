#ifndef CONTROLLER_LOGIC_HPP
#define CONTROLLER_LOGIC_HPP

void controller_setup();
void controller_loop();

// Common variables that different versions might want to expose or share
extern float open_circuit_voltage;
extern float internal_resistance_src;
extern float resistance_tau_est;
extern float load;
extern float fitted_tau;
extern float calibration_interval_current;

#endif
