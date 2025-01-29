#include "stm8s.h"
#include "stm8s_tim1.h"
#include "stm8s_adc1.h"
#include "stm8s_uart1.h"

#define PWM_MAX_PERIOD 512 // define pwm period

// Pin definitions (adjust according to your STM8 board)
#define PWM_PIN             GPIOC, 2  // TIM2 CH3
#define LOAD_PIN           5     // AIN2
#define SOURCE_PIN         4     // AIN3

// Voltage ranges
#define LOAD_VOLTAGE_RANGE    30.0f
#define SOURCE_VOLTAGE_RANGE  28.0f

// PWM limits
#define MAX_PWM              99.0f
#define MIN_PWM              0.0f
#define FREQ_MAX            14000UL

// Timing constants (in milliseconds)
#define CALIBRATION_INTERVAL                20000UL
#define OPEN_CIRCUIT_MEASUREMENT_TIME       1000UL
#define CALIBRATION_DURATION               3000UL
#define RINT_CALIBRATION_MEASUREMENT_TIME    20UL
#define DEBUG_SERIAL_OUTPUT_INTERVAL        100UL

// Control factors
#define LOAD_FACTOR          0.001f
#define CALIBRATION_FACTOR   0.0001f
#define VOLTAGE_FACTOR       0.80f
#define LEARNING_RATE        0.001f
#define LEARNING_RATE_LOAD   0.001f

// State variables
static float open_circuit_voltage = 0;
static float open_circuit_load_voltage = 0;
static float internal_resistance_src = 0;
static float internal_resistance_load = 0;
static float load_resistance = 0;
static float source_to_load_resistance = 0;

// Measurement variables
static float output_voltage = 0;
static float output_current = 0;
static float input_current = 0;
static float output_power = 0;
static float source_voltage = 0;
static float load_voltage = 0;
static float corrected_voltage = 0;
static float error = 0;
static float load = 0;
static uint16_t pwm_value = 0;
static uint16_t pwm_period = 0;

// Timing variables
static uint32_t last_calibration_time = CALIBRATION_INTERVAL;
static uint32_t calibration_start_time = 0;
static uint32_t last_debug_output = 0;
static bool is_calibrating = false;

// Timer tick counter for timing functions
static volatile uint32_t timer_ticks = 0;

// Function prototypes
void clock_setup(void);
void GPIO_setup(void);
void TIM1_setup(uint16_t period);
void ADC_setup(void);
void UART_setup(void);
float read_voltage(ADC1_Channel_TypeDef channel, float voltage_range);
void update_measurements(void);
void calculate_resistances(void);
void calculate_currents_and_power(void);
void update_internal_resistance(void);
void adjust_load(void);
void calibrate(void);
void handle_calibration(void);
void print_debug_info(void);

// Utility functions
//uint32_t millis(void) {
//    return timer_ticks;
//}

//void delay(uint32_t ms) {
//    uint32_t start = millis();
//    while((millis() - start) < ms);
//}

//float constrain(float value, float min, float max) {
//    if(value < min) return min;
//    if(value > max) return max;
//    return value;
//}

// Initialize all peripherals
void system_init(void) {
    clock_setup();
    GPIO_setup();
    TIM1_setup(PWM_MAX_PERIOD);
    ADC_setup();
    UART_setup();
    
    // Enable interrupts
    enableInterrupts();
}

void clock_setup(void) {
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);  // 16MHz
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
}

void GPIO_setup(void) {
    GPIO_Init(PWM_PIN, GPIO_MODE_OUT_PP_HIGH_FAST);
}

void TIM1_setup(uint16_t period) {
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1, ENABLE);
 
    // Configure Timer1 for PWM generation
    TIM1_TimeBaseInit(1, // prescaler 
     TIM1_COUNTERMODE_UP,
     period,0);  // Adjust for desired frequency
    TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,0, TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_LOW,TIM1_OCIDLESTATE_SET,TIM1_OCNIDLESTATE_RESET);
    //TIM2_OC1PreloadConfig(ENABLE);
    TIM1_CtrlPWMOutputs(ENABLE);
    TIM1_Cmd(ENABLE);
    
//    pwm_period = TIM2_GetPeriod();
    pwm_period = period-1; 
}

void ADC_setup(void) {
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, 
              ADC1_CHANNEL_3,
              ADC1_PRESSEL_FCPU_D8,
              ADC1_EXTTRIG_TIM,
              DISABLE,
              ADC1_ALIGN_RIGHT,
              ADC1_SCHMITTTRIG_ALL,
              DISABLE);
}

void UART_setup(void) {
Serial_begin(115200);
}

float read_voltage(ADC1_Channel_TypeDef channel, float voltage_range) {
    ADC1_ConversionConfig(ADC1_CONVERSIONMODE_SINGLE, channel, ADC1_ALIGN_RIGHT);
    ADC1_StartConversion();
    while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
    uint16_t adc_value = ADC1_GetConversionValue();
    ADC1_ClearFlag(ADC1_FLAG_EOC);
    return (float)adc_value * (voltage_range / 1023.0f);
}

// Main control functions
void update_measurements(void) {
    source_voltage = read_voltage(SOURCE_PIN, SOURCE_VOLTAGE_RANGE);
    load_voltage = read_voltage(LOAD_PIN, LOAD_VOLTAGE_RANGE);
}

void calculate_resistances(void) {
    load_resistance = (load_voltage / ((source_voltage - load_voltage)/(1-load) + 0.001f));
    source_to_load_resistance = (source_voltage - load_voltage) / ((1-load) + 0.001f);
    internal_resistance_load = (load_voltage - open_circuit_load_voltage) / ((1-load) + 0.00001f);
    load_resistance = (load_voltage / ((1-load) + 0.001f)) - internal_resistance_load;
}

void calculate_currents_and_power(void) {
    output_current = load_voltage / (load_resistance + internal_resistance_load + 0.001f);
    input_current = source_voltage / (load_resistance + source_to_load_resistance + 0.001f);
    output_power = output_current * load_voltage;
}

void update_internal_resistance(void) {
    float new_resistance = (open_circuit_voltage - source_voltage) /
        (source_voltage / ((source_to_load_resistance + load_resistance + internal_resistance_load) + 0.00001f));
    internal_resistance_src = internal_resistance_src * 0.9f + 0.1f * new_resistance;
}

void adjust_load(void) {
    corrected_voltage = open_circuit_voltage - input_current * internal_resistance_src;
    error = (error + (VOLTAGE_FACTOR * open_circuit_voltage - corrected_voltage)) / 2;
    load = constrain(load - LEARNING_RATE_LOAD * error, 0, 1);
    //load = 0.5; //debug
    pwm_value = (uint16_t)(pwm_period * load);
    TIM1_SetCompare1(pwm_value);
}

void calibrate(void) {
    TIM1_SetCompare1(0);  // Set PWM to minimum
    delay(OPEN_CIRCUIT_MEASUREMENT_TIME);
    
    open_circuit_voltage = read_voltage(SOURCE_PIN, SOURCE_VOLTAGE_RANGE);
    open_circuit_load_voltage = read_voltage(LOAD_PIN, LOAD_VOLTAGE_RANGE);
    
    last_calibration_time = millis();
    is_calibrating = true;
    calibration_start_time = millis();
}

void handle_calibration(void) {
    if (millis() - calibration_start_time > CALIBRATION_DURATION) {
        is_calibrating = false;
        return;
    }
    
    load = constrain(load + CALIBRATION_FACTOR, 0, 1);
    pwm_value = (uint16_t)((pwm_period - 1) * load);
    TIM1_SetCompare1(pwm_value);
    
    delay(RINT_CALIBRATION_MEASUREMENT_TIME);
    update_measurements();
    calculate_resistances();
    calculate_currents_and_power();
}

void print_debug_info(void) {
    // Implementation depends on your UART print function
    // You'll need to implement string conversion functions
    // This is a simplified version
    //char buffer[100];
    //sprintf(buffer, "V_oc:%.2f,V_cor:%.2f,V_src:%.2f,V_load:%.2f,R_int:%.2f,load:%.3f\r\n",
//            open_circuit_voltage, corrected_voltage, source_voltage, load_voltage,
//            internal_resistance_src, load);
    
        Serial_print_s("V_oc:"); Serial_print_f(open_circuit_voltage);
        Serial_print_s(",V_cor:"); Serial_print_f(corrected_voltage);
        Serial_print_s(",V_src:"); Serial_print_f(source_voltage);
        Serial_print_s(",V_load:"); Serial_print_f(load_voltage);
        Serial_print_s(",R_int:"); Serial_print_f(internal_resistance_src);
        Serial_print_s(",load:"); Serial_print_f(load);
                
        Serial_println();

}

void setup() {
    system_init();
}
void loop() {
  
    
        if (millis() - last_calibration_time > CALIBRATION_INTERVAL) {
            calibrate();
        }
        
        if (is_calibrating) {
            handle_calibration();
        } else {
            update_measurements();
            calculate_resistances();
            calculate_currents_and_power();
            update_internal_resistance();
            adjust_load();
        }
        
        if (millis() - last_debug_output > DEBUG_SERIAL_OUTPUT_INTERVAL) {
            print_debug_info();
            last_debug_output = millis();
        }
    
}

// Timer interrupt for millis() functionality
//INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) {
//    timer_ticks++;
//    TIM4_ClearFlag(TIM4_FLAG_UPDATE);
//}
