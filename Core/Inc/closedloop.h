#ifndef CLOSEDLOOP_H
#define CLOSEDLOOP_H

#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

/* ===================== USER CONFIG ===================== */

// Timing configuration (adjustable)
#define ISR_FREQ_HZ_DEFAULT  1000.0f  
#define POLE_PAIRS           7
#define PWM_ARR              4250
#define LUT_SIZE             1024

#define TWO_PI               6.28318530718f
//#define M_PI                 3.14159265359f

// Motor parameters
#define VDC_VOLTAGE          12.0f      // Supply voltage in volts

// Low-pass filter
#define LPF_CUTOFF_HZ        10.0f      // Speed filter cutoff frequency

// Calibration
#define ALIGNMENT_VOLTAGE    2.0f       // Voltage for rotor alignment (V)
#define ALIGNMENT_TIME_MS    1000       // Time to settle during calibration (ms)

// Open-loop startup
#define OPENLOOP_RPM         60.0f      // Open-loop startup speed
#define OPENLOOP_AMPLITUDE   0.2f       // Open-loop voltage amplitude
#define CALIBRAT_AMPLITUDE	 0.5f  		// Calibration Amplitude
#define OPENLOOP_TIME_MS     2000       // Duration of open-loop (ms)

// USART
#define USART_BAUDRATE       2000000    // 2 Mbps
#define USART_TX_BUFFER_SIZE 256
#define USART_RX_BUFFER_SIZE 64

/* ===================== SYSTEM STATES ===================== */

typedef enum {
    STATE_CALIBRATION =0,
	STATE_OPEN_LOOP,
    STATE_CLOSED_LOOP,
	STATE_FAULT
} SystemState_t;


/* ===================== GLOBAL STATE ===================== */

// System state
extern volatile SystemState_t system_state;
extern volatile uint8_t calibration_complete;

// Timing parameters (dynamically adjustable)
extern volatile float ISR_FREQ_HZ_CURRENT;
extern volatile float dt_seconds;

// Control inputs
extern volatile float speed_ref_rpm;
extern volatile float amplitude;

// Open-loop state & direction sense
extern volatile float theta;
extern volatile float delta_theta_per_call;


//Calibration
extern volatile int8_t motor_direction;
extern volatile uint8_t MoveCommand;

// Closed-loop state
extern volatile float encoder_offset;
extern volatile float theta_electrical;
extern volatile float theta_electrical_prev;
extern volatile float speed_rpm;
extern volatile float speed_filtered;
extern volatile bool closedloop_enable;

// Voltage commands
extern volatile float Vd_cmd;
extern volatile float Vq_cmd;

// Output PWM values
extern volatile uint16_t pwm_a;
extern volatile uint16_t pwm_b;
extern volatile uint16_t pwm_c;

// I2C error handling
extern volatile uint32_t i2c_error_count;
extern volatile uint8_t i2c_last_error;  // 1=TXIS, 2=TC, 3=RX_H, 4=RX_L, 5=STOP

// LPF coefficient
extern volatile float lpf_alpha;

// USART communication
extern volatile char usart_tx_buffer[USART_TX_BUFFER_SIZE];
extern volatile uint16_t usart_tx_head;
extern volatile uint16_t usart_tx_tail;
extern volatile uint8_t usart_tx_busy;

extern volatile char usart_rx_buffer[USART_RX_BUFFER_SIZE];
extern volatile uint16_t usart_rx_index;
extern volatile uint8_t usart_rx_complete;

// Task - ISR control
extern volatile uint8_t openloop_enable;

// Debug Controls
extern bool debug_enable;

/* ===================== API ===================== */

// Initialization
void init_sine_lut(void);
void Update_Control_Timing(float new_isr_freq_hz);
void LPF_Init(float cutoff_freq_hz, float sample_freq_hz);

// Main control functions
void Motor_Control_ISR(void);
void Run_Open_Loop(void);
void Run_Calibration(void);
void Run_Closed_Loop(void);
void Run_Targetangle(float angle);


// Utility
float LPF_Update(float new_sample, volatile float *filtered_state);

// USART functions
void USART_Init(void);
void USART_SendString(const char *str);
void USART_SendStatus(void);
void USART_ProcessCommand(void);

// I2C functions
uint16_t AS5600_ReadAngle_Blocking(void);
float AS5600_Angle_To_Radians(uint16_t);

// FreeRTOS task
void Task_State_Machine(void *pvParameters);

/* ===================== PI Controller ===================== */

void PI_Init(float kp, float ki, float dt, float out_min, float out_max);
void PI_Reset(void);
float PI_Update(float error);
void PI_Set_Gains(float kp, float ki);
void PI_Set_SampleTime(float dt);

#endif /* CLOSEDLOOP_H */
