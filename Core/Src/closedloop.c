/*
 * closedloop.c
 *
 *  Created on: Mar 10, 2026
 *      Author: Bilbert George
 *       email: bilber.work@gmail.com
 *    LinkedIn: https://www.linkedin.com/in/bilbert-george-85a505261/
 */

#include "closedloop.h"
#include "main.h"
//#include "stm32g4xx_ll_tim.h"
//#include "stm32g4xx_ll_usart.h"
#include <stdio.h>
#include <string.h>
//#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

#define AS5600_I2C_ADDR 0x6C  // 0x36 shifted left by 1 for 8-bit addressing


/* ===================== GLOBAL DEFINITIONS ===================== */

// System state
volatile SystemState_t system_state = STATE_CALIBRATION;
volatile uint8_t calibration_complete = 0;

// Timing parameters
volatile float ISR_FREQ_HZ_CURRENT = ISR_FREQ_HZ_DEFAULT;
volatile float dt_seconds = 1.0f / ISR_FREQ_HZ_DEFAULT;

// Control inputs
volatile float speed_ref_rpm = OPENLOOP_RPM;
volatile float amplitude = OPENLOOP_AMPLITUDE;

// Open-loop state & direction sense
volatile float theta = 0.0f;
volatile float delta_theta_per_call = 0.0f;

//Calibration
volatile int8_t motor_direction = 0;
volatile uint8_t MoveCommand =0;

float Ecce_LUT[1024]; // Index to compute LUT for eccentricity calculation


// Closed-loop state
volatile float encoder_offset = 0.0f;
volatile float theta_electrical = 0.0f;
volatile float theta_electrical_prev = 0.0f;
volatile float speed_rpm = 0.0f;
volatile float speed_filtered = 0.0f;
volatile bool closedloop_enable = false;
volatile float speed_raw_rpm =0;
volatile float speed_error=0;

// Voltage commands
volatile float Vd_cmd = 0.0f;
volatile float Vq_cmd = 0.0f;

// Output PWM values
volatile uint16_t pwm_a = 0;
volatile uint16_t pwm_b = 0;
volatile uint16_t pwm_c = 0;

// I2C AS5600 handling
volatile uint8_t rx_buf[2];
volatile uint8_t rx_idx = 0;

volatile uint16_t current_angle = 0;
volatile uint32_t i2c_error_count = 0;
volatile uint8_t i2c_last_error = 0;  // 1=TXIS, 2=TC, 3=RX_H, 4=RX_L, 5=STOP

// LPF coefficient
volatile float lpf_alpha = 0.0f;

// USART buffers
volatile char usart_tx_buffer[USART_TX_BUFFER_SIZE];
volatile uint16_t usart_tx_head = 0;
volatile uint16_t usart_tx_tail = 0;
volatile uint8_t usart_tx_busy = 0;

volatile char usart_rx_buffer[USART_RX_BUFFER_SIZE];
volatile uint16_t usart_rx_index = 0;
volatile uint8_t usart_rx_complete = 0;

// Debug
volatile uint16_t debug_count1 =0;
volatile uint16_t debug_count2 =0;
volatile uint16_t debug_count3 =0;
volatile uint16_t debug_count4 =0;

// Task - ISR control
volatile uint8_t openloop_enable = 0;

// Debug Controls
bool debug_enable = true;


/* ===================== PI CONTROLLER GLOBALS ===================== */

// PI Controller parameters
float PI_Kp = 0.0f;
float PI_Ki = 0.0f;
float PI_dt = 0.0f;

// PI Controller state
volatile float PI_integral = 0.0f;

// PI Controller limits
float PI_output_min = 0.0f;
float PI_output_max = 0.0f;
float PI_integral_min = 0.0f;
float PI_integral_max = 0.0f;

/* ===================== PRIVATE VARIABLES ===================== */

static float sin_lut[LUT_SIZE];  // array or vector of size LUT_SIZE


void init_sine_lut(void)
{
    for (uint16_t i = 0; i < LUT_SIZE; i++)
    {
        sin_lut[i] = sinf(TWO_PI * (float)i / (float)LUT_SIZE);
    }
}

static inline float sin_from_lut(float angle)
{
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f) angle += TWO_PI;

    uint16_t index = (uint16_t)((angle * LUT_SIZE) / TWO_PI);
    if (index >= LUT_SIZE) index = LUT_SIZE - 1;

    return sin_lut[index];
}

static inline float cos_from_lut(float angle)
{
    return sin_from_lut(angle + M_PI / 2.0f);
}

/* ===================== TIMING UPDATE ===================== */

void Update_Control_Timing(float new_isr_freq_hz)
{
    ISR_FREQ_HZ_CURRENT = new_isr_freq_hz;
    dt_seconds = 1.0f / ISR_FREQ_HZ_CURRENT;

    // Update open-loop phase increment
    float elec_freq_hz = (OPENLOOP_RPM * POLE_PAIRS) / 60.0f;
    delta_theta_per_call = (TWO_PI * elec_freq_hz) / ISR_FREQ_HZ_CURRENT;

    // Update PI controller sample time
    PI_Set_SampleTime(dt_seconds);  //
    // Update LPF
    LPF_Init(LPF_CUTOFF_HZ, ISR_FREQ_HZ_CURRENT);
}

/* ===================== LOW-PASS FILTER ===================== */

/*Uses a 1st order discrete low pass filter (IIR Filter)
* y = alpha * y_prev + (1.0f - alpha) * x     
* y = present out
* x=present input
* alpha = filter coefficient, calculated based on cutoff frequency and sample rate by LPF_init() */

void LPF_Init(float cutoff_freq_hz, float sample_freq_hz)
{
    float rc = 1.0f / (TWO_PI * cutoff_freq_hz);
    float dt = 1.0f / sample_freq_hz;
    lpf_alpha = dt / (rc + dt);
}

float LPF_Update(float new_sample, volatile float *filtered_state)
{
    *filtered_state = lpf_alpha * new_sample + (1.0f - lpf_alpha) * (*filtered_state);
    return *filtered_state;
}
/* ===================== USART FUNCTIONS ===================== */

void USART_Init(void)
{
    // Enables the receiving interrupts
    LL_USART_EnableIT_RXNE(USART2);
}

void USART_SendString(const char *str)
{
    uint16_t len = strlen(str);

    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t next_head = (usart_tx_head + 1) % USART_TX_BUFFER_SIZE;

        // Wait if buffer full
        while (next_head == usart_tx_tail)
        {
            vTaskDelay(pdMS_TO_TICKS(1));  // Wait 1ms
        }

        usart_tx_buffer[usart_tx_head] = str[i];
        usart_tx_head = next_head;
    }

    // Enable TXE interrupt to start transmission
    LL_USART_EnableIT_TXE(USART2);
}

/* ===================== OPEN-LOOP CONTROL ===================== */

void Run_Open_Loop(void)
{
//	static volatile uint16_t encoder_prev = 0;
//	static volatile uint32_t direction_check_count = 0;


	// Phase accumulator
	theta += delta_theta_per_call;
	if (theta >= TWO_PI)
		theta -= TWO_PI;

	// 3-phase sine generation from LUT
	float sin_a = cos_from_lut(theta);
	float sin_b = cos_from_lut(theta + (2.0f * TWO_PI / 3.0f));
	float sin_c = cos_from_lut(theta + (4.0f * TWO_PI / 3.0f));

	// Amplitude scaling
	sin_a *= amplitude;
	sin_b *= amplitude;
	sin_c *= amplitude;

	// Sine → PWM (bipolar to unipolar conversion)
	pwm_a = (uint16_t)((sin_a + 1.0f) * (PWM_ARR * 0.5f));
	pwm_b = (uint16_t)((sin_b + 1.0f) * (PWM_ARR * 0.5f));
	pwm_c = (uint16_t)((sin_c + 1.0f) * (PWM_ARR * 0.5f));

	// Update PWM registers
	LL_TIM_OC_SetCompareCH1(TIM1, pwm_a);
	LL_TIM_OC_SetCompareCH2(TIM1, pwm_b);
	LL_TIM_OC_SetCompareCH3(TIM1, pwm_c);


// ==============Finding the direction=============
//	direction_check_count++;
//	//Finding the Direction
//	if (direction_check_count >= 1000)  // Every 500ms
//	{
//		direction_check_count = 0;
//
//		uint16_t encoder_now = AS5600_ReadAngle_Blocking();
//		int16_t encoder_delta = encoder_now - encoder_prev;
//
//		// Determine direction from encoder movement
//		if (encoder_delta > 2048)  // Wraparound backward
//			encoder_delta -= 4096;
//		else if (encoder_delta < -2048)  // Wraparound forward
//			encoder_delta += 4096;
//
//		if (encoder_delta > 10)  // Threshold to avoid noise
//			motor_direction = 1;   // CW
//		else if (encoder_delta < -10)
//			motor_direction = -1;  // CCW
//
//		encoder_prev = encoder_now;
//	}

}


void Run_Targetangle(float angle)
{
    theta = angle;
    amplitude = CALIBRAT_AMPLITUDE;  // or calibration-specific value
    // 3-phase sine generation from LUT
    float sin_a = cos_from_lut(theta);
    float sin_b = cos_from_lut(theta + (2.0f * TWO_PI / 3.0f));
    float sin_c = cos_from_lut(theta + (4.0f * TWO_PI / 3.0f));

    // Amplitude scaling
    sin_a *= amplitude;
    sin_b *= amplitude;
    sin_c *= amplitude;

    // Sine → PWM (bipolar to unipolar conversion)
    pwm_a = (uint16_t)((sin_a + 1.0f) * (PWM_ARR * 0.5f));
    pwm_b = (uint16_t)((sin_b + 1.0f) * (PWM_ARR * 0.5f));
    pwm_c = (uint16_t)((sin_c + 1.0f) * (PWM_ARR * 0.5f));

    // Update PWM registers
    LL_TIM_OC_SetCompareCH1(TIM1, pwm_a);
    LL_TIM_OC_SetCompareCH2(TIM1, pwm_b);
    LL_TIM_OC_SetCompareCH3(TIM1, pwm_c);
}
/* ===================== CALIBRATION ===================== */

void Run_Calibration_offset(void)
{
    /*Calibration depends on open-loop speed. It is set to 60RPM by default.
    * Changing this value will affect the calibration process*/
	USART_SendString("\r\n CALIBRATING.... \r\n");
    // Lock rotor to electrical angle 0
	volatile float alpha =0;
	volatile float percent =14.28f;

    // Reading the offset angle at all the electrical zero positions
	for (uint8_t i=0;i<POLE_PAIRS;i++)
	{
		if (debug_enable)
		{
		char cal_msg0[64];
		snprintf(cal_msg0, sizeof(cal_msg0), "Aligning rotor to mech angle: %.3f rad\r\n", alpha);
		USART_SendString(cal_msg0);
		}

	    Run_Targetangle(0);
	    vTaskDelay(pdMS_TO_TICKS(2000)); // 2s delay to settle
		uint16_t angle_raw = AS5600_ReadAngle_Blocking();
		float angle_mech = AS5600_Angle_To_Radians(angle_raw);
		encoder_offset = angle_mech * POLE_PAIRS;

		if (i2c_error_count>0)
		{
		USART_SendString("Encoder i2c error\r\n");
		}
		// Normalize offset to [0, 2π]
		while (encoder_offset >= TWO_PI)
		    encoder_offset -= TWO_PI;

		while (encoder_offset < 0.0f)
		    encoder_offset += TWO_PI;

		if (debug_enable)
		{
		char cal_msg1[64];
		snprintf(cal_msg1, sizeof(cal_msg1), " Offset angle: %.3f rad \r\n\n", encoder_offset);
		USART_SendString(cal_msg1);
		}

		char cal_msg2[32];
		snprintf(cal_msg2, sizeof(cal_msg2), " %.2f%% \r\n", percent);
		USART_SendString(cal_msg2);

	    MoveCommand = 1; // enable openloop inside ISR
	    vTaskDelay(pdMS_TO_TICKS(142)); // Run for 142ms which is the time to cover 51.4 degree. Adjust this value using 1000/PolePairs
	    MoveCommand = 0;
	    alpha = alpha + (TWO_PI/7);
	    percent = percent + (14.28f);
	}

    encoder_offset = 5.578f;
	// Ramp down alignment voltage
	LL_TIM_OC_SetCompareCH1(TIM1, PWM_ARR / 2);
	LL_TIM_OC_SetCompareCH2(TIM1, PWM_ARR / 2);
	LL_TIM_OC_SetCompareCH3(TIM1, PWM_ARR / 2);

	vTaskDelay(pdMS_TO_TICKS(1000));

}

/*ECCENTRICITY CALIBRATION */

void Run_Calibration_eccentricity(void)
{
    /* Clear LUT to 0 — gap detection uses zero check, same as reference */
    for (int i = 0; i < 1024; i++) {
        Ecce_LUT[i] = 0.0f;
    }

    USART_SendString("\r\n Running Eccentricity Calibration.... \r\n");

    /* Sweep 1024 equally spaced mechanical positions across one full revolution */
    for (int i = 0; i < 1024; i++)
    {
        /* 1. Compute target mechanical angle and convert to electrical */
        float theta_mech_deg = (float)i * (360.0f / 1024.0f);   /* 0.351 deg per step  */
        float theta_mech_rad = theta_mech_deg * (TWO_PI / 360.0f);
        float theta_elec_rad = theta_mech_rad * POLE_PAIRS;

        /* Normalise electrical angle to [0, 2π) */
        theta_elec_rad -= TWO_PI * floorf(theta_elec_rad / TWO_PI);

        /* 2. Apply voltage vector and wait briefly */
        Run_Targetangle(theta_elec_rad);
        vTaskDelay(pdMS_TO_TICKS(10));

        /* 3. Read actual rotor position from encoder */
        uint16_t angle_raw      = AS5600_ReadAngle_Blocking();
        float    angle_mech_rad = AS5600_Angle_To_Radians(angle_raw);

        /* 4. Compute error in ELECTRICAL radians */
        float e_angle_rad = (angle_mech_rad * POLE_PAIRS) - encoder_offset;
        float error = theta_elec_rad - e_angle_rad;

        /* 5. Normalise error to (-π, +π) */
        error -= TWO_PI * floorf((error + (float)M_PI) / TWO_PI);

        /* 6. Index LUT by ACTUAL sensor position 
         *    Scale: (mech_rad / 2π) × LUT_SIZE  */
        float lut_pos  = (angle_mech_rad / TWO_PI) * 1024.0f;
        int   lut_index = (int)lut_pos % 1024;
        if (lut_index < 0) lut_index += 1024;   /* Guard against negative modulo */

        Ecce_LUT[lut_index] = error;

        if (debug_enable)
        {
            char msg[64];
            snprintf(msg, sizeof(msg),
                     " lut_index: %d , error: %.4f rad \r\n", lut_index, error);
            USART_SendString(msg);
        }
    }

    /* 7. Fill any empty bins by averaging neighbours.*/
    Clean_Eccentricity_LUT(Ecce_LUT);

//    Average the boundary points to smooth the 1023→0 transition
//    float boundary_avg = (Ecce_LUT[0] + Ecce_LUT[1023]) * 0.5f;
//    Ecce_LUT[0]    = boundary_avg;
//    Ecce_LUT[1023] = boundary_avg;

    /* 8. Kill voltage output */
    LL_TIM_OC_SetCompareCH1(TIM1, PWM_ARR / 2);
    LL_TIM_OC_SetCompareCH2(TIM1, PWM_ARR / 2);
    LL_TIM_OC_SetCompareCH3(TIM1, PWM_ARR / 2);

    USART_SendString("\r\n Eccentricity Calibration Complete \r\n");
}


/* CLEAN LUT — SIMPLE NEIGHBOUR AVERAGE */

void Clean_Eccentricity_LUT(volatile float* lut)
{
    int valid_count = 0;
    for (int j = 0; j < 1024; j++) if (lut[j] != 0.0f) valid_count++;

    if (valid_count < 2)
    {
        USART_SendString("Error: Calibration failed, not enough data!\r\n");
        return;
    }

    /* Two passes to handle back-to-back empty bins */
    for (int pass = 0; pass < 2; pass++)
    {
        for (int i = 0; i < 1024; i++)
        {
            if (lut[i] == 0.0f)
            {
                int last_i = (i - 1 + 1024) % 1024;
                int next_i = (i + 1) % 1024;
                lut[i] = (lut[last_i] + lut[next_i]) * 0.5f;
            }
        }
    }


    USART_SendString("LUT gap filling complete \r\n");
}


/* ===================== CLOSED-LOOP CONTROL ===================== */

void Run_Closed_Loop(void)
{
    static volatile uint8_t first_run = 1;
    static volatile float   angle_mech_prev = 0.0f;

    /* 1. Read raw encoder */
    uint16_t angle_raw      = AS5600_ReadAngle_Blocking();
    float    angle_mech_raw = AS5600_Angle_To_Radians(angle_raw);

    /* 2. Look up eccentricity correction from LUT
     *    Index by actual sensor position — same as calibration build */
    float lut_pos  = (angle_mech_raw / TWO_PI) * 1024.0f;
    int   i_low    = (int)lut_pos % 1024;
    if (i_low < 0) i_low += 1024;
    int   i_high   = (i_low + 1) % 1024;
    float fraction = lut_pos - floorf(lut_pos);

    /* Interpolate correction between neighbouring LUT entries */
    float diff = Ecce_LUT[i_high] - Ecce_LUT[i_low];
    /* Unwrap diff across ±π boundary */
    if (diff >  (float)M_PI) diff -= TWO_PI;
    if (diff < -(float)M_PI) diff += TWO_PI;
    float correction = Ecce_LUT[i_low] + (fraction * diff);

    /* 3. Compute raw electrical angle and apply LUT correction   */
    float e_angle_raw = (angle_mech_raw * POLE_PAIRS) - encoder_offset;
    float theta_electrical = e_angle_raw + correction;

    /* Normalise to [0, 2π) using floor unwrap */
    theta_electrical -= TWO_PI * floorf(theta_electrical / TWO_PI);

    //float angle_mech_rad = angle_mech_raw + (correction/POLE_PAIRS);
    float angle_mech_rad = angle_mech_raw;

    if (first_run)
    {
        angle_mech_prev = angle_mech_rad;
        first_run = 0;
        return;
    }

    /* 4. Speed estimation using mechanical angle */
    float delta_mech = angle_mech_rad - angle_mech_prev;

    /* Unwrap delta */
    if      (delta_mech >  (float)M_PI) delta_mech -= TWO_PI;
    else if (delta_mech < -(float)M_PI) delta_mech += TWO_PI;

    speed_raw_rpm = delta_mech * 9549.29f;
    speed_rpm     = LPF_Update(speed_raw_rpm, &speed_filtered);
    angle_mech_prev = angle_mech_rad;

    /* 5. PI speed controller */
    speed_error = speed_ref_rpm - speed_rpm;
    Vq_cmd      = PI_Update(speed_error);
    Vd_cmd      = 0.0f;

    /* 6. Inverse Park (Vd=0 simplified) */
    float cos_theta = cos_from_lut(theta_electrical);
    float sin_theta = sin_from_lut(theta_electrical);

    float Valpha = -Vq_cmd * sin_theta;
    float Vbeta  =  Vq_cmd * cos_theta;

    /* 7. Inverse Clarke */
    float Va =  Valpha;
    float Vb = -0.5f * Valpha + 0.866025f * Vbeta;
    float Vc = -0.5f * Valpha - 0.866025f * Vbeta;

    /* 8. Normalise to duty cycle and write CCR */
    float Vdc_half = VDC_VOLTAGE * 0.5f;

    float duty_a = Va / Vdc_half;
    float duty_b = Vb / Vdc_half;
    float duty_c = Vc / Vdc_half;

    if (duty_a >  1.0f) duty_a =  1.0f;
    if (duty_a < -1.0f) duty_a = -1.0f;
    if (duty_b >  1.0f) duty_b =  1.0f;
    if (duty_b < -1.0f) duty_b = -1.0f;
    if (duty_c >  1.0f) duty_c =  1.0f;
    if (duty_c < -1.0f) duty_c = -1.0f;

    pwm_a = (uint16_t)((duty_a + 1.0f) * 0.5f * PWM_ARR);
    pwm_b = (uint16_t)((duty_b + 1.0f) * 0.5f * PWM_ARR);
    pwm_c = (uint16_t)((duty_c + 1.0f) * 0.5f * PWM_ARR);

    LL_TIM_OC_SetCompareCH1(TIM1, pwm_a);
    LL_TIM_OC_SetCompareCH2(TIM1, pwm_b);
    LL_TIM_OC_SetCompareCH3(TIM1, pwm_c);
}


/*================= AS5600 Encoder Configuration I2C ======================*/

float AS5600_Angle_To_Radians(uint16_t angle_12bit)
{
    return (float)angle_12bit * TWO_PI / 4096.0f;
}

float AS5600_Radians_To_Angle(float angle_rad)
{
	return(float)angle_rad*360/TWO_PI;
}

uint16_t AS5600_ReadAngle_Blocking(void)
{
    uint8_t highByte, lowByte;
    uint16_t timeout;

    // Bus recovery if stuck
    if (LL_I2C_IsActiveFlag_BUSY(I2C1))
    {
        LL_I2C_GenerateStopCondition(I2C1);
        for (volatile uint32_t i = 0; i < 1000; i++);
    }

    LL_I2C_HandleTransfer(I2C1, AS5600_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

    timeout = 8500;
    while (!LL_I2C_IsActiveFlag_TXIS(I2C1) && --timeout);
    if (timeout == 0)
    {
        i2c_error_count++;
        i2c_last_error = 1;
        if (closedloop_enable)
        {
        	system_state = STATE_FAULT;  // ← Enter fault state
        }
        LL_I2C_ClearFlag_STOP(I2C1);
        LL_I2C_GenerateStopCondition(I2C1);
        return current_angle;
    }
    LL_I2C_TransmitData8(I2C1, 0x0E);

    timeout = 8500;
    while (!LL_I2C_IsActiveFlag_TC(I2C1) && --timeout);
    if (timeout == 0)
    {
        i2c_error_count++;
        i2c_last_error = 2;
        if (closedloop_enable)
        {
        	system_state = STATE_FAULT;  // ← Enter fault state
        }
        LL_I2C_ClearFlag_STOP(I2C1);
        return current_angle;
    }

    LL_I2C_HandleTransfer(I2C1, AS5600_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    timeout = 8500;
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1) && --timeout);
    if (timeout == 0)
    {
        i2c_error_count++;
        i2c_last_error = 3;
        if (closedloop_enable)
        {
        	system_state = STATE_FAULT;  // ← Enter fault state
        }
        LL_I2C_ClearFlag_STOP(I2C1);
        return current_angle;
    }
    highByte = LL_I2C_ReceiveData8(I2C1);

    timeout = 8500;
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1) && --timeout);
    if (timeout == 0)
    {
        i2c_error_count++;
        i2c_last_error = 4;
        if (closedloop_enable)
        {
        	system_state = STATE_FAULT;  // ← Enter fault state
        }
        LL_I2C_ClearFlag_STOP(I2C1);
        return current_angle;
    }
    lowByte = LL_I2C_ReceiveData8(I2C1);

    timeout = 8500;
    while (!LL_I2C_IsActiveFlag_STOP(I2C1) && --timeout);
    if (timeout == 0)
    {
        i2c_error_count++;
        i2c_last_error = 5;
        if (closedloop_enable)
        {
        	system_state = STATE_FAULT;  // ← Enter fault state
        }
        LL_I2C_ClearFlag_STOP(I2C1);
        return current_angle;
    }
    LL_I2C_ClearFlag_STOP(I2C1);

    current_angle = ((uint16_t)(highByte & 0x0F) << 8) | lowByte;
    return current_angle;
}


/* ===================== PI CONTROLLER  ===================== */

void PI_Init(float kp, float ki, float dt, float out_min, float out_max)
{
    PI_Kp = kp;
    PI_Ki = ki;
    PI_dt = dt;

    PI_integral = 0.0f;

    PI_output_min = out_min;
    PI_output_max = out_max;

    // Integral limits
    PI_integral_min = out_min;
    PI_integral_max = out_max;
}

void PI_Reset(void)
{
    PI_integral = 0.0f;
}

float PI_Update(float error)
{
//    // Proportional term
//    float P_term = PI_Kp * error;
//
//    // Integral term with anti-windup
//    PI_integral += error * PI_dt;
//
//    // Clamp integral to prevent windup
//    if (PI_integral > PI_integral_max)
//        PI_integral = PI_integral_max;
//    if (PI_integral < PI_integral_min)
//        PI_integral = PI_integral_min;
//
//    float I_term = PI_Ki * PI_integral;
//
//    // Calculate output
//    float output = P_term + I_term;
//
//    // Saturate output
//    if (output > PI_output_max)
//        output = PI_output_max;
//    if (output < PI_output_min)
//        output = PI_output_min;

	// Proportional Term
	float P_term = PI_Kp * error;

	// Integral Term - we multiply by Ki *before* adding to the integrator
	// This makes the 'integrator' variable actually represent Volts (or Current)
	PI_integral += (PI_Ki * error * PI_dt);

	// Clamping the Integrator (Anti-Windup)
	// Limits should match your VDC/2 (e.g., -6V to 6V)
	if (PI_integral > PI_output_max) PI_integral = PI_output_max;
	else if (PI_integral < PI_output_min) PI_integral = PI_output_min;

	// Sum
	float output = P_term + PI_integral;

	// Final Output Saturation
	if (output > PI_output_max) output = PI_output_max;
	else if (output < PI_output_min) output = PI_output_min;

    return output;
}

void PI_Set_Gains(float kp, float ki)
{
    PI_Kp = kp;
    PI_Ki = ki;
}

void PI_Set_SampleTime(float dt)
{
    PI_dt = dt;
}
