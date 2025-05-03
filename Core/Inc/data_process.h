/*
 * data_process.h
 *
 *  Created on: May 3, 2025
 *      Author: YiMingTuan
 */

#ifndef INC_DATA_PROCESS_H_
#define INC_DATA_PROCESS_H_

#include "main.h"
#include "stdio.h"
#include "stdbool.h"

#define SCREEN_TRIGGERS         4

#define SCREEN_SIZE             200

#define SCREEN_DATA_SIZE		800		   // SCREEN_TRIGGERS * SCREEN_SIZE
#define ADC_INTERMEDIATE_SIZE	2
#define ADC_BUFFER_SIZE			14400	   // ADC_SAMPLING_RATE * ADC_BUFFER_TIME / 1000

#define ADC_SAMPLING_RATE       7200000    // Hz
#define ADC_BUFFER_TIME    		2          // ms
#define ADC_RESOLUTION          8          // bits
#define ADC_VOLTAGE_MAX         3.3        // V
#define ADC_VOLTAGE_MIN         0		   // V


extern volatile bool start_finding_trigger;     // indicate the task start finding trigger, adc-dma end control
extern volatile bool screen_data_ready;         // control by the program task, send signal to let the screen side to know ok to read data
extern volatile bool screen_measure_ready;
extern volatile bool is_adc_buffer_first_half_active;      // indicate which half of adc buffer is active, adc-dma end control
extern volatile bool trigger_found;

// extern trigger_mode_enum trigger_mode = TRIGGER_MODE_FALLING;

extern int time_scale;             // capture 1 data from every n points

extern uint8_t adc_buffer[ADC_BUFFER_SIZE];
extern uint8_t adc_intermediate[ADC_INTERMEDIATE_SIZE][SCREEN_SIZE];   // should initialize
extern int8_t screen_data[SCREEN_DATA_SIZE];

extern uint8_t trigger_level;    // 0-255, 128 is the middle level
extern uint8_t offset;

// screen end know where to read the 200 signed data
extern uint8_t* screen_data_start_ptr;
extern uint8_t* screen_frame_start_ptr;
extern uint8_t* screen_frame_end_ptr;		// closed
extern uint8_t* screen_measure_ptr;

extern double v_pp_output;
extern double frequency_output;
extern double period_output;

extern int adc_intermediate_index;

void captureData(void);
void measure(void);

#endif /* INC_DATA_PROCESS_H_ */
