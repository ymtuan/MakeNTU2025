/*
 * data_process.c
 *
 *  Created on: May 3, 2025
 *      Author: YiMingTuan
 */

#include "../inc/data_process.h"

typedef enum
{
    TRIGGER_MODE_RISING,
    TRIGGER_MODE_FALLING,
    TRIGGER_MODE_BOTH
} trigger_mode_enum;

volatile bool start_finding_trigger;     // indicate the task start finding trigger, adc-dma end control
volatile bool screen_data_ready;         // control by the program task, send signal to let the screen side to know ok to read data
volatile bool screen_measure_ready;
volatile bool is_adc_buffer_first_half_active;      // indicate which half of adc buffer is active, adc-dma end control
volatile bool trigger_found;

trigger_mode_enum trigger_mode = TRIGGER_MODE_FALLING;

int time_scale = 1;             // capture 1 data from every n points

uint8_t adc_buffer[ADC_BUFFER_SIZE];
uint8_t adc_intermediate[ADC_INTERMEDIATE_SIZE][SCREEN_SIZE] = {0};   // should initialize
int8_t screen_data[SCREEN_DATA_SIZE];

uint8_t* adc_intermediate_start_ptr;
uint8_t* adc_intermediate_end_ptr;		// closed

int adc_intermediate_index = 0;
uint8_t* adc_intermediate_ptr;
int half_screen_size = SCREEN_SIZE / 2;
int half_adc_buffer_size = ADC_BUFFER_SIZE / 2;

uint8_t trigger_level = 128;    // 0-255, 128 is the middle level
uint8_t offset = 128;

uint8_t prev_val;
uint8_t curr_val;

uint8_t* adc_buffer_read_start_ptr;
uint8_t* adc_buffer_read_end_ptr;
uint8_t* adc_buffer_read_ptr;
uint8_t* adc_buffer_after_trigger_read_end_ptr;

uint8_t* adc_buffer_read_find_trigger_start_ptr;
uint8_t* adc_buffer_read_find_trigger_end_ptr;

// screen end know where to read the 200 signed data
uint8_t* screen_data_start_ptr;
uint8_t* screen_frame_start_ptr;
uint8_t* screen_frame_end_ptr;		// closed
uint8_t* screen_measure_ptr;


double v_pp_output;
double frequency_output;
double period_output;

int trigger_crossing_count;
double screen_frame_time;

int measure_loop_index;
uint8_t max_val;
uint8_t min_val;

const uint8_t zero_val = 0;



void captureData(void)
{
    if (is_adc_buffer_first_half_active)
    {
        adc_buffer_read_start_ptr = &adc_buffer[0];
        adc_buffer_read_end_ptr = &adc_buffer[half_adc_buffer_size];
    }
    else
    {
        adc_buffer_read_start_ptr = &adc_buffer[half_adc_buffer_size];
        adc_buffer_read_end_ptr = &adc_buffer[ADC_BUFFER_SIZE];
    }

    adc_buffer_read_find_trigger_start_ptr = adc_buffer_read_start_ptr +  time_scale * half_screen_size;
    adc_buffer_read_find_trigger_end_ptr = adc_buffer_read_end_ptr - time_scale * half_screen_size;

    adc_intermediate_start_ptr = adc_intermediate[adc_intermediate_index];
    adc_intermediate_end_ptr = adc_intermediate_start_ptr + SCREEN_SIZE - 1;

    screen_frame_start_ptr = adc_intermediate_start_ptr;
    screen_frame_end_ptr = adc_intermediate_end_ptr;

    adc_intermediate_ptr = adc_intermediate_start_ptr;
    adc_buffer_read_ptr = adc_buffer_read_start_ptr;

    curr_val = *adc_buffer_read_ptr;
    prev_val = curr_val;
    *adc_intermediate_ptr = prev_val;
    adc_intermediate_ptr++;
    adc_buffer_read_ptr += time_scale;

    trigger_found = false;

    while (adc_buffer_read_ptr < adc_buffer_read_end_ptr)
    {
    	if (adc_buffer_read_ptr < adc_buffer_read_find_trigger_start_ptr)
    	{
    		curr_val = *adc_buffer_read_ptr;
    		*adc_intermediate_ptr = curr_val;
    		prev_val = curr_val;
    		adc_intermediate_ptr = adc_intermediate_ptr == adc_intermediate_end_ptr ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;
    		adc_buffer_read_ptr += time_scale;
    		continue;
    	}

        curr_val = *adc_buffer_read_ptr;
        *adc_intermediate_ptr = curr_val;


        switch (trigger_mode)
        {
            case TRIGGER_MODE_RISING :
                trigger_found = (prev_val <= trigger_level && curr_val > trigger_level);
                break;
            case TRIGGER_MODE_FALLING :
                trigger_found = (prev_val >= trigger_level && curr_val < trigger_level);
                break;
            case TRIGGER_MODE_BOTH :
                trigger_found = (prev_val <= trigger_level && curr_val > trigger_level) || (prev_val >= trigger_level && curr_val < trigger_level);
                break;
        }

        if (trigger_found)
        {
            // screen_data_start_ptr = adc_intermediate_ptr;

            adc_buffer_after_trigger_read_end_ptr = adc_buffer_read_ptr + (time_scale * half_screen_size);

            adc_buffer_read_ptr = adc_buffer_read_ptr + time_scale;

            adc_intermediate_ptr = (adc_intermediate_ptr == adc_intermediate_end_ptr) ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;

            for (; adc_buffer_read_ptr < adc_buffer_after_trigger_read_end_ptr; adc_buffer_read_ptr += time_scale)
            {
                *adc_intermediate_ptr = *adc_buffer_read_ptr;
                adc_intermediate_ptr = (adc_intermediate_ptr == adc_intermediate_end_ptr) ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;
            }

            screen_data_start_ptr = adc_intermediate_ptr;
            screen_data_ready = true;
            break;
        }

        prev_val = curr_val;
        adc_intermediate_ptr = adc_intermediate_ptr == adc_intermediate_end_ptr ? adc_intermediate_start_ptr : adc_intermediate_ptr + 1;
        adc_buffer_read_ptr += time_scale;
    }

    if (!trigger_found)
    {
    	// fail to find any trigger in this half of adc buffer
    }

}


void measure(void)
{
    trigger_crossing_count = 0;
    screen_measure_ptr = screen_data_start_ptr;
    prev_val = *screen_measure_ptr;
    max_val = prev_val;
    min_val = prev_val;
    screen_measure_ptr = (screen_measure_ptr == screen_frame_end_ptr) ? screen_frame_start_ptr : screen_measure_ptr + 1;

    for (measure_loop_index = 1; measure_loop_index < SCREEN_SIZE; measure_loop_index++)
    {
        curr_val = *screen_measure_ptr;
        if (((prev_val >= trigger_level) && (curr_val < trigger_level)) || ((prev_val <= trigger_level) && (curr_val > trigger_level)))
        {
            trigger_crossing_count++;
        }

        max_val = (curr_val > max_val) ? curr_val : max_val;
        min_val = (curr_val < min_val) ? curr_val : min_val;

        prev_val = curr_val;
        screen_measure_ptr = (screen_measure_ptr == screen_frame_end_ptr) ? screen_frame_start_ptr : screen_measure_ptr + 1;
    }

    v_pp_output = (double)(max_val - min_val) * (ADC_VOLTAGE_MAX - ADC_VOLTAGE_MIN) / 255.0;
    frequency_output = (double)(trigger_crossing_count * ADC_SAMPLING_RATE) / (2 * SCREEN_SIZE * time_scale);
    period_output = 1.0 / frequency_output;

}
