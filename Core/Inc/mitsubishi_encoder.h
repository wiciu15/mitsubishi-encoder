/*
 * mitsubishi_encoder.h
 *
 *  Created on: Feb 15, 2022
 *      Author: Wiktor
 */

#ifndef INC_MITSUBISHI_ENCODER_H_
#define INC_MITSUBISHI_ENCODER_H_

#include "main.h"

extern uint32_t encoder_position;
enum encoder_state_t {encoder_ok, encoder_error_cheksum, encoder_error_acceleration, encoder_error_no_communication};
extern enum encoder_state_t encoder_state;
extern	uint32_t encoder_position_min;
extern uint32_t encoder_position_max;
extern uint32_t low_res_position_divider;
extern uint8_t low_res_position;  //0-80 position
extern uint32_t last_encoder_position;
extern uint8_t checksum_error_count;
extern uint8_t excessive_acceleration_error_count;

extern uint8_t motor_data_response_packet[9];//full response to 0xA2 command
enum encoder_resolution_t {unknown_resolution,p8192ppr,p131072ppr};
extern enum encoder_resolution_t encoder_resolution;
enum motor_family_t {unknown_family,j2,j2super};
extern enum motor_family_t motor_family;
enum motor_formfactor_t {unknown_formfactor,mf,kf};
extern enum motor_formfactor_t motor_formfactor;
extern uint16_t motor_speed;
extern uint16_t motor_power;
extern char motor_model_string [10];

void motor_identification(void);

#endif /* INC_MITSUBISHI_ENCODER_H_ */
