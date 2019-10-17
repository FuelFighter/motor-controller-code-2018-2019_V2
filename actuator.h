/*
 * actuators.h
 *
 * Created: 07/03/2019 
 * Author : Tyler Harrison for DNV GL Fuel fighter
 * Corresponding Hardware : motor_control_v2.0
 
 This file serves the co-operation between the main program and the linear actuator
 */ 
#include "state_machine.h"

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

typedef enum {
	EEPROM_GEAR_1 = 46,
	EEPROM_GEAR_2 = 44,
	EEPROM_NEUTRAL = 42
} EepromLocations;

typedef struct {
	int clutch_state;
	int actuator_direction;
	int actuator_in_position;
	float actuator_position_error;
	int16_t actuator_duty_cycle;
	uint16_t position_neutral;
	uint16_t position_gear_1;
	uint16_t position_gear_2;
	PowertrainType_t power_train_type;
	
} ActuatorModuleValues_t;

//FUNCTIONS	
void actuator_init(volatile ModuleValues_t * vals);
void actuator_p_controller(volatile ModuleValues_t * vals);
void actuator_save_position(ClutchState_t gear_required, ClutchState_t gear_status, int16_t position_uart_instruction, uint16_t position_neutral, uint16_t position_gear_1, uint16_t position_gear_2);
void actuator_pwm(int start);
void actuator_set_position(volatile ActuatorModuleValues_t *actuator_values, ClutchState_t gear_required,  float uart_debug, int16_t actuator_duty_cycle, uint16_t target_position, float f32_actuator_feedback);
void actuator_update(volatile ModuleValues_t * vals);

#endif