/*
 * actuators.c
 *
 * Created: 07/03/2019 
 * Author : Tyler Harrison for DNV GL Fuel fighter
 * Corresponding Hardware : motor_control_v2.0
 
 *INPUTS:	1) S0 feedback from ADC
			2) gear_required: the next state/gear of the clutch 
			3)gear_status: current state/gear that the clutch is in
			
 *OUTPUTS: none. 
 
 This file serves the co-operation between the main program and the linear actuator 
 ACTUATOR CONTROL: P-controller to maintain the actuators target position from "S0" feedback 
 */

#include <avr/io.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "actuator.h"

#define POSITION_TOLERANCE	5
#define KP 3.8
#define HALF_DUTY_CYCLE 66.5


//Variable description & structures
volatile ActuatorModuleValues_t ActuatorComValues = {
	.clutch_state = 0,
	.actuator_direction = STATIONARY, 
	.actuator_in_position = 0,
	.actuator_position_error = 0,
	.actuator_duty_cycle = 0,
	.position_neutral = 0,
	.position_gear_1 = 0,
	.position_gear_2 = 0,
	.power_train_type = BELT
};

//FUNCTION: ACTUATOR: Set saved actuator positions
void actuator_init(volatile ModuleValues_t * vals)
{
	//Turn actuators on
	actuator_pwm(1);
	
	//set actuator positions
	vals->position_neutral = eeprom_read_word((uint16_t*)EEPROM_NEUTRAL);
	vals->position_gear_1 = eeprom_read_word((uint16_t*)EEPROM_GEAR_1);
	vals->position_gear_2 = eeprom_read_word((uint16_t*)EEPROM_GEAR_2);
	
	//Initialise Actuator variables 
	ActuatorComValues.clutch_state = vals->gear_status;
	ActuatorComValues.actuator_direction = vals->actuator_direction;
	ActuatorComValues.actuator_duty_cycle = vals->u8_actuator_duty_cycle; 
	ActuatorComValues.position_neutral = vals->position_neutral;
	ActuatorComValues.position_gear_1 = vals->position_gear_1;
	ActuatorComValues.position_gear_2 = vals->position_gear_2;
	ActuatorComValues.power_train_type = vals->pwtrain_type;
	
	//ACTUATOR: set/lock the actuator, start PWM signal and enable PCB signal inverter with 3V3
	//actuator_pwm(int enable);
}

void actuator_update(volatile ModuleValues_t * vals)
{
	//Check what power train is selected
	
	
	//Update ComValues in the main program with the local structure ActuatorModuleValues_t
	vals->gear_status = ActuatorComValues.clutch_state;
	vals->actuator_direction = ActuatorComValues.actuator_direction;
	vals->actuator_in_position = ActuatorComValues.actuator_in_position;
	vals->actuator_position_error = ActuatorComValues.actuator_position_error;
	vals->u8_actuator_duty_cycle = ActuatorComValues.actuator_duty_cycle;
	vals->position_neutral = ActuatorComValues.position_neutral;
	vals->position_gear_1 = ActuatorComValues.position_gear_1;
	vals->position_gear_2 = ActuatorComValues.position_gear_2;
	//vals->pwtrain_type = ActuatorComValues.power_train_type;
}

//FUNCTION: saves the actuator/clutch position or S0 value in the micros eeprom memory
void actuator_save_position(ClutchState_t gear_required, ClutchState_t gear_status, int16_t position_uart_instruction, uint16_t position_neutral, uint16_t position_gear_1, uint16_t position_gear_2)
{
	//make sure that "GEAR" is checked before calling function in digicom.c
	//ASSUMPTION THAT THE ACTUATOR IS NOT MOVING!!! Do we need a function to check if actuator is moving? Check actuator_duty_cycle or Actuator 

	switch(gear_required){
		
		case NEUTRAL:
			eeprom_write_word ((uint16_t *)EEPROM_NEUTRAL, position_uart_instruction);
			ActuatorComValues.position_neutral = position_uart_instruction;
			ActuatorComValues.clutch_state = NEUTRAL;
		break;
		
		case GEAR1:
			eeprom_write_word ((uint16_t *)EEPROM_GEAR_1, position_uart_instruction);
			ActuatorComValues.position_gear_1 = position_uart_instruction;
			ActuatorComValues.clutch_state = GEAR1;
		break;
		
		case GEAR2:
			eeprom_write_word ((uint16_t *)EEPROM_GEAR_2, position_uart_instruction);
			ActuatorComValues.position_gear_2 = position_uart_instruction;
			ActuatorComValues.clutch_state = GEAR2;
		break;
	}
}

void actuator_pwm(int enable)
{
	//turn the 3V3 regulator off in motor_control_v2.1 PCB 
	//cannot "unlock" actuators in motor_control_v2.0 as inverter is contineously on and will invert a logic low and drive the actuator in one direction.
	if(enable){
		//PWM: turn pwm ON: enable 3V3 reg
		//TCCR3C = 1;
		//PORTE |= (1<<PE5);	
		
		//ACTUATOR PWM:
		// Set OC3C on Compare Match when up-counting. Clear OC3B on Compare Match when downcounting.
		TCCR3A |= (1<<COM3C1);
		TCCR3A &= ~(1<<COM3C0);
		
		//turn PF3 ON to enable 3V3 
		DDRF |= (1<<PF3);
		PORTF |= (1<<PF3);
	
	}else{
		//PWM: turn pwm OFF: disable 3V3 reg
		TCCR3C = 0;
		PORTE &= ~(1<<PE5);
		
		//turn 3V3 regulator off
		PORTF &= ~(1<<PF3);			
	}
}

int actuator_position_tolerance(float position_error)
{
	if((position_error < POSITION_TOLERANCE) && (-position_error < POSITION_TOLERANCE))
	{
		return 1;	
	}else
	{
		return 0;
	}	
} 


void actuator_set_position(volatile ActuatorModuleValues_t *actuator_values, ClutchState_t gear_required, float uart_debug, int16_t actuator_duty_cycle, uint16_t target_position, float f32_actuator_feedback)
{
/*OUTPUTS:	1) EXTEND, RETRACT or STATIONARY "state"
			2) Position_ERROR
			3) actuator_duty_cycle
			4) gear_status
*/

	//float kp = 3.8; //ATTENTION: Change kp to produce a bigger duty cycle for a given error value
	int16_t position_error = ((int16_t)target_position - (int16_t)f32_actuator_feedback);
	int16_t new_duty_cycle = 0;
	new_duty_cycle = (float)KP*position_error + (float)HALF_DUTY_CYCLE;
	
	if (actuator_position_tolerance(position_error)) 
	{
		actuator_values->actuator_in_position = 1;
		actuator_values->clutch_state = gear_required;
		//new_duty_cycle = 50;
	} else
	{
		actuator_values->actuator_in_position = 0;
	}
	
	//Check actuator state
	switch (actuator_values->actuator_in_position) {
		case (1):
			actuator_values->actuator_direction = STATIONARY;
		break;
		
		case (0):
			if (position_error > 0) 
			{
				//target is bigger than the actuators current position
				actuator_values->actuator_direction = EXTEND;
			} 
			else if (position_error < 0)
			{
				actuator_values->actuator_direction = RETRACT;
			}
		break;
	}
	
	if(new_duty_cycle > 130)
	{
		new_duty_cycle = 130;
	}
	if (new_duty_cycle < 0)
	{
		new_duty_cycle = 0;
	}
	
	OCR3C = new_duty_cycle;
	
	actuator_values->actuator_duty_cycle = new_duty_cycle;
	actuator_values->actuator_position_error = position_error;
}

void actuator_p_controller(volatile ModuleValues_t * vals)
{
	uint16_t target_position = 0;
	
	//vals->uart_debug = (int16_t)vals->uart_debug;
	
	if(vals->clutch_enabled)
	{
		//ACTUATOR: set actuator position based off current state
		switch(vals->gear_required)
		{
			case NEUTRAL:
					target_position = vals->position_neutral;
				break;
				
			case GEAR1:
					target_position = vals->position_gear_1;
				break;
				
			case GEAR2: //ATTENTION: this is belt mode... as currently implemented
					target_position = vals->position_gear_2;
				break;
		}
	
		actuator_set_position(&ActuatorComValues, vals->gear_required, vals->uart_debug_1, vals->u8_actuator_duty_cycle, target_position, vals->f32_actuator_feedback);
		
	}else
	{
		//move with uart instruction
		target_position = vals->position_uart_instruction;
		actuator_set_position(&ActuatorComValues, vals->gear_required,  vals->uart_debug_1, vals->u8_actuator_duty_cycle, target_position, vals->f32_actuator_feedback);

	}
}


