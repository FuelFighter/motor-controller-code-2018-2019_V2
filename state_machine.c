/*
 * state_machine.c
 *
 * Created: 22/04/2018 16:00:41
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : not hardware specific
 */
#include <stdlib.h>
#include <avr/io.h>
#include "state_machine.h"
#include "controller.h"
#include "speed.h"

#define MAX_VOLT 55.0
#define MIN_VOLT 15.0
#define MAX_AMP 10000 //25.0
#define MAX_TEMP 100

static uint8_t b_major_fault = 0;
//static uint8_t fault_count = 0;
//static uint16_t fault_timeout = 0;
//static uint8_t fault_clear_count = 0;
static uint8_t starting_engage = 0;
//static uint8_t engage_count = 0;
uint16_t actuator_target_position = 0; 

void state_handler(volatile ModuleValues_t * vals)
{	
	//THIS NEEDS TO BE SORTED THE FUCK OUT!!!!
	// The SAFETY code below needs to be reimplemented and tested before use! I did not have the possibiliy to test in the car until later, so I commented the code. See the code in  (Motor-drive-2018) in GitHub as a reference to fix it. Sorry to the poor fucker who has to deal with this hahahahaha.
	
	uint8_t b_board_powered = 1;
	/* 
	if ((vals->f32_batt_volt >= 25) && (vals->f32_batt_volt < 55))
	{
		b_board_powered = 1;
		vals->board_powered = 1;
	}
	else
	{
		b_board_powered = 0;
		vals->board_powered = 0;
	}

	//vals->f32_motor_current >= MAX_AMP || vals->f32_motor_current <= -MAX_AMP || vals->f32_batt_volt > 55 || vals->f32_batt_volt < 30 || " && (vals->pwtrain_type == NOT_SELECTED) "
	if (((b_board_powered == 1) && (vals->pwtrain_type == NOT_SELECTED)))
	{
		if (fault_count < 20)
		{
				fault_count ++ ;
		}
	}
	
	if (fault_count > 10) // a fault is cleared after some time and a maximum of three times. If the fault occurs more than three times,
							//the board needs to be reset. there is a real problem to be investigated.
	{
		b_major_fault = 1;
		fault_timeout = 600; 
	}
	
	if (fault_timeout > 0)
	{
		fault_timeout -- ;
	}
	
	if ((b_major_fault) && (fault_timeout <= 3))
	{
		b_major_fault = 0;
		fault_count = 0;
	}
	*/
	
	//uart is not spamming the UM so must reload watchdog timer here
	if(vals->message_mode == UART){
		vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE;
		vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE;
	} 

	switch(vals->motor_status)
	{
		case OFF:
			//transition 1 (see documentation for the state machine description)
			if (vals->u16_watchdog_can > 0 && b_board_powered)
			{
				vals->motor_status = IDLE;
			}
			//During
			drivers(0);//drivers shutdown
			vals->b_driver_status = 0;
			reset_I(); //reset integrator
			vals->u8_brake_cmd = 0;
			vals->u8_accel_cmd = 0;
			vals->u8_duty_cycle = 50;
			vals->gear_required = NEUTRAL ;
		
		break;
		
		case IDLE: 
		
			if (vals->pwtrain_type == BELT)
			{
				//controller(vals);
				drivers(0); //disable
				reset_I();
				vals->u8_duty_cycle = 50 ;
				
				//transition 7
				if (vals->u8_brake_cmd > 0)
				{
					vals->u8_duty_cycle = compute_synch_duty(vals->u16_car_speed, GEAR2, vals->f32_batt_volt) ; //Setting duty
					set_I(vals->u8_duty_cycle) ; //set integrator
					vals->motor_status = BRAKE;
				}
				//transition 5
				if (vals->u8_accel_cmd > 0)
				{
					vals->u8_duty_cycle = compute_synch_duty(vals->u16_car_speed, GEAR2, vals->f32_batt_volt) ; //Setting duty
					set_I(vals->u8_duty_cycle) ; //set integrator
					vals->motor_status = ACCEL;
				}
			}
			
			if (vals->pwtrain_type == GEAR)
			{
				//transition 5
				if ((vals->u8_accel_cmd > 0 || vals->u8_brake_cmd > 0) && vals->gear_status == NEUTRAL)
				{
					vals->motor_status = ENGAGE;
					starting_engage = 1;
				}
				
				drivers(0); //disable
				vals->gear_required = NEUTRAL ;
				reset_I();
				vals->u8_duty_cycle = 50 ;
			}
			
		break;
		
		case ENGAGE: // /!\ TODO : with the two gears, all turning motion has to be inverted for the inner gear.
			
			vals->gear_required = GEAR1;
			if (starting_engage)
			{
				vals->u8_duty_cycle = compute_synch_duty(vals->u16_car_speed, vals->gear_required, vals->f32_batt_volt) ; //Setting duty
				set_I(vals->u8_duty_cycle) ; //set integrator
				starting_engage = 0;
			}
			//save_ctrl_type = vals->ctrl_type ; // PWM type ctrl is needed only for the engagement process. The mode will be reverted to previous in ACCEL and BRAKE modes
			vals->ctrl_type = PWM ;
			controller(vals) ; //speed up motor to synch speed
			drivers(1);
			
			//*****************--------------------------------TESTING START--------------------------*****************
			/*
			switch(vals->gear_required){
				case NEUTRAL:
				actuator_target_position = vals->position_neutral;
				break;
				
				case GEAR1:
				actuator_target_position = vals->position_gear_1;
				break;
				
				case GEAR2:
				actuator_target_position = vals->position_gear_2;
				break;
			}
			while (vals->f32_actuator_feedback < (float)0.75*actuator_target_position) {
				//do absolutely nothing 
			}
			
			//cut power to motor
			vals->u8_duty_cycle = 50;
			controller(vals);
			
			//create delay
			while (engage_count != 400) {
				engage_count ++;
			}
			engage_count = 0;
			
			*/
			//*****************--------------------------------TESTING END--------------------------*****************
			
			//transition 9, GEAR
			if (vals->u8_brake_cmd > 0 && vals->gear_status == GEAR1)
			{
				vals->motor_status = BRAKE;
			}
			//transition 10, GEAR
			if (vals->u8_accel_cmd > 0 && vals->gear_status == GEAR1)
			{
				vals->motor_status = ACCEL;
			}
			//transition 11, GEAR
			if (vals->u8_accel_cmd == 0 && vals->u8_brake_cmd == 0 && vals->u16_watchdog_throttle == 0)
			{
				vals->motor_status = IDLE;
			}
		break;
		
		case ACCEL:			
			vals->ctrl_type = CURRENT;
			controller(vals);
			drivers(1);
			//transition 6
			if (vals->u8_accel_cmd == 0 && vals->u16_watchdog_throttle == 0)
			{
				vals->motor_status = IDLE;
			}
			//transition 12, GEAR
			if (vals->pwtrain_type == GEAR && vals->gear_status == NEUTRAL)
			{
				vals->motor_status = ENGAGE;
				starting_engage = 1;
			}
			//transition 14
			if (vals->u8_brake_cmd > 0 && vals->u8_accel_cmd == 0)
			{
				vals->motor_status = BRAKE;
			}
		break;
		
		case BRAKE:
			vals->ctrl_type = CURRENT ;
			controller(vals); //negative throttle cmd
			drivers(1);
			//transition 8
			if (vals->u8_brake_cmd == 0 && vals->u16_watchdog_throttle == 0)
			{
				vals->motor_status = IDLE;
			}
			//transition 13, GEAR
			if (vals->pwtrain_type == GEAR && vals->gear_status == NEUTRAL)
			{
				vals->motor_status = ENGAGE;
				starting_engage = 1;
			}
			//transition 15
			if (vals->u8_brake_cmd == 0 && vals->u8_accel_cmd > 0)
			{
				vals->motor_status = ACCEL;
			}
		break;
		
		case ERR:
			//transition 4
			//ATTENTION: "TAKEN OUT": && vals->u8_motor_temp < MAX_TEMP
			//TODO: Implement what the actuator should do when in error mode 
			if (!b_major_fault)
			{
				vals->motor_status = IDLE;
			}
			drivers(0);//drivers shutdown
			vals->b_driver_status = 0;
			vals->gear_required = NEUTRAL;
			reset_I(); //reset integrator
			vals->u8_brake_cmd = 0;
			vals->u8_accel_cmd = 0;
			vals->u8_duty_cycle = 50;
		break;	
	}
	
	if ((vals->motor_status == IDLE || vals->motor_status == ACCEL || vals->motor_status == BRAKE || vals->motor_status == ENGAGE) && (vals->u16_watchdog_can == 0 || !b_board_powered))
	{
		// transition 2
		vals->motor_status = OFF;
	}
	
	//ATTENTION: "TAKEN OUT": COMMENTED OUT " || vals->u8_motor_temp >= MAX_TEMP) "
	if (b_major_fault) //over current, over voltage, over temp
	{
		//transition 3
		vals->motor_status = ERR;
	}
}
