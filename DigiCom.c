/*
 * DigiCom.c
 *
 * Created: 04/03/2018 14:12:59
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 *UART COMMUNICATION: Use either SerialPlotter or Putty to send/receive uart data
					  Send the commands via USB without a whitespace and terminate with \r\n
 */ 

#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include "actuator.h"
#include "DigiCom.h"
#include "sensors.h"
#include "controller.h"
#include "UniversalModuleDrivers/adc.h"
#include "UniversalModuleDrivers/spi.h"
#include "UniversalModuleDrivers/rgbled.h"
#include "AVR-UART-lib-master/usart.h"

//ADC buffers
static uint16_t u16_ADC0_reg = 0;
static uint16_t u16_ADC1_reg = 0;
//static uint16_t u16_ADC2_reg = 0;
//static uint16_t u16_ADC3_reg = 0;
static uint16_t u16_ADC4_reg = 0;
//static uint16_t u16_ADC5_reg = 0;
static uint16_t u16_ADC6_reg = 0;
static uint16_t u16_ADC7_reg = 0;

/*
ADDED CODE FOR MOTOR_CONTROL_V2.0
CH0 - S_B_V, BATTERY VOLTAGE
CH1 - S_B_I, BATTERY CURRENT
CH2 - X, N/C
CH3 - X, N/C
CH4 - S_M_T, MOTOR TEMPERATURE
CH5 - S_S_01, SPARE 
CH6 - S_A_S0, ACTUATOR POSITION FEEDBACK
CH7 - S_M_I, MOTOR CURRENT
*/


//for SPI
static uint8_t u8_txBuffer[2];
static uint8_t u8_rxBuffer[3];

/////////////////////////  SPI  /////////////////////////

void SPI_handler_0(volatile float * f32_batt_volt) // motor current ***CH0 - S_B_V, BATTERY VOLTAGE
{
	Set_ADC_Channel_ext(0, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC0_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	*f32_batt_volt = VOLT_CONVERSION_OFFSET+(((float)u16_ADC0_reg/VOLT_CONVERSION_COEFF)*10 - 17);
}

void SPI_handler_1(volatile float * f32_batt_current) // battery current ***CH1 - S_B_I, BATTERY CURRENT
{
	Set_ADC_Channel_ext(1, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC1_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_current_sensor(f32_batt_current, u16_ADC1_reg, 1);
}
/*
void SPI_handler_2(volatile float * f32_batvolt) //battery voltage ***CH2 - X, N/C
{
	Set_ADC_Channel_ext(2, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC2_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	
}

void SPI_handler_3(volatile float * p_f32_motcurrent) // motor current ***CH3 - X, N/C
{
	Set_ADC_Channel_ext(0, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC0_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	//ATTENTION - IMPLEMENT CONVERSION HERE
	//handle_current_sensor(p_f32_motcurrent, u16_ADC0_reg,0);
}
*/
void SPI_handler_4(volatile uint8_t * u8_motor_temp) //motor temperature ***CH4 - S_M_T, MOTOR TEMPERATURE
{
	Set_ADC_Channel_ext(4, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC4_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	
	handle_temp_sensor(u8_motor_temp, u16_ADC4_reg);
}
/*
void SPI_handler_5(volatile float * "PUT VARIABLE HERE") //CH5 - S_S_01, SPARE
{
	Set_ADC_Channel_ext(5, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC5_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	//ATTENTION - IMPLEMENT CONVERSION HERE
}
*/
void SPI_handler_6(volatile float * f32_actuator_feedback) //CH6 - S_A_S0, ACTUATOR POSITION FEEDBACK
{
	Set_ADC_Channel_ext(6, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC6_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	//handle_actuator_feedback(f32_actuator_feedback, u16_ADC6_reg);
	
	volatile float new_f32_actuator_feedback = ((volatile float)u16_ADC6_reg*5.0/4096.0);
	//set new feedback to feedback pointer
	*f32_actuator_feedback = (new_f32_actuator_feedback*100 + 25);
}

void SPI_handler_7(volatile float * f32_motor_current) //CH7 - S_M_I, MOTOR CURRENT
{
	Set_ADC_Channel_ext(7, u8_txBuffer);
	spi_trancieve(u8_txBuffer, u8_rxBuffer, 3, 1);
	u8_rxBuffer[1]&= ~(0b111<<5);
	u16_ADC7_reg = (u8_rxBuffer[1] << 8 ) | u8_rxBuffer[2];
	//ATTENTION - IMPLEMENT CONVERSION HERE
	handle_current_sensor(f32_motor_current, u16_ADC7_reg, 0);
}


///////////////////////  CAN  /////////////////////////


//receiving
void handle_can(volatile ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx) && vals->motor_status != ERR){
		switch (rx->id){
			case DASHBOARD_CAN_ID	: //receiving can messages from the steering wheel
				
				vals->message_mode = CAN ;
				vals->ctrl_type = CURRENT ;
				vals->u16_watchdog_can = WATCHDOG_CAN_RELOAD_VALUE ; // resetting to max value each time a message is received.
				if (rx->data.u8[3] > 8)
				{
					vals->u8_accel_cmd = rx->data.u8[3]/8 ; 
					vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE ;
				}
				
				if (rx->data.u8[4] > 8)
				{
					vals->u8_brake_cmd = rx->data.u8[2]/10 ;
					vals->u16_watchdog_throttle = WATCHDOG_THROTTLE_RELOAD_VALUE ;
				}
				
				if (rx->data.u8[4] <= 8)
				{
					vals->u8_brake_cmd = 0;
				}
				if (rx->data.u8[3] <= 8)
				{
					vals->u8_accel_cmd = 0;
				}
				
			break;
			
			case E_CLUTCH_CAN_ID :
				vals->pwtrain_type = GEAR ;
				vals->u16_motor_speed = rx->data.u16[0] ; //receiving motor speed from encoder from clutch board
				vals->gear_status = rx->data.u8[2] ; //receiving gear status from the clutch board 
			break;
		}
	}
}

//sending
void handle_motor_status_can_msg(volatile ModuleValues_t vals){
	
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	
	txFrame.data.u8[0] = vals.motor_status;
	txFrame.data.i8[1] = (int8_t)(vals.f32_motor_current*10);
	txFrame.data.u16[1] = (uint16_t)(vals.f32_batt_volt*10);
	txFrame.data.u16[2] = (uint16_t)abs((int16_t)vals.f32_energy/100.0) ;
	txFrame.data.u8[6] = (uint8_t)(vals.u16_car_speed*3.6*0.5) ; //sent in km/h
	txFrame.data.u8[7] = vals.u8_motor_temp;
		
	can_send_message(&txFrame);
}

void handle_clutch_cmd_can_msg(volatile ModuleValues_t vals){
	
	txFrame1.id = MOTOR_CL_CMD_CAN_ID;
	txFrame1.length = 1;

	txFrame1.data.u8[0] = vals.gear_required;
		
	can_send_message(&txFrame1);
}

///////////////////  UART  ////////////////////
//RECEIVING
//Send the commands via USB without a whitespace and terminate with \r\n 
void receive_uart(volatile ModuleValues_t * vals)
{
	
	char uart_characters_received[22] = {0};
	uint16_t uart_uint16_received = 0;

	if(uart_AvailableBytes() != 0)
	{
		//IT IS ALWAYS ASSUMMED THAT CONTROLLER IS IN BELT MODE
		
		vals->message_mode = UART;
		vals->pwtrain_type = BELT;
		vals->ctrl_type = CURRENT;
		
		uart0_getln(uart_characters_received, 22);				// reads until \r\n
		uart_uint16_received = atoi(uart_characters_received);	//convert characters to integers ### may want to change the data type later if its fucking up
		
		if((vals->motor_enabled == 0) && strcmp(uart_characters_received, "StartMotorControl") == 0)
		{
			//MOTOR: Start Current Control (ACCELERATION or DEACCELERATION)
			vals->ctrl_type = CURRENT;
			vals->motor_enabled = 1;
		}
		
		if((vals->motor_enabled) && strcmp(uart_characters_received, "StopMotorControl") == 0)
		{
			//MOTOR: Stop Current Control (ACCELERATION or DEACCELERATION)
			vals->motor_enabled = 0;
			vals->u8_accel_cmd = 0;
			vals->u16_watchdog_throttle = 0;
			
		}
		
		if(strcmp(uart_characters_received, "StartClutchControl") == 0)
		{
			//ACTUATOR: Start clutch/actuator control/movement
			vals->clutch_enabled = 0; //disables the actuator P-controller from automatically handing actuator position based on statemachine
			switch(vals->gear_status){
				case NEUTRAL:
					vals->position_uart_instruction = vals->position_neutral;
				break;
				
				case GEAR1:
					vals->position_uart_instruction = vals->position_gear_1;
				break;
				
				case GEAR2:
					vals->position_uart_instruction = vals->position_gear_2;
				break;
			}
		}
		
		if(strcmp(uart_characters_received, "StopClutchControl") == 0)
		{
			//ACTUATOR: Stop clutch/actuator control/movement
			vals->clutch_enabled = 1; //re-enable actuator P-controller
			//vals->u8_actuator_duty_cycle = 50; //
		}
		
		//CURRENT CONTROL: 0=10 deacceleration, 20=10 acceleration, 10=0 constant speed
		if((vals->motor_enabled) && ((uart_uint16_received > 10) & (uart_uint16_received <= 20)))
		{
			//ACCELERATION
			vals->u8_accel_cmd = (uart_uint16_received - 10);
		}
		
		if((vals->motor_enabled) && (uart_uint16_received >= 0) & (uart_uint16_received < 10))
		{
			//DEACCELERATION
			vals->u8_brake_cmd = (UART_CONTROL_OFFSET - uart_uint16_received);		
		}
		
		if((vals->motor_enabled) && (uart_uint16_received == 0))
		{
			//STOP DRIVING MOTOR
			vals->u8_accel_cmd = 0;
			vals->u8_brake_cmd = 0;
			vals->u8_duty_cycle = 50;
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "n") == 0))
		{
			//ACTUATOR: START 50% PWM SIGNAL TO UN-LOCK THE ACTUATOR
			vals->gear_required = NEUTRAL;
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "f") == 0))
		{
			//ACTUATOR: START 50% PWM SIGNAL TO UN-LOCK THE ACTUATOR
			vals->gear_required = GEAR1;
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "s") == 0))
		{
			//ACTUATOR: START 50% PWM SIGNAL TO UN-LOCK THE ACTUATOR
			vals->gear_required = GEAR2;
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "release") == 0))
		{
			//ACTUATOR: STOP 50% PWM SIGNAL TO UN-LOCK THE ACTUATOR
			actuator_pwm(0);
		}

		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "start") == 0))
		{
			//ACTUATOR: START 50% PWM SIGNAL TO UN-LOCK THE ACTUATOR
			actuator_pwm(1);
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "setNeutralPos") == 0))
		{
			vals->gear_required = NEUTRAL;
			actuator_save_position(vals->gear_required, vals->gear_status, vals->f32_actuator_feedback, vals->position_neutral, vals->position_gear_1, vals->position_gear_2);
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "setFirstGearPos") == 0))
		{
			vals->gear_required = GEAR1;
			actuator_save_position(vals->gear_required, vals->gear_status, vals->f32_actuator_feedback, vals->position_neutral, vals->position_gear_1, vals->position_gear_2);
		}
		
		if((vals->clutch_enabled == 0) && (strcmp(uart_characters_received, "setSecondGearPos") == 0))
		{
			vals->gear_required = GEAR2;
			actuator_save_position(vals->gear_required, vals->gear_status, vals->f32_actuator_feedback, vals->position_neutral, vals->position_gear_1, vals->position_gear_2);
		}
		
		if((vals->clutch_enabled == 0) && ((uart_uint16_received > 0) &&  (uart_uint16_received < 1000)))
		{
			vals->position_uart_instruction = uart_uint16_received;
			//uart_flush();
		}
		
		uart_flush();
	}
}
		
		
//sending
//sends motor current and current cmd through USB
void send_uart(volatile ModuleValues_t vals)
{
	printf("\r\n");
	printf("%u", vals.gear_required);
	printf(",");
	printf("%u",vals.gear_status);
	printf(",");
	printf("%u", vals.motor_status);
	printf(",");
	printf("message_mode 0=uart: %u", vals.message_mode);
	printf(",");
	printf("%u",vals.u8_duty_cycle);
	printf(",");
	printf("%u",vals.u8_accel_cmd);
	printf(",");
	printf("%u",vals.u8_brake_cmd);
	printf(",");
	printf("%i",(int16_t)(vals.f32_batt_volt));
	printf(",");
	printf("%i",(int16_t)(vals.f32_motor_current));
	printf(",");
	printf("%i",(int16_t)(vals.f32_batt_current*1000));
	printf(",");
	printf("%i",(int16_t)(vals.f32_actuator_feedback));
	printf(",");
	printf("%u",vals.position_uart_instruction);
	printf(",");
	printf("%i",vals.motor_enabled);
	printf(",");
	printf("%i",vals.clutch_enabled);
	printf(",");
	printf("%u",vals.position_neutral);
	printf(",");
	printf("%u",vals.position_gear_1);
	printf(",");
	printf("%u",vals.position_gear_2);	
	printf(",");
	printf("%i", (int16_t)vals.u8_actuator_duty_cycle);
	printf(",");
	printf("%i", (int16_t)vals.uart_debug);
	
}

///////////////// LED /////////////////////
void manage_LEDs(volatile ModuleValues_t vals)
{	
	switch (vals.motor_status)
	{
		case OFF :
			rgbled_turn_off(LED_GREEN);
			rgbled_turn_on(LED_BLUE);
			if (vals.u16_watchdog_can == 0) //no can messages
			{
				rgbled_turn_on(LED_RED);
			}else{
				rgbled_turn_off(LED_RED);
			}
		break ;
		
		case ENGAGE :
			rgbled_turn_off(LED_RED);
			rgbled_turn_on(LED_GREEN);
			rgbled_turn_on(LED_BLUE);
		break ;
		
		case ACCEL :
			rgbled_turn_off(LED_RED);
			rgbled_turn_off(LED_BLUE);
			rgbled_toggle(LED_GREEN);
		break;
		
		case BRAKE :
			rgbled_turn_off(LED_BLUE);
			rgbled_toggle(LED_GREEN);
			rgbled_toggle(LED_RED);
		break;
		
		case IDLE :
			rgbled_turn_off(LED_RED);
			rgbled_turn_off(LED_BLUE);
			rgbled_turn_on(LED_GREEN);
		break;
		
		case ERR :
			rgbled_turn_off(LED_GREEN);
			rgbled_turn_off(LED_BLUE);
			rgbled_turn_on(LED_RED);
		break;
	}
}