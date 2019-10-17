/*
 * DigiCom.h
 *
 * Created: 04/03/2018 14:13:20
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 


#ifndef DIGICOM_H_
#define DIGICOM_H_

#include "motor_controller_selection.h"
#include "state_machine.h"
#include "UniversalModuleDrivers/can.h"

typedef enum {
	UART_CONTROL_MIN = 0,
	UART_CONTROL_OFFSET = 10,
	UART_CONTROL_MAX = 20
	
} UartControlLimits;


// CAN Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
CanMessage_t rxFrame1;
CanMessage_t txFrame1;

///////////////// PROTOTYPES //////////////s

//SPI CHANNEL HANDLERS
/*
ADDED CODE FOR MOTOR_CONTROL_V2.0
CH0 - S_B_V, BATTERY VOLTAGE
CH1 - S_B_I, BATTERY CURRENT
CH2 - S_M_BELT, POWER TRAIN SELECTION
CH3 - S_M_GEAR, POWER TRAIN SELECTION
CH4 - S_M_T, MOTOR TEMPERATURE
CH5 - S_S_01, SPARE
CH6 - S_A_S0, ACTUATOR POSITION FEEDBACK
CH7 - S_M_I, MOTOR CURRENT
*/
void SPI_handler_0(volatile float * f32_batt_volt);		//CH0 - S_B_V, BATTERY VOLTAGE
void SPI_handler_1(volatile float * f32_batt_current);	//CH1 - S_B_I, BATTERY CURRENT
void SPI_handler_2(volatile PowertrainType_t * pwtrain_type, volatile float * uart_debug_1);		//CH2 - S_M_BELT, POWER TRAIN SELECTION
void SPI_handler_3(volatile PowertrainType_t * pwtrain_type, volatile float * uart_debug_2);		//CH3 - S_M_GEAR, POWER TRAIN SELECTION
void SPI_handler_4(volatile uint8_t * u8_motor_temp);	//CH4 - S_M_T, MOTOR TEMPERATURE 
//void SPI_handler_5(volatile float * "VARIABLE");		//CH5 - S_S_01, SPARE
void SPI_handler_6(volatile float * f32_actuator_feedback);		//CH6 - S_A_S0, ACTUATOR POSITION FEEDBACK
void SPI_handler_7(volatile float * f32_motor_current);	//CH7 - S_M_I, MOTOR CURRENT

//CAN
void handle_motor_status_can_msg(volatile ModuleValues_t vals); //sending status
void handle_clutch_cmd_can_msg(volatile ModuleValues_t vals); //sending required gear to clutch
void handle_can(volatile ModuleValues_t *vals, CanMessage_t *rx); //receiving

//UART
void receive_uart(volatile ModuleValues_t * vals);
void send_uart(volatile ModuleValues_t vals);

//POWER TRAIN SWITCH
void init_com_led_batt_led();
void manage_pwr_train_switch(volatile ModuleValues_t * vals);

//LEDs
void manage_LEDs(volatile ModuleValues_t vals);


#endif /* DIGICOM_H_ */